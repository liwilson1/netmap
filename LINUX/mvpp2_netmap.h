/*
 * Copyright 2023, Allied Telesis Labs New Zealand, Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <bsd_glue.h>
#include <net/netmap.h>
#include <netmap/netmap_kern.h>
#include <dev/netmap/netmap_mem2.h>

/* Private adapter storage */
struct mvpp2_nm_adapter {
	struct netmap_hw_adapter up;
	bool irqs_enabled[32];
};

/* When in Netmap mode we run the native driver in shared buffer mode which uses
   the first 2 of 8 buffer pools (0=MVPP2_BM_SHORT, 1=MVPP2_BM_LONG) and then
   we use buffer pools 4-7 for netmap (1 per queue/core). Changing a port from
   native mode to netmap mode basically requires pointing the ports percpu
   queues at the appropriate buffer pool. */
#define MVPP2_NETMAP_BMPOOL_FIRST		4
#define MVPP2_NETMAP_BMPOOL_LAST		7
/* Extra netmap buffers required for each of the 4 netmap pools for each MVPP2 instance */
#define MVPP2_NETMAP_POOL_NUM_BUF		2048

#define MVPP2_NETMAP_MAX_RX_PKT_SIZE(na) NETMAP_BUF_SIZE(na) - NETMAP_SLOT_HEADROOM - MVPP2_SKB_SHINFO_SIZE

/* Used functions from below netmap header include */
static u32 mvpp2_thread_read(struct mvpp2 *priv, unsigned int thread, u32 offset);
static u32 mvpp2_thread_read_relaxed(struct mvpp2 *priv, unsigned int thread, u32 offset);
static void mvpp2_thread_write(struct mvpp2 *priv, unsigned int thread, u32 offset, u32 data);
static inline u32 mvpp2_cpu_to_thread(struct mvpp2 *priv, int cpu);
static inline void mvpp2_cause_error(struct net_device *dev, int cause);
static inline void mvpp2_qvec_interrupt_enable(struct mvpp2_queue_vector *qvec);
static inline void mvpp2_qvec_interrupt_disable(struct mvpp2_queue_vector *qvec);
static inline int mvpp2_rxq_received(struct mvpp2_port *port, int rxq_id);
static inline struct mvpp2_rx_desc *mvpp2_rxq_next_desc_get(struct mvpp2_rx_queue *rxq);
static u32 mvpp2_rxdesc_status_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static unsigned long mvpp2_rxdesc_cookie_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static size_t mvpp2_rxdesc_size_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static dma_addr_t mvpp2_rxdesc_dma_addr_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static void mvpp2_rx_error(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static void mvpp2_buff_hdr_pool_put(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc, int pool, u32 rx_status);
static inline void mvpp2_bm_pool_put(struct mvpp2_port *port, int pool, dma_addr_t buf_dma_addr, phys_addr_t buf_phys_addr);
static inline void mvpp2_rxq_status_update(struct mvpp2_port *port, int rxq_id, int used_count, int free_count);
static int mvpp2_bm_switch_buffers(struct mvpp2 *priv, bool percpu);
static int mvpp2_bm_pool_create(struct device *dev, struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool, int size);
static void mvpp2_bm_pool_bufsize_set(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool, int buf_size);
static int mvpp2_check_hw_buf_num(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool);
static int mvpp2_bm_pool_destroy(struct device *dev, struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool);
static void mvpp2_rxq_long_pool_set(struct mvpp2_port *port, int lrxq, int long_pool);
static void mvpp2_rxq_short_pool_set(struct mvpp2_port *port, int lrxq, int short_pool);
static void mvpp2_rxq_offset_set(struct mvpp2_port *port, int prxq, int offset);
static void mvpp2_rxq_drop_pkts(struct mvpp2_port *port, struct mvpp2_rx_queue *rxq);
static void mvpp2_bm_bufs_get_addrs(struct device *dev, struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool, dma_addr_t *dma_addr, phys_addr_t *phys_addr);
static void mvpp2_ingress_enable(struct mvpp2_port *port);
static void mvpp2_ingress_disable(struct mvpp2_port *port);
static enum hrtimer_restart mvpp2_hr_timer_cb(struct hrtimer *timer);


/* Mask/unmask TX interupts - we will process tx completion before sending new packets */
static void mvpp2_netmap_mask_tx_interrupts(struct mvpp2_port *port, unsigned int thread)
{
	if (port->has_tx_irqs) {
		u32 val = mvpp2_thread_read(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id));
		val &= ~MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	}
}
static void mvpp2_netmap_unmask_tx_interrupts(struct mvpp2_port *port, unsigned int thread)
{
	if (port->has_tx_irqs) {
		u32 val = mvpp2_thread_read(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id));
		val |= MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	}
}

/* HRTIMER_MODE_REL_PINNED_SOFT pins a timer to whichever CPU calls
 * hrtimer_start()/hrtimer_cancel(), not to whichever per-thread pcpu struct
 * it operates on. mvpp2_hr_timer_cb() relies on that pinning to recover its
 * own thread index via smp_processor_id(), so these must each run on the
 * CPU that owns the given thread - same reasoning as mvpp2_interrupts_mask()
 * / mvpp2_interrupts_unmask(), which use the same on_each_cpu() pattern. */
static void mvpp2_netmap_tx_timer_start_cpu(void *arg)
{
	struct mvpp2_port *port = arg;
	int cpu = smp_processor_id();
	struct mvpp2_port_pcpu *port_pcpu;
	unsigned int thread;

	if (cpu >= port->priv->nthreads)
		return;

	thread = mvpp2_cpu_to_thread(port->priv, cpu);
	port_pcpu = per_cpu_ptr(port->pcpu, thread);

	/* has_tx_irqs ports never set this timer up at probe time (they
	 * normally rely on the TX-done IRQ instead) - do it now, the first
	 * time it's actually needed. !has_tx_irqs ports already did this at
	 * probe (mvpp2_probe()); re-running hrtimer_init() on a timer that
	 * may still be armed from normal (non-netmap) operation is unsafe,
	 * so leave their existing timer alone and just make sure it's
	 * running below. */
	if (port->has_tx_irqs) {
		hrtimer_init(&port_pcpu->tx_done_timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL_PINNED_SOFT);
		port_pcpu->tx_done_timer.function = mvpp2_hr_timer_cb;
		port_pcpu->dev = port->dev;
	}

	if (!port_pcpu->timer_scheduled) {
		port_pcpu->timer_scheduled = true;
		hrtimer_start(&port_pcpu->tx_done_timer,
			      MVPP2_TXDONE_HRTIMER_PERIOD_NS,
			      HRTIMER_MODE_REL_PINNED_SOFT);
	}
}

static void mvpp2_netmap_tx_timer_stop_cpu(void *arg)
{
	struct mvpp2_port *port = arg;
	int cpu = smp_processor_id();
	struct mvpp2_port_pcpu *port_pcpu;
	unsigned int thread;

	if (cpu >= port->priv->nthreads)
		return;

	thread = mvpp2_cpu_to_thread(port->priv, cpu);
	port_pcpu = per_cpu_ptr(port->pcpu, thread);

	/* Only tear the timer down entirely for has_tx_irqs ports - it's
	 * netmap-only there. For !has_tx_irqs ports it's a native mechanism
	 * that must keep working after netmap deactivates too, so just let
	 * it naturally stop re-arming once idle (mvpp2_hr_timer_cb()) rather
	 * than cancelling it here. */
	if (port->has_tx_irqs) {
		hrtimer_cancel(&port_pcpu->tx_done_timer);
		port_pcpu->timer_scheduled = false;
	}
}

/* TX-done IRQs are masked while netmap is active (see
 * mvpp2_netmap_mask_tx_interrupts()), and even for ports without a TX-done
 * IRQ the trickle-kick in mvpp2_tx() can go a whole netmap session without
 * firing under sustained load. Make sure the per-thread TX-done heartbeat
 * is running regardless of has_tx_irqs, so there is always something
 * reaping completions and waking a stalled queue. */
static void mvpp2_netmap_tx_timer_start(struct mvpp2_port *port)
{
	on_each_cpu(mvpp2_netmap_tx_timer_start_cpu, port, 1);
}

/* Undo mvpp2_netmap_tx_timer_start(). Must be called before TX interrupts
 * are unmasked and native operation resumes, so the timer never outlives
 * netmap deactivation (it touches port_pcpu/port state that native
 * teardown may then free). */
static void mvpp2_netmap_tx_timer_stop(struct mvpp2_port *port)
{
	on_each_cpu(mvpp2_netmap_tx_timer_stop_cpu, port, 1);
}

/* Enable queue interrupts */
static void mvpp2_netmap_qvec_interrupt_enable (struct netmap_adapter *na, int vec)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_queue_vector *qv = port->qvecs + vec;

	if (!mna->irqs_enabled[vec]) {
		nm_prdis("NETMAP[%s:%d] Enabled IRQ\n", ifp->name, vec);
		mna->irqs_enabled[vec] = true;
		mvpp2_qvec_interrupt_enable(qv);
	}
}

/* Disable queue interrupts */
static void mvpp2_netmap_qvec_interrupt_disable (struct netmap_adapter *na, int vec)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_queue_vector *qv = port->qvecs + vec;

	if (mna->irqs_enabled[vec]) {
		nm_prdis("NETMAP[%s] Disable IRQ\n", ifp->name);
		mvpp2_qvec_interrupt_disable(qv);
		mna->irqs_enabled[vec] = false;
	}
}

/* Handle driver intterupts
	MVPP2_ISR_ENABLE_REG 0xF2005420 + (4 * port=0-3) 0xF4005420 + (4 * port=0-3) 0xF6005420 + (4 * port=0-3)
		0:15 Enable port interrupt (write a 1 to enable, read shows status)
		16:31 DisablePortInterupt (write a 1 to disable)

	MVPP2_ISR_RX_TX_CAUSE_REG 0xF2005480 + (4 * port=0-3) 0xF4005480 + (4 * port=0-3) 0xF6005480 + (4 * port=0-3)
		0:7 Rx Occupied Descriptor (1 bit per queue of the Rx queues subgroup)
		8:15 Reserved
		16:23 Tx Occupied Descriptor (1 bit per Tx queue of the Ethernet port)
		24:31 Various Errors
*/
static int mvpp2_netmap_rx_irq(struct mvpp2_queue_vector *qv)
{
	struct netmap_adapter *na = NA(qv->port->dev);
	struct mvpp2_port *port = qv->port;
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());
	uint32_t cause_rx_tx, cause_rx_exc, cause_rx, cause_tx, cause_misc;
	int dummy;

	if (!nm_netmap_on(na))
		return NM_IRQ_PASS;

	/* IRQ cause */
	cause_rx_tx = mvpp2_thread_read_relaxed(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id));

	/* ERROR */
	cause_misc = cause_rx_tx & MVPP2_CAUSE_MISC_SUM_MASK;
	if (cause_misc) {
		nm_prdis("NETMAP[%s:%d]CAUSE_MISC(0x%08x)\n", port->dev->name, thread, cause_misc);
		mvpp2_cause_error(port->dev, cause_misc);
		mvpp2_write(port->priv, MVPP2_ISR_MISC_CAUSE_REG, 0);
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id), cause_rx_tx & ~MVPP2_CAUSE_MISC_SUM_MASK);
	}
	cause_rx_exc = cause_rx_tx & MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK;
	if (cause_rx_exc) {
		/* Run out of descriptors - we need to clear the error - should not happen */
		#define MVPP2_ISR_RX_CAUSE_REG(group)		(0x5500 + 4 * (group))
		nm_prerr("NETMAP[%s:%d] CAUSE_RX_EXCEPTION(0x%08x) RXExcptIC(0x%08x)\n",
			   port->dev->name, thread, cause_rx_exc,
			   mvpp2_thread_read_relaxed(port->priv, thread, MVPP2_ISR_RX_CAUSE_REG(thread)));
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_CAUSE_REG(thread), 0);
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id), cause_rx_tx & ~MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK);
	}

	/* RX */
	cause_rx = cause_rx_tx & MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(port->priv->hw_version);
	if (cause_rx && netmap_rx_irq(port->dev, thread, &dummy) == NM_IRQ_COMPLETED) {
		struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;

		nm_prdis("NETMAP[%s:%d] CAUSE_RXQ_OCCUP_DESC(0x%08x)\n", port->dev->name, thread, cause_rx);

		/* After interface admin down/up irqs_enabled flag may get out of sync.
		 * Interrupts must be enabled since we got here so make sure flag is set. */
		if (!mna->irqs_enabled[thread])
			nm_prdis("NETMAP[%s:%d] RX IRQ: stale irqs_enabled flag corrected\n",
				 port->dev->name, thread);
		mna->irqs_enabled[thread] = true;

		mvpp2_netmap_qvec_interrupt_disable (na, thread);
		// mvpp2_netmap_mask_rx_interrupts(qv);
	}

	/* TX */
	cause_tx = cause_rx_tx & MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
	if (cause_tx) {
		nm_prdis("NETMAP[%s:%d] CAUSE_TXQ_OCCUP_DESC(0x%08x)\n", port->dev->name, thread, cause_tx);
	}

	wmb(); /* Force memory writes to complete */

	return IRQ_HANDLED;
}

static int mvpp2_netmap_txsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = (struct netmap_adapter *)kring->na;
	struct ifnet *ifp = na->ifp;

	nm_prerr("NETMAP[%s] txsync\n", ifp->name);
	BUG(); // TODO
	return 0;
}

static int mvpp2_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	int force_update = (flags & NAF_FORCE_READ) || kring->nr_kflags & NKR_PENDINTR;
	struct netmap_adapter *na = (struct netmap_adapter *)kring->na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_rx_queue *rxq = port->rxqs[kring->ring_id];
	struct netmap_ring *ring = kring->ring;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	u_int nm_i;
	int i;

	if (!netif_carrier_ok(ifp))
	{
		return 0;
	}

	rmb(); /* Force memory reads to complete */

	if (head > lim)
	{
		nm_prerr("NETMAP[%s:%d] head(%u) > lim(%u)\n", ifp->name, rxq->id, head, lim);
		return netmap_ring_reinit(kring);
	}

	/* First part: import newly received packets */
	if (netmap_no_pendintr || force_update) {
		uint32_t hwtail_lim = nm_prev(kring->nr_hwcur, lim);
		int rx_received;
		int rx_complete = 0;
		int rx_packets = 0;
		uint64_t rx_bytes = 0;

		rx_received = mvpp2_rxq_received(port, rxq->id);
		nm_prdis("NETMAP[%s:%d] RX(%s) pkts=%d\n", ifp->name, rxq->id, force_update ? "force": "irq", rx_received);

		nm_i = kring->nr_hwtail;
		while (rx_received && nm_i != hwtail_lim) {
			struct netmap_slot *slot = &ring->slot[nm_i];
			struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
			u32 status = mvpp2_rxdesc_status_get(port, rx_desc);
			int pool = (status & MVPP2_RXD_BM_POOL_ID_MASK) >> MVPP2_RXD_BM_POOL_ID_OFFS;
			int length = mvpp2_rxdesc_size_get(port, rx_desc) - MVPP2_MH_SIZE;
			uint32_t buf_index = (uint32_t)mvpp2_rxdesc_cookie_get(port, rx_desc);
			phys_addr_t paddr;

			if (status & (MVPP2_RXD_BUF_HDR|MVPP2_RXD_ERR_SUMMARY)) {
				nm_prerr("NETMAP[%s:%d] rxsync error(0x%08x)\n", ifp->name, rxq->id, status);
				ifp->stats.rx_errors++;
				mvpp2_rx_error(port, rx_desc);
				if (status & MVPP2_RXD_BUF_HDR)
					mvpp2_buff_hdr_pool_put(port, rx_desc, pool, status);
				else
					mvpp2_bm_pool_put(port, pool, mvpp2_rxdesc_dma_addr_get(port, rx_desc), buf_index);
				rx_received--;
				rx_complete++;
				continue;
			}

			/* Put the received netmap buffer into the slot */
			BUG_ON(buf_index >= na->na_lut.objtotal);
			paddr = na->na_lut.plut[buf_index].paddr + MVPP2_MH_SIZE + NETMAP_SLOT_HEADROOM;
			netmap_sync_map_cpu(na, NULL, (bus_dmamap_t)&paddr, length, NR_RX);
			uint32_t old_index = slot->buf_idx;
			slot->buf_idx = buf_index;
			slot->ll_ofs = MVPP2_MH_SIZE + NETMAP_SLOT_HEADROOM;
			slot->len = length;
			slot->hash = (uint16_t) ((rx_desc->pp22.buf_dma_addr_key_hash >> 40) & 0xFFFFFF);
			slot->flags = 0;

			/* Put the old buffer into the buffer pool */
			BUG_ON(old_index >= na->na_lut.objtotal);
			paddr = na->na_lut.plut[old_index].paddr;
			netmap_sync_map_dev(na, NULL, (bus_dmamap_t)&paddr, NETMAP_BUF_SIZE(na), NR_RX);
			mvpp2_bm_pool_put(port, pool, paddr, old_index);

			rx_packets++;
			rx_bytes += length;
			nm_i = kring->nr_hwtail = nm_next(nm_i, lim);
			rx_received--;
			rx_complete++;
		}

		if (rx_complete) {
			nm_prdis("NETMAP[%s:%d] RXQ Update used:%d free:%d\n", ifp->name, rxq->id, rx_complete, rx_complete);
			mvpp2_rxq_status_update(port, rxq->id, rx_complete, rx_complete);
			if (rx_packets) {
				struct mvpp2_pcpu_stats *stats = this_cpu_ptr(port->stats);
				u64_stats_update_begin(&stats->syncp);
				stats->rx_packets += rx_packets;
				stats->rx_bytes   += rx_bytes;
				u64_stats_update_end(&stats->syncp);
			}
		}

		/* Netmap ring is fully empty and no packets available to process */
		if (kring->nr_hwtail == kring->nr_hwcur && !rx_received) {
			kring->nr_kflags &= ~NKR_PENDINTR;
			mvpp2_netmap_qvec_interrupt_enable (na, kring->ring_id);
		}
	}

	/* Second part: skip past packets that userspace has released */
	nm_i = kring->nr_hwcur;
	for (i = 0; nm_i != head; i++) {
		struct netmap_slot *slot = &ring->slot[nm_i];
		void *addr = NMB(na, slot);

		/* We currently do not do anything here because
		   we allocate enough 'extra' buffers to keep
		   the h/w buffer pool full even when userspace
		   is holding on to a buffer.
		 */
		if (addr == NETMAP_BUF_BASE(na))	/* bad buf */
		{
			printk("NETMAP[%s:%d] Bad buffer @ %p (index %d)\n", ifp->name, rxq->id, addr + nm_get_offset(kring, slot), nm_i);
			return netmap_ring_reinit(kring);
		}
		slot->flags &= ~NS_BUF_CHANGED;
		nm_i = nm_next(nm_i, lim);
	}
	kring->nr_hwcur = head;

	wmb(); /* Force memory writes to complete */

	return 0;
}

static int mvpp2_netmap_change_mtu(struct net_device *dev, int mtu)
{
	struct netmap_adapter *na = NA(dev);

	/* The native drivers behaviour when setting the MTU shuffles queue
	 * pools in several circumstances and switches between per_cpu and
	 * shared pools.  This is problematic for the zero copy support in the
	 * netmap driver as it forces the native driver into shared pool mode
	 * and then changes the netmap enabled interface pools.  Thus having
	 * other code change pools associated with rx queues will break invariants
	 * expected by the netmap driver.
	 *
	 * Thus only allow changing the MTU below the buffer size alloted by netmap
	 * and don't actually alter any hardware settings based on this.
	 */
	if (!IS_ALIGNED(MVPP2_RX_PKT_SIZE(mtu), 8)) {
		netdev_info(dev, "illegal MTU value %d, round to %d\n", mtu,
			    ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8));
		mtu = ALIGN(MVPP2_RX_PKT_SIZE(mtu), 8);
	}

	if (na && mtu > MVPP2_NETMAP_MAX_RX_PKT_SIZE(na)) {
		netdev_err(dev, "Illegal MTU value %d (> %d) for netmap\n",
			   mtu, (int)MVPP2_NETMAP_MAX_RX_PKT_SIZE(na));
		return -EINVAL;
	}

	/* Update mtu field in netdev
	 * Should be fine to leave the buffer size at the netmap size this should
	 * only mean some packets are received which are larger than the mtu.
	 * This also does not do any of the flow control config which the native
	 * driver does, this may need to be revisited.
	 */
	dev->mtu = mtu;

	return 0;
}

static int mvpp2_netmap_users(struct mvpp2 *priv)
{
	int count = 0;
	int i;
	for (i = 0; i < priv->port_count; i++) {
		if (priv->port_list[i] && NA(priv->port_list[i]->dev) && nm_netmap_on(NA(priv->port_list[i]->dev)))
			count++;
	}
	return count;
}

static void mvpp2_netmap_create_pools(struct netmap_adapter *na, int num_buffers)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);
	struct device *dev = port->dev->dev.parent;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_bm_pool *pool;
	int i, j;

	/* Change native driver to shared BM pools (0-2) */
	mvpp2_bm_switch_buffers(priv, false);
	BUG_ON(priv->percpu_pools);

	/* Create BM pools (4-7) for netmap buffers */
	for (i = MVPP2_NETMAP_BMPOOL_FIRST; i < (MVPP2_NETMAP_BMPOOL_FIRST + 4); i++) {
		pool = &priv->bm_pools[i];
		pool->id = i;

		mvpp2_write(priv, MVPP2_BM_INTR_MASK_REG(pool->id), 0);
		mvpp2_write(priv, MVPP2_BM_INTR_CAUSE_REG(pool->id), 0);

		if (mvpp2_bm_pool_create(dev, priv, pool, MVPP2_BM_POOL_SIZE_MAX)) {
			printk("ERROR: Failed to create pool of %d buffers\n", MVPP2_BM_POOL_SIZE_MAX);
			BUG();
		}
		pool->pkt_size = NETMAP_BUF_SIZE(na) - NETMAP_SLOT_HEADROOM - MVPP2_SKB_SHINFO_SIZE;
		mvpp2_bm_pool_bufsize_set(priv, pool, pool->pkt_size + NETMAP_SLOT_HEADROOM);

		for (j=0; j<num_buffers; j++) {
			uint32_t index = netmap_ext_buf_malloc(na);
			BUG_ON(!index);
			void *paddr = (void *)na->na_lut.plut[index].paddr;
			netmap_sync_map_dev(na, NULL, (bus_dmamap_t)&paddr, NETMAP_BUF_SIZE(na), NR_RX);
			mvpp2_bm_pool_put(port, pool->id, (dma_addr_t)paddr, index);
			pool->buf_num += 1;
		}
	}

	/* Debug - dump all pool details */
	for (i = 0; i < MVPP2_BM_MAX_POOLS; i++) {
		pool = &priv->bm_pools[i];
		nm_prdis("POOL:%d size:%d size_bytes:%d buf_num:%d buf_size:%d pkt_size:%d\n        port_map:%08x vaddr:0x%llx\n",
			i, pool->size, pool->size_bytes, pool->buf_num, pool->buf_size, pool->pkt_size, pool->port_map, (unsigned long long)pool->virt_addr);
	}
}

static void mvpp2_netmap_destroy_pools(struct netmap_adapter *na)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);
	struct device *dev = port->dev->dev.parent;
	struct mvpp2 *priv = port->priv;
	int i, j;

	/* Free all the netmap buffers assigned to the pool and destory it */
	for (i = MVPP2_NETMAP_BMPOOL_FIRST; i < (MVPP2_NETMAP_BMPOOL_FIRST + 4); i++) {
		struct mvpp2_bm_pool *pool = &priv->bm_pools[i];
		int buf_num = mvpp2_check_hw_buf_num(priv, pool);
		for (j = 0; j < buf_num; j++) {
			dma_addr_t buf_dma_addr;
			phys_addr_t buf_phys_addr;
			mvpp2_bm_bufs_get_addrs(dev, priv, pool, &buf_dma_addr, &buf_phys_addr);
			netmap_ext_buf_free(na, (uint32_t)buf_phys_addr);
		}
		mvpp2_bm_pool_destroy(dev, priv, &priv->bm_pools[i]);
	}

	/* Change native driver to percpu BM pools */
	mvpp2_bm_switch_buffers(priv, true);
	BUG_ON(!priv->percpu_pools);
}

/* Set the correct rxq offset based on whether netmap is enabled or not */
static void mvpp2_netmap_rxq_offset_set(struct mvpp2_port *port, int prxq)
{
	if (NA(port->dev) && nm_netmap_on(NA(port->dev)))
		mvpp2_rxq_offset_set(port, prxq, NETMAP_SLOT_HEADROOM);
	else
		mvpp2_rxq_offset_set(port, prxq, MVPP2_SKB_HEADROOM);
}

static void mvpp2_netmap_start(struct netmap_adapter *na)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);
	struct mvpp2 *priv = port->priv;
	int i;
	bool running = netif_running(port->dev);

	rtnl_lock();

	/* Use percpu_pools to indicate we do not have netmap buffer pools configured */
	if (priv->percpu_pools) {
		mvpp2_netmap_create_pools (na, MVPP2_NETMAP_POOL_NUM_BUF);
	}

	/* Disable ingress while emptying rings */
	if (running)
		mvpp2_ingress_disable(port);

	/* Use netmap BM pools for this port */
	for (i = 0; i < port->nrxqs; i++) {
		struct mvpp2_rx_queue *rxq = port->rxqs[i];
		mvpp2_rxq_drop_pkts(port, rxq);
		mvpp2_rxq_short_pool_set(port, i, MVPP2_NETMAP_BMPOOL_FIRST + i);
		mvpp2_rxq_long_pool_set(port, i, MVPP2_NETMAP_BMPOOL_FIRST + i);
		mvpp2_rxq_offset_set(port, rxq->id, NETMAP_SLOT_HEADROOM);
	}

	/* Re-enable ingress if interface was running */
	if (running)
		mvpp2_ingress_enable(port);


	rtnl_unlock();
}


static void mvpp2_netmap_stop(struct netmap_adapter *na)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);
	struct mvpp2 *priv = port->priv;
	int i;
	bool running = netif_running(port->dev);

	rtnl_lock();

	/* Disable ingress while emptying rings */
	if (running)
		mvpp2_ingress_disable(port);

	/* Use native BM pools for this port */
	for (i = 0; i < port->nrxqs; i++) {
		struct mvpp2_rx_queue *rxq = port->rxqs[i];
		mvpp2_rxq_drop_pkts(port, rxq);
		mvpp2_rxq_short_pool_set(port, i, 0); /* MVPP2_BM_SHORT */
		mvpp2_rxq_long_pool_set(port, i, 1); /* MVPP2_BM_LONG */
		mvpp2_rxq_offset_set(port, rxq->id, MVPP2_SKB_HEADROOM);
	}

	/* Re-enable ingress if interface was running */
	if (running)
		mvpp2_ingress_enable(port);

	/* Free all netmap buffers and pools if no longer needed */
	if (mvpp2_netmap_users(priv) == 0) {
		mvpp2_netmap_destroy_pools (na);
	}

	rtnl_unlock();
}

static int mvpp2_netmap_reg(struct netmap_adapter *na, int onoff)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct mvpp2_port *port = netdev_priv(na->ifp);
	int r;

	nm_prdis("NETMAP[%s] %s (active=%d)", na->ifp->name, onoff ? "ON" : "OFF", na->active_fds);

	/* Enable or disable */
	if (onoff) {
		/* Configure all queues on first activation for this port */
		if (na->active_fds == 0) {
			/* Setup netmap buffer pools if required and configure port queues to use them */
			mvpp2_netmap_start(na);

			/* We process TX completions before sending new packets */
			for (r = 0; r < port->priv->nthreads; r++) {
				mvpp2_netmap_mask_tx_interrupts(port, r);
				mna->irqs_enabled[r] = true;
			}

			/* Make sure the TX-done heartbeat is running to reap
			 * completions/wake a stalled queue - has_tx_irqs
			 * ports have no other way now that the TX-done IRQ
			 * is masked above */
			mvpp2_netmap_tx_timer_start(port);
		}
		nm_set_native_flags(na);
	} else {
		nm_clear_native_flags(na);
		/* Restore all queues to native operation on last deactivation */
		if (na->active_fds == 0) {
			/* Stop the TX-done heartbeat before native TX
			 * interrupts and teardown resume */
			mvpp2_netmap_tx_timer_stop(port);

			/* Use native buffer pools for port and destroy netmap buffer pools if no longer used */
			mvpp2_netmap_stop(na);

			/* Unmask TX interrupts */
			for (r = 0; r < port->priv->nthreads; r++) {
				mvpp2_netmap_unmask_tx_interrupts(port, r);
				mvpp2_netmap_qvec_interrupt_enable (na, r);
			}
		}
	}

	for (r = 0; r < na->num_rx_rings; r++)
		(void)netmap_reset(na, NR_RX, r, 0);
	for (r = 0; r < na->num_tx_rings; r++)
		(void)netmap_reset(na, NR_TX, r, 0);

	return 0;
}

static int
mvpp2_netmap_config(struct netmap_adapter *na, struct nm_config_info *info)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);

	info->num_tx_rings = port->ntxqs;
	info->num_rx_rings = port->nrxqs;
	info->num_tx_descs = 256;
	info->num_rx_descs = 256;
	info->rx_buf_maxsize = NETMAP_BUF_SIZE(na);

	return 0;
}

static void mvpp2_netmap_attach(struct mvpp2_port *port)
{
	struct netmap_adapter na;

	nm_prdis("NETMAP[%s] attach\n", port->dev->name);

	bzero(&na, sizeof(na));
	na.na_flags = NAF_OFFSETS;
	na.ifp = port->dev;
	na.pdev = port->dev->dev.parent;
	na.num_tx_desc = 256;
	na.num_rx_desc = 256;
	na.nm_register = mvpp2_netmap_reg;
	na.nm_config = mvpp2_netmap_config;
	na.nm_txsync = mvpp2_netmap_txsync;
	na.nm_rxsync = mvpp2_netmap_rxsync;
	na.num_tx_rings = port->ntxqs;
	na.num_rx_rings = port->nrxqs;
	netmap_attach_ext(&na, sizeof(struct mvpp2_nm_adapter), 0);

	nm_prdis("NETMAP[%s] attached\n", port->dev->name);
}

static void mvpp2_netmap_detach(struct mvpp2_port *port)
{
	nm_prdis("NETMAP[%s] detach", port->dev->name);
	netmap_detach(port->dev);
}
