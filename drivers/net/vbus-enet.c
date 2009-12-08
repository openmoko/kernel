/*
 * vbus_enet - A virtualized 802.x network device based on the VBUS interface
 *
 * Copyright (C) 2009 Novell, Gregory Haskins <ghaskins@novell.com>
 *
 * Derived from the SNULL example from the book "Linux Device Drivers" by
 * Alessandro Rubini, Jonathan Corbet, and Greg Kroah-Hartman, published
 * by O'Reilly & Associates.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>

#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ioq.h>
#include <linux/vbus_driver.h>

#include <linux/in6.h>
#include <asm/checksum.h>

#include <linux/venet.h>

MODULE_AUTHOR("Gregory Haskins");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("virtual-ethernet");
MODULE_VERSION("1");

static int rx_ringlen = 256;
module_param(rx_ringlen, int, 0444);
static int tx_ringlen = 256;
module_param(tx_ringlen, int, 0444);
static int sg_enabled = 1;
module_param(sg_enabled, int, 0444);

#define PDEBUG(_dev, fmt, args...) dev_dbg(&(_dev)->dev, fmt, ## args)

#define SG_DESC_SIZE VSG_DESC_SIZE(MAX_SKB_FRAGS)

struct vbus_enet_queue {
	struct ioq              *queue;
	struct ioq_notifier      notifier;
	unsigned long            count;
};

struct vbus_enet_priv {
	spinlock_t                 lock;
	struct net_device         *dev;
	struct vbus_device_proxy  *vdev;
	struct napi_struct         napi;
	struct vbus_enet_queue     rxq;
	struct {
		struct vbus_enet_queue veq;
		struct tasklet_struct  task;
		struct sk_buff_head    outstanding;
	} tx;
	bool                       sg;
	struct {
		bool               enabled;
		char              *pool;
	} pmtd; /* pre-mapped transmit descriptors */
	struct {
		bool                   enabled;
		bool                   linkstate;
		bool                   txc;
		unsigned long          evsize;
		struct vbus_enet_queue veq;
		struct tasklet_struct  task;
		char                  *pool;
	} evq;
	struct {
		bool                   available;
		char                  *pool;
		struct vbus_enet_queue pageq;
	} l4ro;

	struct sk_buff *(*import)(struct vbus_enet_priv *priv,
				  struct ioq_ring_desc *desc);
};

static void vbus_enet_tx_reap(struct vbus_enet_priv *priv);

static struct vbus_enet_priv *
napi_to_priv(struct napi_struct *napi)
{
	return container_of(napi, struct vbus_enet_priv, napi);
}

static int
queue_init(struct vbus_enet_priv *priv,
	   struct vbus_enet_queue *q,
	   const char *name,
	   int qid,
	   size_t ringsize,
	   void (*func)(struct ioq_notifier *))
{
	struct vbus_device_proxy *dev = priv->vdev;
	int ret;
	char _name[64];

	if (name)
		snprintf(_name, sizeof(_name), "%s-%s", priv->dev->name, name);

	ret = vbus_driver_ioq_alloc(dev, name ? _name : NULL, qid, 0,
				    ringsize, &q->queue);
	if (ret < 0)
		panic("ioq_alloc failed: %d\n", ret);

	if (func) {
		q->notifier.signal = func;
		q->queue->notifier = &q->notifier;
	}

	q->count = ringsize;

	return 0;
}

static int
devcall(struct vbus_enet_priv *priv, u32 func, void *data, size_t len)
{
	struct vbus_device_proxy *dev = priv->vdev;

	return dev->ops->call(dev, func, data, len, 0);
}

/*
 * ---------------
 * rx descriptors
 * ---------------
 */

static void
rxdesc_alloc(struct vbus_enet_priv *priv, struct ioq_ring_desc *desc, size_t len)
{
	struct net_device *dev = priv->dev;
	struct sk_buff *skb;

	len += ETH_HLEN;

	skb = netdev_alloc_skb(dev, len + NET_IP_ALIGN);
	BUG_ON(!skb);

	skb_reserve(skb, NET_IP_ALIGN); /* align IP on 16B boundary */

	if (priv->l4ro.available) {
		/*
		 * We will populate an SG descriptor initially with one
		 * IOV filled with an MTU SKB.  If the packet needs to be
		 * larger than MTU, the host will grab pages out of the
		 * page-queue and populate additional IOVs
		 */
		struct venet_sg *vsg = (struct venet_sg *)(unsigned long)desc->cookie;
		struct venet_iov *iov = &vsg->iov[0];

		memset(vsg, 0, SG_DESC_SIZE);

		vsg->cookie  = (u64)(unsigned long)skb;
		vsg->count   = 1;

		iov->ptr     = (u64)__pa(skb->data);
		iov->len     = len;
	} else {
		desc->cookie = (u64)(unsigned long)skb;
		desc->ptr    = cpu_to_le64(__pa(skb->data));
		desc->len    = cpu_to_le64(len); /* total length  */
	}

	desc->valid  = 1;
}

static void
rx_pageq_refill(struct vbus_enet_priv *priv, gfp_t gfp_mask)
{
	struct ioq *ioq = priv->l4ro.pageq.queue;
	struct ioq_iterator iter;
	int ret, added = 0;

	if (ioq_full(ioq, ioq_idxtype_inuse))
		/* nothing to do if the pageq is already fully populated */
		return;

	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_inuse, 0);
	BUG_ON(ret < 0); /* will never fail unless seriously broken */

	ret = ioq_iter_seek(&iter, ioq_seek_tail, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * Now populate each descriptor with an empty page
	 */
	while (!iter.desc->sown) {
		struct page *page = NULL;

		page = alloc_page(gfp_mask);

		if (!page)
			break;

		added = 1;
		iter.desc->cookie = (u64)(unsigned long)page;
		iter.desc->ptr    = cpu_to_le64(__pa(page_address(page)));
		iter.desc->len    = cpu_to_le64(PAGE_SIZE);

		ret = ioq_iter_push(&iter, 0);
		BUG_ON(ret < 0);
	}

	if (added)
		ioq_signal(ioq, 0);
}

static void
rx_setup(struct vbus_enet_priv *priv)
{
	struct ioq *ioq = priv->rxq.queue;
	struct ioq_iterator iter;
	int ret;
	int i = 0;

	/*
	 * We want to iterate on the "valid" index.  By default the iterator
	 * will not "autoupdate" which means it will not hypercall the host
	 * with our changes.  This is good, because we are really just
	 * initializing stuff here anyway.  Note that you can always manually
	 * signal the host with ioq_signal() if the autoupdate feature is not
	 * used.
	 */
	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_valid, 0);
	BUG_ON(ret < 0); /* will never fail unless seriously broken */

	/*
	 * Seek to the tail of the valid index (which should be our first
	 * item, since the queue is brand-new)
	 */
	ret = ioq_iter_seek(&iter, ioq_seek_tail, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * Now populate each descriptor with an empty buffer and mark it valid
	 */
	while (!iter.desc->valid) {
		if (priv->l4ro.available) {
			size_t offset = (i * SG_DESC_SIZE);
			void *addr = &priv->l4ro.pool[offset];

			iter.desc->ptr    = cpu_to_le64(offset);
			iter.desc->cookie = (u64)(unsigned long)addr;
			iter.desc->len    = cpu_to_le64(SG_DESC_SIZE);
		}

		rxdesc_alloc(priv, iter.desc, priv->dev->mtu);

		/*
		 * This push operation will simultaneously advance the
		 * valid-head index and increment our position in the queue
		 * by one.
		 */
		ret = ioq_iter_push(&iter, 0);
		BUG_ON(ret < 0);

		i++;
	}

	if (priv->l4ro.available)
		rx_pageq_refill(priv, GFP_KERNEL);
}

static void
rx_rxq_teardown(struct vbus_enet_priv *priv)
{
	struct ioq *ioq = priv->rxq.queue;
	struct ioq_iterator iter;
	int ret;

	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_valid, 0);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_head, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * free each valid descriptor
	 */
	while (iter.desc->valid) {
		struct sk_buff *skb;

		if (priv->l4ro.available) {
			struct venet_sg *vsg;
			int i;

			vsg = (struct venet_sg *)(unsigned long)iter.desc->cookie;

			/* skip i=0, since that is the skb->data IOV */
			for (i = 1; i < vsg->count; i++) {
				struct venet_iov *iov = &vsg->iov[i];
				struct page *page = (struct page *)(unsigned long)iov->ptr;

				put_page(page);
			}

			skb = (struct sk_buff *)(unsigned long)vsg->cookie;
		} else
			skb = (struct sk_buff *)(unsigned long)iter.desc->cookie;

		iter.desc->valid = 0;
		wmb();

		iter.desc->ptr = 0;
		iter.desc->cookie = 0;

		ret = ioq_iter_pop(&iter, 0);
		BUG_ON(ret < 0);

		dev_kfree_skb(skb);
	}
}

static void
rx_l4ro_teardown(struct vbus_enet_priv *priv)
{
	struct ioq *ioq = priv->l4ro.pageq.queue;
	struct ioq_iterator iter;
	int ret;

	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_inuse, 0);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_head, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * free each valid descriptor
	 */
	while (iter.desc->sown) {
		struct page *page = (struct page *)(unsigned long)iter.desc->cookie;

		iter.desc->valid = 0;
		wmb();

		iter.desc->ptr = 0;
		iter.desc->cookie = 0;

		ret = ioq_iter_pop(&iter, 0);
		BUG_ON(ret < 0);

		put_page(page);
	}

	ioq_put(ioq);
	kfree(priv->l4ro.pool);
}

static void
rx_teardown(struct vbus_enet_priv *priv)
{
	rx_rxq_teardown(priv);

	if (priv->l4ro.available)
		rx_l4ro_teardown(priv);
}

static int
tx_setup(struct vbus_enet_priv *priv)
{
	struct ioq *ioq    = priv->tx.veq.queue;
	struct ioq_iterator iter;
	int i;
	int ret;

	if (!priv->sg)
		/*
		 * There is nothing to do for a ring that is not using
		 * scatter-gather
		 */
		return 0;

	/* pre-allocate our descriptor pool if pmtd is enabled */
	if (priv->pmtd.enabled) {
		struct vbus_device_proxy *dev = priv->vdev;
		size_t poollen = SG_DESC_SIZE * priv->tx.veq.count;
		char *pool;
		int shmid;

		/* pmtdquery will return the shm-id to use for the pool */
		ret = devcall(priv, VENET_FUNC_PMTDQUERY, NULL, 0);
		BUG_ON(ret < 0);

		shmid = ret;

		pool = kzalloc(poollen, GFP_KERNEL | GFP_DMA);
		if (!pool)
			return -ENOMEM;

		priv->pmtd.pool = pool;

		ret = dev->ops->shm(dev, NULL, shmid, 0, pool, poollen,
				    NULL, NULL, 0);
		BUG_ON(ret < 0);
	}

	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_valid, 0);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_set, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * Now populate each descriptor with an empty SG descriptor
	 */
	for (i = 0; i < priv->tx.veq.count; i++) {
		struct venet_sg *vsg;

		if (priv->pmtd.enabled) {
			size_t offset = (i * SG_DESC_SIZE);

			vsg = (struct venet_sg *)&priv->pmtd.pool[offset];
			iter.desc->ptr = cpu_to_le64(offset);
		} else {
			vsg = kzalloc(SG_DESC_SIZE, GFP_KERNEL);
			if (!vsg)
				return -ENOMEM;

			iter.desc->ptr = cpu_to_le64(__pa(vsg));
		}

		iter.desc->cookie = (u64)(unsigned long)vsg;
		iter.desc->len    = cpu_to_le64(SG_DESC_SIZE);

		ret = ioq_iter_seek(&iter, ioq_seek_next, 0, 0);
		BUG_ON(ret < 0);
	}

	return 0;
}

static void
tx_teardown(struct vbus_enet_priv *priv)
{
	struct ioq *ioq = priv->tx.veq.queue;
	struct ioq_iterator iter;
	struct sk_buff *skb;
	int ret;

	/* forcefully free all outstanding transmissions */
	while ((skb = __skb_dequeue(&priv->tx.outstanding)))
		dev_kfree_skb(skb);

	if (!priv->sg)
		/*
		 * There is nothing else to do for a ring that is not using
		 * scatter-gather
		 */
		return;

	if (priv->pmtd.enabled) {
		/*
		 * PMTD mode means we only need to free the pool
		 */
		kfree(priv->pmtd.pool);
		return;
	}

	ret = ioq_iter_init(ioq, &iter, ioq_idxtype_valid, 0);
	BUG_ON(ret < 0);

	/* seek to position 0 */
	ret = ioq_iter_seek(&iter, ioq_seek_set, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * free each valid descriptor
	 */
	while (iter.desc->cookie) {
		struct venet_sg *vsg = (struct venet_sg *)(unsigned long)iter.desc->cookie;

		iter.desc->valid = 0;
		wmb();

		iter.desc->ptr = 0;
		iter.desc->cookie = 0;

		ret = ioq_iter_seek(&iter, ioq_seek_next, 0, 0);
		BUG_ON(ret < 0);

		kfree(vsg);
	}
}

static void
evq_teardown(struct vbus_enet_priv *priv)
{
	if (!priv->evq.enabled)
		return;

	ioq_put(priv->evq.veq.queue);
	kfree(priv->evq.pool);
}

/*
 * Open and close
 */

static int
vbus_enet_open(struct net_device *dev)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);
	int ret;

	ret = devcall(priv, VENET_FUNC_LINKUP, NULL, 0);
	BUG_ON(ret < 0);

	napi_enable(&priv->napi);

	return 0;
}

static int
vbus_enet_stop(struct net_device *dev)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);
	int ret;

	napi_disable(&priv->napi);

	ret = devcall(priv, VENET_FUNC_LINKDOWN, NULL, 0);
	BUG_ON(ret < 0);

	return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
static int
vbus_enet_config(struct net_device *dev, struct ifmap *map)
{
	if (dev->flags & IFF_UP) /* can't act on a running interface */
		return -EBUSY;

	/* Don't allow changing the I/O address */
	if (map->base_addr != dev->base_addr) {
		dev_warn(&dev->dev, "Can't change I/O address\n");
		return -EOPNOTSUPP;
	}

	/* ignore other fields */
	return 0;
}

static void
vbus_enet_schedule_rx(struct vbus_enet_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (napi_schedule_prep(&priv->napi)) {
		/* Disable further interrupts */
		ioq_notify_disable(priv->rxq.queue, 0);
		__napi_schedule(&priv->napi);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int
vbus_enet_change_mtu(struct net_device *dev, int new_mtu)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);
	int ret;

	dev->mtu = new_mtu;

	/*
	 * FLUSHRX will cause the device to flush any outstanding
	 * RX buffers.  They will appear to come in as 0 length
	 * packets which we can simply discard and replace with new_mtu
	 * buffers for the future.
	 */
	ret = devcall(priv, VENET_FUNC_FLUSHRX, NULL, 0);
	BUG_ON(ret < 0);

	vbus_enet_schedule_rx(priv);

	return 0;
}

static struct sk_buff *
vbus_enet_l4ro_import(struct vbus_enet_priv *priv, struct ioq_ring_desc *desc)
{
	struct venet_sg *vsg = (struct venet_sg *)(unsigned long)desc->cookie;
	struct sk_buff *skb = (struct sk_buff *)(unsigned long)vsg->cookie;
	struct skb_shared_info *sinfo = skb_shinfo(skb);
	int i;

	rx_pageq_refill(priv, GFP_ATOMIC);

	if (!vsg->len)
		/*
		 * the device may send a zero-length packet when its
		 * flushing references on the ring.  We can just drop
		 * these on the floor
		 */
		goto fail;

	/* advance only by the linear portion in IOV[0] */
	skb_put(skb, vsg->iov[0].len);

	/* skip i=0, since that is the skb->data IOV */
	for (i = 1; i < vsg->count; i++) {
		struct venet_iov *iov = &vsg->iov[i];
		struct page *page = (struct page *)(unsigned long)iov->ptr;
		skb_frag_t *f = &sinfo->frags[i-1];

		f->page        = page;
		f->page_offset = 0;
		f->size        = iov->len;

		PDEBUG(priv->dev, "SG: Importing %d byte page[%i]\n",
		       f->size, i);

		skb->data_len += f->size;
		skb->len      += f->size;
		skb->truesize += f->size;
		sinfo->nr_frags++;
	}

	if (vsg->flags & VENET_SG_FLAG_NEEDS_CSUM
	    && !skb_partial_csum_set(skb, vsg->csum.start,
				     vsg->csum.offset)) {
		priv->dev->stats.rx_frame_errors++;
		goto fail;
	}

	if (vsg->flags & VENET_SG_FLAG_GSO) {
		PDEBUG(priv->dev, "L4RO packet detected\n");

		switch (vsg->gso.type) {
		case VENET_GSO_TYPE_TCPV4:
			sinfo->gso_type = SKB_GSO_TCPV4;
			break;
		case VENET_GSO_TYPE_TCPV6:
			sinfo->gso_type = SKB_GSO_TCPV6;
			break;
		case VENET_GSO_TYPE_UDP:
			sinfo->gso_type = SKB_GSO_UDP;
			break;
		default:
			PDEBUG(priv->dev, "Illegal L4RO type: %d\n",
			       vsg->gso.type);
			priv->dev->stats.rx_frame_errors++;
			goto fail;
		}

		if (vsg->flags & VENET_SG_FLAG_ECN)
			sinfo->gso_type |= SKB_GSO_TCP_ECN;

		sinfo->gso_size = vsg->gso.size;
		if (sinfo->gso_size == 0) {
			PDEBUG(priv->dev, "Illegal L4RO size: %d\n",
			       vsg->gso.size);
			priv->dev->stats.rx_frame_errors++;
			goto fail;
		}

		/*
		 * Header must be checked, and gso_segs
		 * computed.
		 */
		sinfo->gso_type |= SKB_GSO_DODGY;
		sinfo->gso_segs = 0;
	}

	return skb;

fail:
	dev_kfree_skb(skb);

	return NULL;
}

static struct sk_buff *
vbus_enet_flat_import(struct vbus_enet_priv *priv, struct ioq_ring_desc *desc)
{
	struct sk_buff *skb = (struct sk_buff *)(unsigned long)desc->cookie;

	if (!desc->len) {
		/*
		 * the device may send a zero-length packet when its
		 * flushing references on the ring.  We can just drop
		 * these on the floor
		 */
		dev_kfree_skb(skb);
		return NULL;
	}

	skb_put(skb, le64_to_cpu(desc->len));

	return skb;
}

/*
 * The poll implementation.
 */
static int
vbus_enet_poll(struct napi_struct *napi, int budget)
{
	struct vbus_enet_priv *priv = napi_to_priv(napi);
	int npackets = 0;
	struct ioq_iterator iter;
	int ret;

	PDEBUG(priv->dev, "polling...\n");

	/* We want to iterate on the head of the in-use index */
	ret = ioq_iter_init(priv->rxq.queue, &iter, ioq_idxtype_inuse,
			    IOQ_ITER_AUTOUPDATE);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_head, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * We stop if we have met the quota or there are no more packets.
	 * The EOM is indicated by finding a packet that is still owned by
	 * the south side
	 */
	while ((npackets < budget) && (!iter.desc->sown)) {
		struct sk_buff *skb;

		skb = priv->import(priv, iter.desc);
		if (skb) {
			/* Maintain stats */
			npackets++;
			priv->dev->stats.rx_packets++;
			priv->dev->stats.rx_bytes += skb->len;

			/* Pass the buffer up to the stack */
			skb->dev      = priv->dev;
			skb->protocol = eth_type_trans(skb, priv->dev);
			netif_receive_skb(skb);

			mb();
		}

		/* Grab a new buffer to put in the ring */
		rxdesc_alloc(priv, iter.desc, priv->dev->mtu);

		/* Advance the in-use tail */
		ret = ioq_iter_pop(&iter, 0);
		BUG_ON(ret < 0);
	}

	PDEBUG(priv->dev, "%d packets received\n", npackets);

	/*
	 * If we processed all packets, we're done; tell the kernel and
	 * reenable ints
	 */
	if (ioq_empty(priv->rxq.queue, ioq_idxtype_inuse)) {
		napi_complete(napi);
		ioq_notify_enable(priv->rxq.queue, 0);
		ret = 0;
	} else
		/* We couldn't process everything. */
		ret = 1;

	return ret;
}

/*
 * Transmit a packet (called by the kernel)
 */
static int
vbus_enet_tx_start(struct sk_buff *skb, struct net_device *dev)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);
	struct ioq_iterator    iter;
	int ret;
	unsigned long flags;

	PDEBUG(priv->dev, "sending %d bytes\n", skb->len);

	spin_lock_irqsave(&priv->lock, flags);

	if (ioq_full(priv->tx.veq.queue, ioq_idxtype_valid)) {
		/*
		 * We must flow-control the kernel by disabling the
		 * queue
		 */
		spin_unlock_irqrestore(&priv->lock, flags);
		netif_stop_queue(dev);
		dev_err(&priv->dev->dev, "tx on full queue bug\n");
		return 1;
	}

	/*
	 * We want to iterate on the tail of both the "inuse" and "valid" index
	 * so we specify the "both" index
	 */
	ret = ioq_iter_init(priv->tx.veq.queue, &iter, ioq_idxtype_both,
			    IOQ_ITER_AUTOUPDATE);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_tail, 0, 0);
	BUG_ON(ret < 0);
	BUG_ON(iter.desc->sown);

	if (priv->sg) {
		struct venet_sg *vsg = (struct venet_sg *)(unsigned long)iter.desc->cookie;
		struct scatterlist sgl[MAX_SKB_FRAGS+1];
		struct scatterlist *sg;
		int count, maxcount = ARRAY_SIZE(sgl);

		sg_init_table(sgl, maxcount);

		memset(vsg, 0, sizeof(*vsg));

		vsg->cookie = (u64)(unsigned long)skb;
		vsg->len    = skb->len;

		if (skb->ip_summed == CHECKSUM_PARTIAL) {
			vsg->flags      |= VENET_SG_FLAG_NEEDS_CSUM;
			vsg->csum.start  = skb->csum_start - skb_headroom(skb);
			vsg->csum.offset = skb->csum_offset;
		}

		if (skb_is_gso(skb)) {
			struct skb_shared_info *sinfo = skb_shinfo(skb);

			vsg->flags |= VENET_SG_FLAG_GSO;

			vsg->gso.hdrlen = skb_headlen(skb);
			vsg->gso.size = sinfo->gso_size;
			if (sinfo->gso_type & SKB_GSO_TCPV4)
				vsg->gso.type = VENET_GSO_TYPE_TCPV4;
			else if (sinfo->gso_type & SKB_GSO_TCPV6)
				vsg->gso.type = VENET_GSO_TYPE_TCPV6;
			else if (sinfo->gso_type & SKB_GSO_UDP)
				vsg->gso.type = VENET_GSO_TYPE_UDP;
			else
				panic("Virtual-Ethernet: unknown GSO type " \
				      "0x%x\n", sinfo->gso_type);

			if (sinfo->gso_type & SKB_GSO_TCP_ECN)
				vsg->flags |= VENET_SG_FLAG_ECN;
		}

		count = skb_to_sgvec(skb, sgl, 0, skb->len);

		BUG_ON(count > maxcount);

		for (sg = &sgl[0]; sg; sg = sg_next(sg)) {
			struct venet_iov *iov = &vsg->iov[vsg->count++];

			iov->len = sg->length;
			iov->ptr = (u64)sg_phys(sg);
		}

		iter.desc->len = cpu_to_le64(VSG_DESC_SIZE(vsg->count));

	} else {
		/*
		 * non scatter-gather mode: simply put the skb right onto the
		 * ring.
		 */
		iter.desc->cookie = (u64)(unsigned long)skb;
		iter.desc->len = cpu_to_le64(skb->len);
		iter.desc->ptr = cpu_to_le64(__pa(skb->data));
	}

	iter.desc->valid  = 1;

	priv->dev->stats.tx_packets++;
	priv->dev->stats.tx_bytes += skb->len;

	skb_queue_tail(&priv->tx.outstanding, skb);

	/*
	 * This advances both indexes together implicitly, and then
	 * signals the south side to consume the packet
	 */
	ret = ioq_iter_push(&iter, 0);
	BUG_ON(ret < 0);

	dev->trans_start = jiffies; /* save the timestamp */

	if (ioq_full(priv->tx.veq.queue, ioq_idxtype_valid)) {
		/*
		 * If the queue is congested, we must flow-control the kernel
		 */
		PDEBUG(priv->dev, "backpressure tx queue\n");
		netif_stop_queue(dev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

/* assumes priv->lock held */
static void
vbus_enet_skb_complete(struct vbus_enet_priv *priv, struct sk_buff *skb)
{
	PDEBUG(priv->dev, "completed sending %d bytes\n",
	       skb->len);

	skb_unlink(skb, &priv->tx.outstanding);
	dev_kfree_skb(skb);
}

/*
 * reclaim any outstanding completed tx packets
 *
 * assumes priv->lock held
 */
static struct sk_buff *
vbus_enet_tx_reap_one(struct vbus_enet_priv *priv)
{
	struct sk_buff *skb = NULL;
	struct ioq_iterator iter;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->lock, flags);

	/*
	 * We want to iterate on the head of the valid index, but we
	 * do not want the iter_pop (below) to flip the ownership, so
	 * we set the NOFLIPOWNER option
	 */
	ret = ioq_iter_init(priv->tx.veq.queue, &iter, ioq_idxtype_valid,
			    IOQ_ITER_NOFLIPOWNER);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_head, 0, 0);
	BUG_ON(ret < 0);

	if (iter.desc->valid && !iter.desc->sown) {

		if (priv->sg) {
			struct venet_sg *vsg;

			vsg = (struct venet_sg *)(unsigned long)iter.desc->cookie;
			skb = (struct sk_buff *)(unsigned long)vsg->cookie;
		} else
			skb = (struct sk_buff *)(unsigned long)iter.desc->cookie;

		/* Reset the descriptor */
		iter.desc->valid  = 0;

		/* Advance the valid-index head */
		ret = ioq_iter_pop(&iter, 0);
		BUG_ON(ret < 0);
	}

	/*
	 * If we were previously stopped due to flow control, restart the
	 * processing
	 */
	if (netif_queue_stopped(priv->dev)
	    && !ioq_full(priv->tx.veq.queue, ioq_idxtype_valid)) {
		PDEBUG(priv->dev, "re-enabling tx queue\n");
		netif_wake_queue(priv->dev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return skb;
}

static void
vbus_enet_tx_reap(struct vbus_enet_priv *priv)
{
	struct sk_buff *skb;

	while ((skb = vbus_enet_tx_reap_one(priv))) {
		if (!priv->evq.txc)
			/*
			 * We are responsible for freeing the packet upon
			 * reap if TXC is not enabled
			 */
			vbus_enet_skb_complete(priv, skb);
	}
}

static void
vbus_enet_timeout(struct net_device *dev)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);

	dev_dbg(&dev->dev, "Transmit timeout\n");

	vbus_enet_tx_reap(priv);
}

static void
rx_isr(struct ioq_notifier *notifier)
{
	struct vbus_enet_priv *priv;
	struct net_device  *dev;

	priv = container_of(notifier, struct vbus_enet_priv, rxq.notifier);
	dev = priv->dev;

	if (!ioq_empty(priv->rxq.queue, ioq_idxtype_inuse))
		vbus_enet_schedule_rx(priv);
}

static void
deferred_tx_isr(unsigned long data)
{
	struct vbus_enet_priv *priv = (struct vbus_enet_priv *)data;

	PDEBUG(priv->dev, "deferred_tx_isr\n");

	vbus_enet_tx_reap(priv);

	ioq_notify_enable(priv->tx.veq.queue, 0);
}

static void
tx_isr(struct ioq_notifier *notifier)
{
       struct vbus_enet_priv *priv;

       priv = container_of(notifier, struct vbus_enet_priv, tx.veq.notifier);

       PDEBUG(priv->dev, "tx_isr\n");

       ioq_notify_disable(priv->tx.veq.queue, 0);
       tasklet_schedule(&priv->tx.task);
}

static void
evq_linkstate_event(struct vbus_enet_priv *priv,
		    struct venet_event_header *header)
{
	struct venet_event_linkstate *event =
		(struct venet_event_linkstate *)header;

	switch (event->state) {
	case 0:
		netif_carrier_off(priv->dev);
		break;
	case 1:
		netif_carrier_on(priv->dev);
		break;
	default:
		break;
	}
}

static void
evq_txc_event(struct vbus_enet_priv *priv,
	      struct venet_event_header *header)
{
	struct venet_event_txc *event =
		(struct venet_event_txc *)header;

	vbus_enet_tx_reap(priv);

	vbus_enet_skb_complete(priv, (struct sk_buff *)(unsigned long)event->cookie);
}

static void
deferred_evq_isr(unsigned long data)
{
	struct vbus_enet_priv *priv = (struct vbus_enet_priv *)data;
	int nevents = 0;
	struct ioq_iterator iter;
	int ret;

	PDEBUG(priv->dev, "evq: polling...\n");

	/* We want to iterate on the head of the in-use index */
	ret = ioq_iter_init(priv->evq.veq.queue, &iter, ioq_idxtype_inuse,
			    IOQ_ITER_AUTOUPDATE);
	BUG_ON(ret < 0);

	ret = ioq_iter_seek(&iter, ioq_seek_head, 0, 0);
	BUG_ON(ret < 0);

	/*
	 * The EOM is indicated by finding a packet that is still owned by
	 * the south side
	 */
	while (!iter.desc->sown) {
		struct venet_event_header *header;

		header = (struct venet_event_header *)(unsigned long)iter.desc->cookie;

		switch (header->id) {
		case VENET_EVENT_LINKSTATE:
			evq_linkstate_event(priv, header);
			break;
		case VENET_EVENT_TXC:
			evq_txc_event(priv, header);
			break;
		default:
			panic("venet: unexpected event id:%d of size %d\n",
			      header->id, header->size);
			break;
		}

		memset((void *)(unsigned long)iter.desc->cookie, 0, priv->evq.evsize);

		/* Advance the in-use tail */
		ret = ioq_iter_pop(&iter, 0);
		BUG_ON(ret < 0);

		nevents++;
	}

	PDEBUG(priv->dev, "%d events received\n", nevents);

	ioq_notify_enable(priv->evq.veq.queue, 0);
}

static void
evq_isr(struct ioq_notifier *notifier)
{
       struct vbus_enet_priv *priv;

       priv = container_of(notifier, struct vbus_enet_priv, evq.veq.notifier);

       PDEBUG(priv->dev, "evq_isr\n");

       ioq_notify_disable(priv->evq.veq.queue, 0);
       tasklet_schedule(&priv->evq.task);
}

static int
vbus_enet_sg_negcap(struct vbus_enet_priv *priv)
{
	struct net_device *dev = priv->dev;
	struct venet_capabilities caps;
	int ret;

	memset(&caps, 0, sizeof(caps));

	if (sg_enabled) {
		caps.gid = VENET_CAP_GROUP_SG;
		caps.bits |= (VENET_CAP_SG|VENET_CAP_TSO4|VENET_CAP_TSO6
			      |VENET_CAP_ECN|VENET_CAP_PMTD);
		/* note: exclude UFO for now due to stack bug */
	}

	ret = devcall(priv, VENET_FUNC_NEGCAP, &caps, sizeof(caps));
	if (ret < 0)
		return ret;

	if (caps.bits & VENET_CAP_SG) {
		priv->sg = true;

		dev->features |= NETIF_F_SG|NETIF_F_HW_CSUM|NETIF_F_FRAGLIST;

		if (caps.bits & VENET_CAP_TSO4)
			dev->features |= NETIF_F_TSO;
		if (caps.bits & VENET_CAP_UFO)
			dev->features |= NETIF_F_UFO;
		if (caps.bits & VENET_CAP_TSO6)
			dev->features |= NETIF_F_TSO6;
		if (caps.bits & VENET_CAP_ECN)
			dev->features |= NETIF_F_TSO_ECN;

		if (caps.bits & VENET_CAP_PMTD)
			priv->pmtd.enabled = true;
	}

	return 0;
}

static int
vbus_enet_evq_negcap(struct vbus_enet_priv *priv, unsigned long count)
{
	struct venet_capabilities caps;
	int ret;

	memset(&caps, 0, sizeof(caps));

	caps.gid = VENET_CAP_GROUP_EVENTQ;
	caps.bits |= VENET_CAP_EVQ_LINKSTATE;
	caps.bits |= VENET_CAP_EVQ_TXC;

	ret = devcall(priv, VENET_FUNC_NEGCAP, &caps, sizeof(caps));
	if (ret < 0)
		return ret;

	if (caps.bits) {
		struct vbus_device_proxy *dev = priv->vdev;
		struct venet_eventq_query query;
		size_t                    poollen;
		struct ioq_iterator       iter;
		char                     *pool;
		int                       i;

		priv->evq.enabled = true;

		if (caps.bits & VENET_CAP_EVQ_LINKSTATE) {
			/*
			 * We will assume there is no carrier until we get
			 * an event telling us otherwise
			 */
			netif_carrier_off(priv->dev);
			priv->evq.linkstate = true;
		}

		if (caps.bits & VENET_CAP_EVQ_TXC)
			priv->evq.txc = true;

		memset(&query, 0, sizeof(query));

		ret = devcall(priv, VENET_FUNC_EVQQUERY, &query, sizeof(query));
		if (ret < 0)
			return ret;

		priv->evq.evsize = query.evsize;
		poollen = query.evsize * count;

		pool = kzalloc(poollen, GFP_KERNEL | GFP_DMA);
		if (!pool)
			return -ENOMEM;

		priv->evq.pool = pool;

		ret = dev->ops->shm(dev, NULL, query.dpid, 0,
				    pool, poollen, NULL, NULL, 0);
		if (ret < 0)
			return ret;

		queue_init(priv, &priv->evq.veq, "evq",
			   query.qid, count, evq_isr);

		ret = ioq_iter_init(priv->evq.veq.queue,
				    &iter, ioq_idxtype_valid, 0);
		BUG_ON(ret < 0);

		ret = ioq_iter_seek(&iter, ioq_seek_set, 0, 0);
		BUG_ON(ret < 0);

		/* Now populate each descriptor with an empty event */
		for (i = 0; i < count; i++) {
			size_t offset = (i * query.evsize);
			void *addr = &priv->evq.pool[offset];

			iter.desc->ptr    = cpu_to_le64(offset);
			iter.desc->cookie = (u64)(unsigned long)addr;
			iter.desc->len    = cpu_to_le64(query.evsize);

			ret = ioq_iter_push(&iter, 0);
			BUG_ON(ret < 0);
		}

		/* Finally, enable interrupts */
		tasklet_init(&priv->evq.task, deferred_evq_isr,
			     (unsigned long)priv);
		ioq_notify_enable(priv->evq.veq.queue, 0);
	}

	return 0;
}

static int
vbus_enet_l4ro_negcap(struct vbus_enet_priv *priv, unsigned long count)
{
	struct venet_capabilities caps;
	int ret;

	memset(&caps, 0, sizeof(caps));

	caps.gid = VENET_CAP_GROUP_L4RO;
	caps.bits |= (VENET_CAP_SG|VENET_CAP_TSO4|VENET_CAP_TSO6
		      |VENET_CAP_ECN);

	ret = devcall(priv, VENET_FUNC_NEGCAP, &caps, sizeof(caps));
	if (ret < 0) {
		printk(KERN_ERR "Error negotiating L4RO: %d\n", ret);
		return ret;
	}

	if (caps.bits & VENET_CAP_SG) {
		struct vbus_device_proxy *dev = priv->vdev;
		size_t                    poollen = SG_DESC_SIZE * count;
		struct venet_l4ro_query    query;
		char                     *pool;

		memset(&query, 0, sizeof(query));

		ret = devcall(priv, VENET_FUNC_L4ROQUERY, &query, sizeof(query));
		if (ret < 0) {
			printk(KERN_ERR "Error querying L4RO: %d\n", ret);
			return ret;
		}

		pool = kzalloc(poollen, GFP_KERNEL | GFP_DMA);
		if (!pool)
			return -ENOMEM;

		/*
		 * pre-mapped descriptor pool
		 */
		ret = dev->ops->shm(dev, NULL, query.dpid, 0,
				    pool, poollen, NULL, NULL, 0);
		if (ret < 0) {
			printk(KERN_ERR "Error registering L4RO pool: %d\n",
			       ret);
			kfree(pool);
			return ret;
		}

		/*
		 * page-queue: contains a ring of arbitrary pages for
		 * consumption by the host for when the SG::IOV count exceeds
		 * one MTU frame.  All we need to do is keep it populated
		 * with free pages.
		 */
		queue_init(priv, &priv->l4ro.pageq, "pageq", query.pqid,
			   count, NULL);

		priv->l4ro.pool      = pool;
		priv->l4ro.available = true;
	}

	return 0;
}

static int
vbus_enet_negcap(struct vbus_enet_priv *priv)
{
	int ret;

	ret = vbus_enet_sg_negcap(priv);
	if (ret < 0)
		return ret;

	ret = vbus_enet_evq_negcap(priv, tx_ringlen);
	if (ret < 0)
		return ret;

	ret = vbus_enet_l4ro_negcap(priv, rx_ringlen);
	if (ret < 0)
		return ret;

	return 0;
}

static int vbus_enet_set_tx_csum(struct net_device *dev, u32 data)
{
	struct vbus_enet_priv *priv = netdev_priv(dev);

	if (data && !priv->sg)
		return -ENOSYS;

	return ethtool_op_set_tx_hw_csum(dev, data);
}

static struct ethtool_ops vbus_enet_ethtool_ops = {
	.set_tx_csum = vbus_enet_set_tx_csum,
	.set_sg      = ethtool_op_set_sg,
	.set_tso     = ethtool_op_set_tso,
	.get_link    = ethtool_op_get_link,
};

static const struct net_device_ops vbus_enet_netdev_ops = {
	.ndo_open            = vbus_enet_open,
	.ndo_stop            = vbus_enet_stop,
	.ndo_set_config      = vbus_enet_config,
	.ndo_start_xmit      = vbus_enet_tx_start,
	.ndo_change_mtu	     = vbus_enet_change_mtu,
	.ndo_tx_timeout      = vbus_enet_timeout,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr   = eth_validate_addr,
};

/*
 * This is called whenever a new vbus_device_proxy is added to the vbus
 * with the matching VENET_ID
 */
static int
vbus_enet_probe(struct vbus_device_proxy *vdev)
{
	struct net_device  *dev;
	struct vbus_enet_priv *priv;
	int ret;

	printk(KERN_INFO "VENET: Found new device at %lld\n", vdev->id);

	ret = vdev->ops->open(vdev, VENET_VERSION, 0);
	if (ret < 0)
		return ret;

	dev = alloc_etherdev(sizeof(struct vbus_enet_priv));
	if (!dev)
		return -ENOMEM;

	/*
	 * establish our device-name early so we can incorporate it into
	 * the signal-path names, etc
	 */
	rtnl_lock();

	ret = dev_alloc_name(dev, dev->name);
	if (ret < 0)
		goto out_free;

	priv = netdev_priv(dev);

	spin_lock_init(&priv->lock);
	priv->dev  = dev;
	priv->vdev = vdev;

	ret = vbus_enet_negcap(priv);
	if (ret < 0) {
		printk(KERN_INFO "VENET: Error negotiating capabilities for " \
		       "%lld\n",
		       priv->vdev->id);
		goto out_free;
	}

	if (priv->l4ro.available)
		priv->import = &vbus_enet_l4ro_import;
	else
		priv->import = &vbus_enet_flat_import;

	skb_queue_head_init(&priv->tx.outstanding);

	queue_init(priv, &priv->rxq, "rx", VENET_QUEUE_RX, rx_ringlen,
		   rx_isr);
	queue_init(priv, &priv->tx.veq, "tx", VENET_QUEUE_TX, tx_ringlen,
		   tx_isr);

	rx_setup(priv);
	tx_setup(priv);

	ioq_notify_enable(priv->rxq.queue, 0);  /* enable rx interrupts */

	if (!priv->evq.txc) {
		/*
		 * If the TXC feature is present, we will recieve our
		 * tx-complete notification via the event-channel.  Therefore,
		 * we only enable txq interrupts if the TXC feature is not
		 * present.
		 */
		tasklet_init(&priv->tx.task, deferred_tx_isr,
			     (unsigned long)priv);
		ioq_notify_enable(priv->tx.veq.queue, 0);
	}

	dev->netdev_ops     = &vbus_enet_netdev_ops;
	dev->watchdog_timeo = 5 * HZ;
	SET_ETHTOOL_OPS(dev, &vbus_enet_ethtool_ops);
	SET_NETDEV_DEV(dev, &vdev->dev);

	netif_napi_add(dev, &priv->napi, vbus_enet_poll, 128);

	ret = devcall(priv, VENET_FUNC_MACQUERY, priv->dev->dev_addr, ETH_ALEN);
	if (ret < 0) {
		printk(KERN_INFO "VENET: Error obtaining MAC address for " \
		       "%lld\n",
		       priv->vdev->id);
		goto out_free;
	}

	dev->features |= NETIF_F_HIGHDMA;

	ret = register_netdevice(dev);
	if (ret < 0) {
		printk(KERN_INFO "VENET: error %i registering device \"%s\"\n",
		       ret, dev->name);
		goto out_free;
	}

	rtnl_unlock();

	vdev->priv = priv;

	return 0;

 out_free:
	rtnl_unlock();

	free_netdev(dev);

	return ret;
}

static int
vbus_enet_remove(struct vbus_device_proxy *vdev)
{
	struct vbus_enet_priv *priv = (struct vbus_enet_priv *)vdev->priv;
	struct vbus_device_proxy *dev = priv->vdev;

	unregister_netdev(priv->dev);
	napi_disable(&priv->napi);

	rx_teardown(priv);
	ioq_put(priv->rxq.queue);

	tx_teardown(priv);
	ioq_put(priv->tx.veq.queue);

	if (priv->evq.enabled)
		evq_teardown(priv);

	dev->ops->close(dev, 0);

	free_netdev(priv->dev);

	return 0;
}

/*
 * Finally, the module stuff
 */

static struct vbus_driver_ops vbus_enet_driver_ops = {
	.probe  = vbus_enet_probe,
	.remove = vbus_enet_remove,
};

static struct vbus_driver vbus_enet_driver = {
	.type   = VENET_TYPE,
	.owner  = THIS_MODULE,
	.ops    = &vbus_enet_driver_ops,
};

static __init int
vbus_enet_init_module(void)
{
	printk(KERN_INFO "Virtual Ethernet: Copyright (C) 2009 Novell, Gregory Haskins\n");
	printk(KERN_DEBUG "VENET: Using %d/%d queue depth\n",
	       rx_ringlen, tx_ringlen);
	return vbus_driver_register(&vbus_enet_driver);
}

static __exit void
vbus_enet_cleanup(void)
{
	vbus_driver_unregister(&vbus_enet_driver);
}

module_init(vbus_enet_init_module);
module_exit(vbus_enet_cleanup);

VBUS_DRIVER_AUTOPROBE(VENET_TYPE);
