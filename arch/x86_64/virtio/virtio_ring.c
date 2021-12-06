#include "virtio.h"

#define START_USE(vq)
#define END_USE(vq)
#define LAST_ADD_TIME_UPDATE(vq)
#define LAST_ADD_TIME_CHECK(vq)
#define LAST_ADD_TIME_INVALID(vq)

#define to_vvq(_vq) container_of(_vq, struct vring_virtqueue, vq)

struct vring_desc_extra {
	dma_addr_t addr;		/* Buffer DMA addr. */
	u32 len;			/* Buffer length. */
	u16 flags;			/* Descriptor flags. */
	u16 next;			/* The next desc state in a list. */
};

struct vring_desc_state_split {
	void *data;			/* Data for callback. */
	struct vring_desc *indir_desc;	/* Indirect descriptor, if any. */
};

struct vring_virtqueue {
	struct virtqueue vq;

	/* Is this a packed ring? */
	bool packed_ring;

	/* Is DMA API used? */
	bool use_dma_api;

	/* Can we use weak barriers? */
	bool weak_barriers;

	/* Other side has made a mess, don't try any more. */
	bool broken;

	/* Host supports indirect buffers */
	bool indirect;

	/* Host publishes avail event idx */
	bool event;

	/* Head of free buffer list. */
	unsigned int free_head;
	/* Number we've added since last sync. */
	unsigned int num_added;

	/* Last used index we've seen. */
	u16 last_used_idx;

	/* Hint for event idx: already triggered no need to disable. */
	bool event_triggered;

	union {
		/* Available for split ring */
		struct {
			/* Actual memory layout for this queue. */
			struct vring vring;

			/* Last written value to avail->flags */
			u16 avail_flags_shadow;

			/*
			 * Last written value to avail->idx in
			 * guest byte order.
			 */
			u16 avail_idx_shadow;

			/* Per-descriptor state. */
			struct vring_desc_state_split *desc_state;
			struct vring_desc_extra *desc_extra;

			/* DMA address and size information */
			dma_addr_t queue_dma_addr;
			size_t queue_size_in_bytes;
		} split;

		/* Available for packed ring */
		struct {
			/* Actual memory layout for this queue. */
			struct {
				unsigned int num;
				struct vring_packed_desc *desc;
				struct vring_packed_desc_event *driver;
				struct vring_packed_desc_event *device;
			} vring;

			/* Driver ring wrap counter. */
			bool avail_wrap_counter;

			/* Device ring wrap counter. */
			bool used_wrap_counter;

			/* Avail used flags. */
			u16 avail_used_flags;

			/* Index of the next avail descriptor. */
			u16 next_avail_idx;

			/*
			 * Last written value to driver->flags in
			 * guest byte order.
			 */
			u16 event_flags_shadow;

			/* Per-descriptor state. */
			struct vring_desc_state_packed *desc_state;
			struct vring_desc_extra *desc_extra;

			/* DMA address and size information */
			dma_addr_t ring_dma_addr;
			dma_addr_t driver_event_dma_addr;
			dma_addr_t device_event_dma_addr;
			size_t ring_size_in_bytes;
			size_t event_size_in_bytes;
		} packed;
	};

	/* How to notify other side. FIXME: commonalize hcalls! */
	bool (*notify)(struct virtqueue *vq);

	/* DMA, allocation, and size information */
	bool we_own_ring;

#ifdef DEBUG
	/* They're supposed to lock for us. */
	unsigned int in_use;

	/* Figure out if their kicks are too delayed. */
	bool last_add_time_valid;
	ktime_t last_add_time;
#endif
};

static bool vring_use_dma_api(struct virtio_device *vdev)
{
#if 0
	if (!virtio_has_dma_quirk(vdev))
		return true;

	/* Otherwise, we are left to guess. */
	/*
	 * In theory, it's possible to have a buggy QEMU-supposed
	 * emulated Q35 IOMMU and Xen enabled at the same time.  On
	 * such a configuration, virtio has never worked and will
	 * not work without an even larger kludge.  Instead, enable
	 * the DMA API if we're a Xen guest, which at least allows
	 * all of the sensible Xen configurations to work correctly.
	 */
	if (xen_domain())
		return true;
#endif

	return false;
}

static struct vring_desc_extra *vring_alloc_desc_extra(struct vring_virtqueue *vq,
						       unsigned int num)
{
	struct vring_desc_extra *desc_extra;
	unsigned int i;

	desc_extra = kmalloc_array(num, sizeof(struct vring_desc_extra));
	if (!desc_extra)
		return NULL;

	memset(desc_extra, 0, num * sizeof(struct vring_desc_extra));

	for (i = 0; i < num - 1; i++)
		desc_extra[i].next = i + 1;

	return desc_extra;
}

/* Only available for split ring */
struct virtqueue *__vring_new_virtqueue(unsigned int index,
					struct vring vring,
					struct virtio_device *vdev,
					bool weak_barriers,
					bool context,
					bool (*notify)(struct virtqueue *),
					void (*callback)(struct virtqueue *),
					const char *name)
{
	struct vring_virtqueue *vq;

	if (virtio_has_feature(vdev, VIRTIO_F_RING_PACKED))
		return NULL;

	vq = kmalloc(sizeof(*vq));
	if (!vq)
		return NULL;

	vq->packed_ring = false;
	vq->vq.callback = callback;
	vq->vq.vdev = vdev;
	vq->vq.name = name;
	vq->vq.num_free = vring.num;
	vq->vq.index = index;
	vq->we_own_ring = false;
	vq->notify = notify;
	vq->weak_barriers = weak_barriers;
	vq->broken = false;
	vq->last_used_idx = 0;
	vq->event_triggered = false;
	vq->num_added = 0;
	vq->use_dma_api = vring_use_dma_api(vdev);
#ifdef DEBUG
	vq->in_use = false;
	vq->last_add_time_valid = false;
#endif

	vq->indirect = virtio_has_feature(vdev, VIRTIO_RING_F_INDIRECT_DESC) &&
		!context;
	vq->event = virtio_has_feature(vdev, VIRTIO_RING_F_EVENT_IDX);

	if (virtio_has_feature(vdev, VIRTIO_F_ORDER_PLATFORM))
		vq->weak_barriers = false;

	vq->split.queue_dma_addr = 0;
	vq->split.queue_size_in_bytes = 0;

	vq->split.vring = vring;
	vq->split.avail_flags_shadow = 0;
	vq->split.avail_idx_shadow = 0;

#if 0
	/* No callback?  Tell other side not to bother us. */
	if (!callback) {
		vq->split.avail_flags_shadow |= VRING_AVAIL_F_NO_INTERRUPT;
		if (!vq->event)
			vq->split.vring.avail->flags = cpu_to_virtio16(vdev,
					vq->split.avail_flags_shadow);
	}
#endif

	vq->split.desc_state = kmalloc_array(vring.num,
			sizeof(struct vring_desc_state_split));
	if (!vq->split.desc_state)
		goto err_state;

	vq->split.desc_extra = vring_alloc_desc_extra(vq, vring.num);
	if (!vq->split.desc_extra)
		goto err_extra;

	/* Put everything in free lists. */
	vq->free_head = 0;
	memset(vq->split.desc_state, 0, vring.num *
			sizeof(struct vring_desc_state_split));

	//spin_lock(&vdev->vqs_list_lock);
	//list_add_tail(&vq->vq.list, &vdev->vqs);
	//spin_unlock(&vdev->vqs_list_lock);
	return &vq->vq;

err_extra:
	kfree(vq->split.desc_state);
err_state:
	kfree(vq);
	return NULL;
}

struct virtqueue *
vring_create_virtqueue_split(unsigned int index,
	unsigned int num,
	unsigned int vring_align,
	struct virtio_device *vdev,
	bool weak_barriers,
	bool may_reduce_num,
	bool context,
	bool (*notify)(struct virtqueue *),
	void (*callback)(struct virtqueue *),
	const char *name)
{
	void *queue = NULL;
	uint64_t dma_addr;
	size_t queue_size_in_bytes;
	struct vring vring;
	struct virtqueue *vq;

	if (num & (num - 1))
		return NULL;

	// vring_alloc_queue
	queue = page_alloc(vring_size(num, vring_align), VMA_READ | VMA_WRITE | VMA_CACHEABLE);
	if (!queue)
		return NULL;
	dma_addr = virt_to_phys((size_t)queue);

	queue_size_in_bytes = vring_size(num, vring_align);
	vring_init(&vring, num, queue, vring_align);

	vq = __vring_new_virtqueue(index, vring, vdev, weak_barriers, context, notify, callback, name);
	if (!vq)
		return NULL;

	to_vvq(vq)->split.queue_dma_addr = dma_addr;
	to_vvq(vq)->split.queue_size_in_bytes = queue_size_in_bytes;
	to_vvq(vq)->we_own_ring = true;

	return vq;
}

dma_addr_t virtqueue_get_desc_addr(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	//BUG_ON(!vq->we_own_ring);

	if (vq->packed_ring)
		return vq->packed.ring_dma_addr;

	return vq->split.queue_dma_addr;
}

static inline bool more_used_split(const struct vring_virtqueue *vq)
{
	return vq->last_used_idx != virtio16_to_cpu(vq->vq.vdev,
			vq->split.vring.used->idx);
}

irqreturn_t vring_interrupt(void *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (!more_used_split(vq)) {
		//pr_debug("virtqueue interrupt with no work for %p\n", vq);
		return IRQ_NONE;
	}

	if (unlikely(vq->broken))
		return IRQ_HANDLED;

	/* Just a hint for performance: so it's ok that this can be racy! */
	if (vq->event)
		vq->event_triggered = true;

	//pr_debug("virtqueue callback for %p (%p)\n", vq, vq->vq.callback);
	if (vq->vq.callback)
		vq->vq.callback(&vq->vq);

	return IRQ_HANDLED;
}

/* Map one sg entry. */
static dma_addr_t vring_map_one_sg(const struct vring_virtqueue *vq,
				   struct scatterlist *sg,
				   enum dma_data_direction direction)
{
	return sg->phy;
}

static dma_addr_t vring_map_single(const struct vring_virtqueue *vq,
				   void *cpu_addr, size_t size,
				   enum dma_data_direction direction)
{
	return virt_to_phys((size_t)cpu_addr);
}

static int vring_mapping_error(const struct vring_virtqueue *vq,
			       dma_addr_t addr)
{
	return 0;
}

static inline unsigned int virtqueue_add_desc_split(struct virtqueue *vq,
						    struct vring_desc *desc,
						    unsigned int i,
						    dma_addr_t addr,
						    unsigned int len,
						    u16 flags,
						    bool indirect)
{
	struct vring_virtqueue *vring = to_vvq(vq);
	struct vring_desc_extra *extra = vring->split.desc_extra;
	u16 next;

	desc[i].flags = cpu_to_virtio16(vq->vdev, flags);
	desc[i].addr = cpu_to_virtio64(vq->vdev, addr);
	desc[i].len = cpu_to_virtio32(vq->vdev, len);

	if (!indirect) {
		next = extra[i].next;
		desc[i].next = cpu_to_virtio16(vq->vdev, next);

		extra[i].addr = addr;
		extra[i].len = len;
		extra[i].flags = flags;
	} else
		next = virtio16_to_cpu(vq->vdev, desc[i].next);

	return next;
}

/* dma_unmap_single do nothing */
static void vring_unmap_one_split_indirect(const struct vring_virtqueue *vq,
					   struct vring_desc *desc)
{
}

/* dma_unmap_single do nothing */
static unsigned int vring_unmap_one_split(const struct vring_virtqueue *vq,
					  unsigned int i)
{
	struct vring_desc_extra *extra = vq->split.desc_extra;

	return extra[i].next;
}

static struct scatterlist *sg_next(struct scatterlist *sg)
{
	if (sg->is_last)
		return NULL;

	sg++;

	return sg;
}

int virtqueue_add_split(struct virtqueue *_vq,
			struct scatterlist *sgs[],
			unsigned int total_sg,
			unsigned int out_sgs,
			unsigned int in_sgs,
			void *data,
			void *ctx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	struct scatterlist *sg;
	struct vring_desc *desc;
	unsigned int i, n, avail, descs_used, prev, err_idx;
	int head;
	bool indirect;

	START_USE(vq);

	BUG_ON(data == NULL);
	BUG_ON(ctx && vq->indirect);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return -EIO;
	}

	LAST_ADD_TIME_UPDATE(vq);

	BUG_ON(total_sg == 0);

	head = vq->free_head;

	// XXX: don't want support it now
	/*if (virtqueue_use_indirect(_vq, total_sg))
		desc = alloc_indirect_split(_vq, total_sg, gfp);
	else {*/
		desc = NULL;
		WARN_ON_ONCE(total_sg > vq->split.vring.num && !vq->indirect);
	//}

#if 0
	if (desc) {
		/* Use a single buffer which doesn't continue */
		indirect = true;
		/* Set up rest to use this indirect table. */
		i = 0;
		descs_used = 1;
	} else {
#endif
		indirect = false;
		desc = vq->split.vring.desc;
		i = head;
		descs_used = total_sg;
	//}

	if (vq->vq.num_free < descs_used) {
#if 0
		pr_debug("Can't add buf len %i - avail = %i\n",
			 descs_used, vq->vq.num_free);
#endif
		/* FIXME: for historical reasons, we force a notify here if
		 * there are outgoing parts to the buffer.  Presumably the
		 * host should service the ring ASAP. */
		if (out_sgs)
			vq->notify(&vq->vq);
		if (indirect)
			kfree(desc);
		END_USE(vq);
		return -ENOSPC;
	}

	for (n = 0; n < out_sgs; n++) {
		for (sg = sgs[n]; sg; sg = sg_next(sg)) {
			dma_addr_t addr = vring_map_one_sg(vq, sg, DMA_TO_DEVICE);
			if (vring_mapping_error(vq, addr))
				goto unmap_release;

			prev = i;
			/* Note that we trust indirect descriptor
			 * table since it use stream DMA mapping.
			 */
			i = virtqueue_add_desc_split(_vq, desc, i, addr, sg->length,
						     VRING_DESC_F_NEXT,
						     indirect);
		}
	}
	for (; n < (out_sgs + in_sgs); n++) {
		for (sg = sgs[n]; sg; sg = sg_next(sg)) {
			dma_addr_t addr = vring_map_one_sg(vq, sg, DMA_FROM_DEVICE);
			if (vring_mapping_error(vq, addr))
				goto unmap_release;

			prev = i;
			/* Note that we trust indirect descriptor
			 * table since it use stream DMA mapping.
			 */
			i = virtqueue_add_desc_split(_vq, desc, i, addr,
						     sg->length,
						     VRING_DESC_F_NEXT |
						     VRING_DESC_F_WRITE,
						     indirect);
		}
	}
	/* Last one doesn't continue. */
	desc[prev].flags &= cpu_to_virtio16(_vq->vdev, ~VRING_DESC_F_NEXT);
	if (!indirect && vq->use_dma_api)
		vq->split.desc_extra[prev & (vq->split.vring.num - 1)].flags =
			~VRING_DESC_F_NEXT;

	if (indirect) {
		/* Now that the indirect table is filled in, map it. */
		dma_addr_t addr = vring_map_single(
			vq, desc, total_sg * sizeof(struct vring_desc),
			DMA_TO_DEVICE);
		if (vring_mapping_error(vq, addr))
			goto unmap_release;

		virtqueue_add_desc_split(_vq, vq->split.vring.desc,
					 head, addr,
					 total_sg * sizeof(struct vring_desc),
					 VRING_DESC_F_INDIRECT,
					 false);
	}

	/* We're using some buffers from the free list. */
	vq->vq.num_free -= descs_used;

	/* Update free pointer */
	if (indirect)
		vq->free_head = vq->split.desc_extra[head].next;
	else
		vq->free_head = i;

	/* Store token and indirect buffer state. */
	vq->split.desc_state[head].data = data;
	if (indirect)
		vq->split.desc_state[head].indir_desc = desc;
	else
		vq->split.desc_state[head].indir_desc = ctx;

	/* Put entry in available array (but don't update avail->idx until they
	 * do sync). */
	avail = vq->split.avail_idx_shadow & (vq->split.vring.num - 1);
	vq->split.vring.avail->ring[avail] = cpu_to_virtio16(_vq->vdev, head);

	/* Descriptors and available array need to be set before we expose the
	 * new available array entries. */
	virtio_wmb(vq->weak_barriers);
	vq->split.avail_idx_shadow++;
	vq->split.vring.avail->idx = cpu_to_virtio16(_vq->vdev,
						vq->split.avail_idx_shadow);
	vq->num_added++;

	//pr_debug("Added buffer head %i to %p\n", head, vq);
	END_USE(vq);

	/* This is very unlikely, but theoretically possible.  Kick
	 * just in case. */
	if (unlikely(vq->num_added == (1 << 16) - 1))
		virtqueue_kick(_vq);

	return 0;

unmap_release:
	err_idx = i;

	if (indirect)
		i = 0;
	else
		i = head;

	for (n = 0; n < total_sg; n++) {
		if (i == err_idx)
			break;
		if (indirect) {
			vring_unmap_one_split_indirect(vq, &desc[i]);
			i = virtio16_to_cpu(_vq->vdev, desc[i].next);
		} else
			i = vring_unmap_one_split(vq, i);
	}

	if (indirect)
		kfree(desc);

	END_USE(vq);
	return -ENOMEM;
}

static bool virtqueue_kick_prepare_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 new, old;
	bool needs_kick;

	START_USE(vq);
	/* We need to expose available array entries before checking avail
	 * event. */
	virtio_mb(vq->weak_barriers);

	old = vq->split.avail_idx_shadow - vq->num_added;
	new = vq->split.avail_idx_shadow;
	vq->num_added = 0;

	LAST_ADD_TIME_CHECK(vq);
	LAST_ADD_TIME_INVALID(vq);

	if (vq->event) {
		needs_kick = vring_need_event(virtio16_to_cpu(_vq->vdev,
					vring_avail_event(&vq->split.vring)),
					      new, old);
	} else {
		needs_kick = !(vq->split.vring.used->flags &
					cpu_to_virtio16(_vq->vdev,
						VRING_USED_F_NO_NOTIFY));
	}
	END_USE(vq);
	return needs_kick;
}

/**
 * virtqueue_notify - second half of split virtqueue_kick call.
 * @_vq: the struct virtqueue
 *
 * This does not need to be serialized.
 *
 * Returns false if host notify failed or queue is broken, otherwise true.
 */
bool virtqueue_notify(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (unlikely(vq->broken))
		return false;

	/* Prod other side to tell it about changes. */
	if (!vq->notify(_vq)) {
		vq->broken = true;
		return false;
	}
	return true;
}

bool virtqueue_kick(struct virtqueue *vq)
{
	if (virtqueue_kick_prepare_split(vq))
		return virtqueue_notify(vq);
	return true;
}

int virtqueue_add_outbuf(struct virtqueue *vq,
			 struct scatterlist *sg, unsigned int num,
			 void *data)
{
	return virtqueue_add_split(vq, &sg, num, 1, 0, data, NULL);
}

static void detach_buf_split(struct vring_virtqueue *vq, unsigned int head,
			     void **ctx)
{
	unsigned int i, j;
	__virtio16 nextflag = cpu_to_virtio16(vq->vq.vdev, VRING_DESC_F_NEXT);

	/* Clear data ptr. */
	vq->split.desc_state[head].data = NULL;

	/* Put back on free list: unmap first-level descriptors and find end */
	i = head;

	while (vq->split.vring.desc[i].flags & nextflag) {
		vring_unmap_one_split(vq, i);
		i = vq->split.desc_extra[i].next;
		vq->vq.num_free++;
	}

	vring_unmap_one_split(vq, i);
	vq->split.desc_extra[i].next = vq->free_head;
	vq->free_head = head;

	/* Plus final descriptor */
	vq->vq.num_free++;

	if (vq->indirect) {
		struct vring_desc *indir_desc =
				vq->split.desc_state[head].indir_desc;
		u32 len;

		/* Free the indirect table, if any, now that it's unmapped. */
		if (!indir_desc)
			return;

		len = vq->split.desc_extra[head].len;

		BUG_ON(!(vq->split.desc_extra[head].flags &
				VRING_DESC_F_INDIRECT));
		BUG_ON(len == 0 || len % sizeof(struct vring_desc));

		for (j = 0; j < len / sizeof(struct vring_desc); j++)
			vring_unmap_one_split_indirect(vq, &indir_desc[j]);

		kfree(indir_desc);
		vq->split.desc_state[head].indir_desc = NULL;
	} else if (ctx) {
		*ctx = vq->split.desc_state[head].indir_desc;
	}
}

static void *virtqueue_get_buf_ctx_split(struct virtqueue *_vq,
					 unsigned int *len,
					 void **ctx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	void *ret;
	unsigned int i;
	u16 last_used;

	START_USE(vq);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return NULL;
	}

	if (!more_used_split(vq)) {
		//pr_debug("No more buffers in queue\n");
		END_USE(vq);
		return NULL;
	}

	/* Only get used array entries after they have been exposed by host. */
	virtio_rmb(vq->weak_barriers);

	last_used = (vq->last_used_idx & (vq->split.vring.num - 1));
	i = virtio32_to_cpu(_vq->vdev,
			vq->split.vring.used->ring[last_used].id);
	*len = virtio32_to_cpu(_vq->vdev,
			vq->split.vring.used->ring[last_used].len);

	if (unlikely(i >= vq->split.vring.num)) {
		//BAD_RING(vq, "id %u out of range\n", i);
		return NULL;
	}
	if (unlikely(!vq->split.desc_state[i].data)) {
		//BAD_RING(vq, "id %u is not a head!\n", i);
		return NULL;
	}

	/* detach_buf_split clears data, so grab it now. */
	ret = vq->split.desc_state[i].data;
	detach_buf_split(vq, i, ctx);
	vq->last_used_idx++;
	/* If we expect an interrupt for the next entry, tell host
	 * by writing event index and flush out the write before
	 * the read in the next get_buf call. */
	if (!(vq->split.avail_flags_shadow & VRING_AVAIL_F_NO_INTERRUPT))
		virtio_store_mb(vq->weak_barriers,
				&vring_used_event(&vq->split.vring),
				cpu_to_virtio16(_vq->vdev, vq->last_used_idx));

	LAST_ADD_TIME_INVALID(vq);

	END_USE(vq);
	return ret;
}

void *virtqueue_get_buf(struct virtqueue *_vq, unsigned int *len)
{
	return virtqueue_get_buf_ctx_split(_vq, len, NULL);
}

bool virtqueue_is_broken(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return READ_ONCE(vq->broken);
}
