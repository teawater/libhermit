#include "virtio.h"

#define dev_err(...)

#define PCI_STD_RESOURCES 0
#define PCI_STD_RESOURCE_END 5

#define PCI_STATUS		0x06	/* 16 bits */
#define  PCI_STATUS_CAP_LIST	0x10	/* Support Capability List */
#define PCI_CAPABILITY_LIST	0x34	/* Offset of first capability list entry */
#define PCI_CB_CAPABILITY_LIST	0x14
#define PCI_FIND_CAP_TTL	48
#define  PCI_CAP_ID_VNDR	0x09	/* Vendor-Specific */
#define PCI_CAP_LIST_NEXT	1	/* Next capability in the list */

/*
 * These helpers provide future and backwards compatibility
 * for accessing popular PCI BAR info
 */
#define pci_resource_start(dev, bar)	((dev)->resource[(bar)].start)
#define pci_resource_end(dev, bar)	((dev)->resource[(bar)].end)
#define pci_resource_flags(dev, bar)	((dev)->resource[(bar)].flags)
#define pci_resource_len(dev,bar) \
	((pci_resource_end((dev), (bar)) == 0) ? 0 :	\
							\
	 (pci_resource_end((dev), (bar)) -		\
	  pci_resource_start((dev), (bar)) + 1))

/* This is the PCI capability header: */
struct virtio_pci_cap {
	__u8 cap_vndr;		/* Generic PCI field: PCI_CAP_ID_VNDR */
	__u8 cap_next;		/* Generic PCI field: next ptr. */
	__u8 cap_len;		/* Generic PCI field: capability length */
	__u8 cfg_type;		/* Identifies the structure. */
	__u8 bar;		/* Where to find it. */
	__u8 id;		/* Multiple capabilities of the same type */
	__u8 padding[2];	/* Pad to full dword. */
	__le32 offset;		/* Offset within bar. */
	__le32 length;		/* Length of the structure, in bytes. */
};

/* Fields in VIRTIO_PCI_CAP_COMMON_CFG: */
struct virtio_pci_common_cfg {
	/* About the whole device. */
	__le32 device_feature_select;	/* read-write */
	__le32 device_feature;		/* read-only */
	__le32 guest_feature_select;	/* read-write */
	__le32 guest_feature;		/* read-write */
	__le16 msix_config;		/* read-write */
	__le16 num_queues;		/* read-only */
	__u8 device_status;		/* read-write */
	__u8 config_generation;		/* read-only */

	/* About a specific virtqueue. */
	__le16 queue_select;		/* read-write */
	__le16 queue_size;		/* read-write, power of 2. */
	__le16 queue_msix_vector;	/* read-write */
	__le16 queue_enable;		/* read-write */
	__le16 queue_notify_off;	/* read-only */
	__le32 queue_desc_lo;		/* read-write */
	__le32 queue_desc_hi;		/* read-write */
	__le32 queue_avail_lo;		/* read-write */
	__le32 queue_avail_hi;		/* read-write */
	__le32 queue_used_lo;		/* read-write */
	__le32 queue_used_hi;		/* read-write */
};

#define build_mmio_read(name, size, type, reg, barrier) \
static inline type name(const volatile void __iomem *addr) \
{ type ret; asm volatile("mov" size " %1,%0":reg (ret) \
:"m" (*(volatile type *)addr) barrier); return ret; }

#define build_mmio_write(name, size, type, reg, barrier) \
static inline void name(type val, volatile void __iomem *addr) \
{ asm volatile("mov" size " %0,%1": :reg (val), \
"m" (*(volatile type *)addr) barrier); }

build_mmio_read(readb, "b", unsigned char, "=q", :"memory")
build_mmio_read(readw, "w", unsigned short, "=r", :"memory")
build_mmio_read(readl, "l", unsigned int, "=r", :"memory")
build_mmio_write(writeb, "b", unsigned char, "q", :"memory")
build_mmio_write(writew, "w", unsigned short, "r", :"memory")
build_mmio_write(writel, "l", unsigned int, "r", :"memory")
static void vp_iowrite64_twopart(u64 val, __le32 __iomem *lo,
				 __le32 __iomem *hi)
{
	writel((u32)val, lo);
	writel(val >> 32, hi);
}	

static u8 __pci_find_next_cap_ttl(uint32_t bus, unsigned int devfn,
				  u8 pos, int cap, int *ttl)
{
	u8 id;
	u16 ent;

	pci_bus_read_config_byte(bus, devfn, pos, &pos);

	while ((*ttl)--) {
		if (pos < 0x40)
			break;
		pos &= ~3;
		pci_bus_read_config_word(bus, devfn, pos, &ent);

		id = ent & 0xff;
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos = (ent >> 8);
	}
	return 0;
}

static u8 __pci_find_next_cap(uint32_t bus, unsigned int devfn,
			      u8 pos, int cap)
{
	int ttl = PCI_FIND_CAP_TTL;

	return __pci_find_next_cap_ttl(bus, devfn, pos, cap, &ttl);
}

static u8 __pci_bus_find_cap_start(uint32_t bus,
				   unsigned int devfn, u8 hdr_type)
{
	u16 status;

	pci_bus_read_config_word(bus, devfn, PCI_STATUS, &status);
	if (!(status & PCI_STATUS_CAP_LIST))
		return 0;

	switch (hdr_type) {
	case PCI_HEADER_TYPE_NORMAL:
	case PCI_HEADER_TYPE_BRIDGE:
		return PCI_CAPABILITY_LIST;
	case PCI_HEADER_TYPE_CARDBUS:
		return PCI_CB_CAPABILITY_LIST;
	}

	return 0;
}

/**
 * pci_find_capability - query for devices' capabilities
 * @dev: PCI device to query
 * @cap: capability code
 *
 * Tell if a device supports a given PCI capability.
 * Returns the address of the requested capability structure within the
 * device's PCI configuration space or 0 in case the device does not
 * support it.  Possible values for @cap include:
 *
 *  %PCI_CAP_ID_PM           Power Management
 *  %PCI_CAP_ID_AGP          Accelerated Graphics Port
 *  %PCI_CAP_ID_VPD          Vital Product Data
 *  %PCI_CAP_ID_SLOTID       Slot Identification
 *  %PCI_CAP_ID_MSI          Message Signalled Interrupts
 *  %PCI_CAP_ID_CHSWP        CompactPCI HotSwap
 *  %PCI_CAP_ID_PCIX         PCI-X
 *  %PCI_CAP_ID_EXP          PCI Express
 */
u8 pci_find_capability(pci_info_t* dev, int cap)
{
	u8 pos;

	pos = __pci_bus_find_cap_start(dev->bus, dev->devfn, PCI_HEADER_TYPE_NORMAL);
	if (pos)
		pos = __pci_find_next_cap(dev->bus, dev->devfn, pos, cap);

	return pos;
}

u8 pci_find_next_capability(pci_info_t *dev, u8 pos, int cap)
{
	return __pci_find_next_cap(dev->bus, dev->devfn,
				   pos + PCI_CAP_LIST_NEXT, cap);
}

/**
 * virtio_pci_find_capability - walk capabilities to find device info.
 * @dev: the pci device
 * @cfg_type: the VIRTIO_PCI_CAP_* value we seek
 * @ioresource_types: IORESOURCE_MEM and/or IORESOURCE_IO.
 * @bars: the bitmask of BARs
 *
 * Returns offset of the capability, or 0.
 */
static inline int virtio_pci_find_capability(pci_info_t* dev, u8 cfg_type,
					     u32 ioresource_types, int *bars)
{
	int pos;

	for (pos = pci_find_capability(dev, PCI_CAP_ID_VNDR);
	     pos > 0;
	     pos = pci_find_next_capability(dev, pos, PCI_CAP_ID_VNDR)) {
		u8 type, bar;
		pci_read_config_byte(dev, pos + offsetof(struct virtio_pci_cap,
							 cfg_type),
				     &type);
		pci_read_config_byte(dev, pos + offsetof(struct virtio_pci_cap,
							 bar),
				     &bar);

		/* Ignore structures with reserved BAR values */
		if (bar > 0x5)
			continue;

		if (type == cfg_type) {
			if (pci_resource_len(dev, bar) &&
			    pci_resource_flags(dev, bar) & ioresource_types) {
				*bars |= (1 << bar);
				return pos;
			}
		}
	}
	return 0;
}

static void *ioremap(resource_size_t start, resource_size_t len)
{
	size_t vaddr;
	size_t page_start = PAGE_FLOOR(start);

	len = PAGE_CEIL(len);

	vaddr = vma_alloc(len, VMA_READ|VMA_WRITE);
	if (vaddr == 0)
		return NULL;

	if (page_map(vaddr, page_start, len >> PAGE_BITS, PG_RW|PG_GLOBAL|PG_PCD|PG_NX) != 0)
		return NULL;

	return (void *)(vaddr + start - page_start);
}

/**
 * pci_iomap_range - create a virtual mapping cookie for a PCI BAR
 * @dev: PCI device that owns the BAR
 * @bar: BAR number
 * @offset: map memory at the given offset in BAR
 * @maxlen: max length of the memory to map
 *
 * Using this function you will get a __iomem address to your device BAR.
 * You can access it using ioread*() and iowrite*(). These functions hide
 * the details if this is a MMIO or PIO address space and will just do what
 * you expect from them in the correct way.
 *
 * @maxlen specifies the maximum length to map. If you want to get access to
 * the complete BAR from offset to the end, pass %0 here.
 * */
void __iomem *pci_iomap_range(pci_info_t *dev,
			      int bar,
			      unsigned long offset,
			      unsigned long maxlen)
{
	resource_size_t start = pci_resource_start(dev, bar);
	resource_size_t len = pci_resource_len(dev, bar);
	unsigned long flags = pci_resource_flags(dev, bar);

	if (len <= offset || !start)
		return NULL;
	len -= offset;
	start += offset;
	if (maxlen && len > maxlen)
		len = maxlen;
	//XXX: Do not support IOPORT_MAP
	if (flags & IORESOURCE_IO)
		return NULL;
		//return __pci_ioport_map(dev, start, len);
	if (flags & IORESOURCE_MEM)
		return ioremap(start, len);
	/* What? */
	return NULL;
}

/*
 * vp_modern_map_capability - map a part of virtio pci capability
 * @mdev: the modern virtio-pci device
 * @off: offset of the capability
 * @minlen: minimal length of the capability
 * @align: align requirement
 * @start: start from the capability
 * @size: map size
 * @len: the length that is actually mapped
 * @pa: physical address of the capability
 *
 * Returns the io address of for the part of the capability
 */
static void *
vp_modern_map_capability(pci_info_t* dev, int off,
			 size_t minlen, u32 align, u32 start, u32 size,
			 size_t *len)
{
	u8 bar;
	u32 offset, length;
	void *p;

	pci_read_config_byte(dev,
			     off + offsetof(struct virtio_pci_cap, bar),
			     &bar);
	pci_read_config_dword(dev,
			      off + offsetof(struct virtio_pci_cap, offset),
			      &offset);
	pci_read_config_dword(dev,
			      off + offsetof(struct virtio_pci_cap, length),
			      &length);

	if (length <= start) {
		dev_err("virtio_pci: bad capability len %u (>%u expected)\n",
			length, start);
		return NULL;
	}

	if (length - start < minlen) {
		dev_err("virtio_pci: bad capability len %u (>=%zu expected)\n",
			length, minlen);
		return NULL;
	}

	length -= start;

	if (start + offset < offset) {
		dev_err("virtio_pci: map wrap-around %u+%u\n",
			start, offset);
		return NULL;
	}

	offset += start;

	if (offset & (align - 1)) {
		dev_err("virtio_pci: offset %u not aligned to %u\n",
			offset, align);
		return NULL;
	}

	if (length > size)
		length = size;

	if (len)
		*len = length;

	if (minlen + offset < minlen ||
	    minlen + offset > pci_resource_len(dev, bar)) {
		dev_err(&dev->dev,
			"virtio_pci: map virtio %zu@%u "
			"out of range on bar %i length %lu\n",
			minlen, offset,
			bar, (unsigned long)pci_resource_len(dev, bar));
		return NULL;
	}

	p = pci_iomap_range(dev, bar, offset, length);
	if (!p) {
		dev_err(&dev->dev,
			"virtio_pci: unable to map virtio %u@%u on bar %i\n",
			length, offset, bar);
		return NULL;
	}

	return p;
}

struct virtio_pci_notify_cap {
	struct virtio_pci_cap cap;
	__le32 notify_off_multiplier;	/* Multiplier for queue_notify_off. */
};

static u16 vp_modern_get_queue_notify_off(struct virtio_device *vdev,
					  u16 index)
{
	writew(index, &vdev->cfg->queue_select);

	return readw(&vdev->cfg->queue_notify_off);
}

static int
vp_modern_map_vq_notify(pci_info_t* pci_info, struct virtqueue *vq)
{
	int ret = -EINVAL;
	u16 off = vp_modern_get_queue_notify_off(vq->vdev, vq->index);

	if (vq->vdev->notify_base) {
		/* offset should not wrap */
		if ((u64)off * vq->vdev->notify_offset_multiplier + 2
			> vq->vdev->notify_len)
			goto out;
		vq->priv = vq->vdev->notify_base + off * vq->vdev->notify_offset_multiplier;
	} else {
		vq->priv = vp_modern_map_capability(pci_info,
						    vq->vdev->notify_map_cap,
						    2, 2,
						    off * vq->vdev->notify_offset_multiplier,
						    2, NULL);
		if (!vq->priv)
			goto out;
	}

	ret = 0;
out:
	return ret;
}

static void vp_set_status(struct virtio_device *vdev, uint8_t status)
{
	writeb(status, &vdev->cfg->device_status);
}

static u8 vp_get_status(struct virtio_device *vdev)
{
	return readb(&vdev->cfg->device_status);
}

static u64 vp_get_features(struct virtio_device *vdev)
{
	u64 features;

	writel(0, &vdev->cfg->device_feature_select);
	features = readl(&vdev->cfg->device_feature);
	writel(1, &vdev->cfg->device_feature_select);
	features |= ((u64)readl(&vdev->cfg->device_feature) << 32);

	return features;
}

static void
vp_set_features(struct virtio_device *vdev)
{
	writel(0, &vdev->cfg->guest_feature_select);
	writel((u32)vdev->features, &vdev->cfg->guest_feature);
	writel(1, &vdev->cfg->guest_feature_select);
	writel((u32)(vdev->features >> 32), &vdev->cfg->guest_feature);
}

static bool vp_notify(struct virtqueue *vq)
{
	writew(vq->index, vq->priv);

	return true;
}

static void vp_get(struct virtio_device *vdev, unsigned offset,
		   u8 *buf, unsigned len)
{
	switch (len) {
	case 1: {
		u8 b = readb(vdev->device + offset);
		memcpy(buf, &b, sizeof b);
	}
		break;
	case 2: {
		u16 w = readw(vdev->device + offset);
		memcpy(buf, &w, sizeof w);
	}
		break;
	case 4: {
		u32 l = readl(vdev->device + offset);
		memcpy(buf, &l, sizeof l);
	}
		break;
	case 8: {
		u32 l = readl(vdev->device + offset);
		memcpy(buf, &l, sizeof l);
		l = readl(vdev->device + offset + sizeof l);
		memcpy(buf + sizeof l, &l, sizeof l);
	}
		break;
	}
}

#define CONFIG_X86_L1_CACHE_SHIFT 6
#define L1_CACHE_SHIFT	(CONFIG_X86_L1_CACHE_SHIFT)
#define L1_CACHE_BYTES	(1 << L1_CACHE_SHIFT)
#define SMP_CACHE_BYTES L1_CACHE_BYTES

static struct virtqueue *
vp_setup_vq(pci_info_t* pci_info,
	    struct virtio_device *vdev,
	    int index, void (*callback)(struct virtqueue *vq),
	    const char *name,
	    bool ctx)
{
	struct virtqueue *vq;
	u16 num;
	int err = -ENOENT;

	if (index >= readw(&vdev->cfg->num_queues))
		goto err;

	/* Check if queue is either not available or already active. */
	writew(index, &vdev->cfg->queue_select);
	num = readw(&vdev->cfg->queue_size);
	writew(index, &vdev->cfg->queue_select);
	if (!num || readw(&vdev->cfg->queue_enable))
		goto err;

	if (num & (num - 1)) {
		err = -EINVAL;
		goto err;
	}

	/* create the vring */
	vq = vring_create_virtqueue_split(index, num,
					  SMP_CACHE_BYTES, vdev,
					  true, true, ctx,
					  vp_notify, callback, name);
	if (!vq) {
		err = -ENOMEM;
		goto err;
	}

	/* activate the queue */
	writew(index, &vdev->cfg->queue_select);
	writew(virtqueue_get_vring_size(vq), &vdev->cfg->queue_size);
	writew(index, &vdev->cfg->queue_select);
	vp_iowrite64_twopart(virtqueue_get_desc_addr(vq), &vdev->cfg->queue_desc_lo,
			     &vdev->cfg->queue_desc_hi);
	vp_iowrite64_twopart(virtqueue_get_avail_addr(vq), &vdev->cfg->queue_avail_lo,
			     &vdev->cfg->queue_avail_hi);
	vp_iowrite64_twopart(virtqueue_get_used_addr(vq), &vdev->cfg->queue_used_lo,
			     &vdev->cfg->queue_used_hi);
	writew(index, &vdev->cfg->queue_select);
	writew(1, &vdev->cfg->queue_enable);

	err = vp_modern_map_vq_notify(pci_info, vq);
	if (err)
		goto err;

	return vq;
err:
	return ERR_PTR(err);
}

int
virtio_pci_modern_init(struct virtio_device *vdev, pci_info_t* pci_info)
{
	int ret = -ENXIO;
	int common, notify, device, modern_bars = 0;
	u32 notify_length;
	u32 notify_offset;

	common = virtio_pci_find_capability(pci_info, VIRTIO_PCI_CAP_COMMON_CFG,
					    IORESOURCE_IO | IORESOURCE_MEM,
					    &modern_bars);
	if (!common)
		goto out;
	
	notify = virtio_pci_find_capability(pci_info, VIRTIO_PCI_CAP_NOTIFY_CFG,
					    IORESOURCE_IO | IORESOURCE_MEM,
					    &modern_bars);
	if (!notify)
		goto out;

	device = virtio_pci_find_capability(pci_info, VIRTIO_PCI_CAP_DEVICE_CFG,
					    IORESOURCE_IO | IORESOURCE_MEM,
					    &modern_bars);
	if (!device)
		goto out;

	vdev->cfg = vp_modern_map_capability(pci_info, common,
					     sizeof(struct virtio_pci_common_cfg),
					     4, 0,
					     sizeof(struct virtio_pci_common_cfg),
					     NULL);
	if (!vdev->cfg)
		goto out;

	vdev->device = vp_modern_map_capability(pci_info, device,
						0, 4, 0, PAGE_SIZE,
						&vdev->device_len);
	if (!vdev->device)
		goto out;

	/* Read notify_off_multiplier from config space. */
	pci_read_config_dword(pci_info,
			      notify + offsetof(struct virtio_pci_notify_cap,
						notify_off_multiplier),
			      &vdev->notify_offset_multiplier);
	/* Read notify length and offset from config space. */
	pci_read_config_dword(pci_info,
			      notify + offsetof(struct virtio_pci_notify_cap,
						cap.length),
			      &notify_length);

	pci_read_config_dword(pci_info,
			      notify + offsetof(struct virtio_pci_notify_cap,
						cap.offset),
			      &notify_offset);

	/* We don't know how many VQs we'll map, ahead of the time.
	 * If notify length is small, map it all now.
	 * Otherwise, map each VQ individually later.
	 */
	if ((u64)notify_length + (notify_offset % PAGE_SIZE) <= PAGE_SIZE) {
		vdev->notify_base = vp_modern_map_capability(pci_info, notify,
							     2, 2,
							     0, notify_length,
							     &vdev->notify_len);
		if (!vdev->notify_base)
			goto out;
	} else {
		vdev->notify_map_cap = notify;
	}

	vdev->set_status = vp_set_status;
	vdev->get_status = vp_get_status;
	vdev->get_features = vp_get_features;
	vdev->set_features = vp_set_features;
	vdev->get = vp_get;
	vdev->setup_vq = vp_setup_vq;

	ret = 0;
out:
	return ret;
}
