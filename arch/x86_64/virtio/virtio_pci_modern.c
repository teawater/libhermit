#include "virtio.h"

#define dev_err(...)

#define PCI_STD_RESOURCES 0
#define PCI_STD_RESOURCE_END 5

#if 0

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
u8 pci_find_capability(pci_info_t* pci_info, int cap)
{
	u8 pos;

	pos = __pci_bus_find_cap_start(pci_info->bus, dev->devfn, dev->hdr_type);
	if (pos)
		pos = __pci_find_next_cap(dev->bus, dev->devfn, pos, cap);

	return pos;
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
static inline int virtio_pci_find_capability(struct pci_dev *dev, u8 cfg_type,
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
vp_modern_map_capability(pci_info_t* pci_info, int off,
			 size_t minlen, u32 align, u32 start, u32 size,
			 size_t *len, resource_size_t *pa)
{
	u8 bar;
	u32 offset, length;
	void *p;

	pci_read_config_byte(pci_info->bus, pci_info->slot,
			     off + offsetof(struct virtio_pci_cap, bar),
			     &bar);
	pci_read_config_dword(pci_info->bus, pci_info->slot,
			      off + offsetof(struct virtio_pci_cap, offset),
			      &offset);
	pci_read_config_dword(pci_info->bus, pci_info->slot,
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
	if (!p)
		dev_err(&dev->dev,
			"virtio_pci: unable to map virtio %u@%u on bar %i\n",
			length, offset, bar);
	else if (pa)
		*pa = pci_resource_start(dev, bar) + offset;

	return p;
}

static void vp_set_status(struct virtio_device *vdev, uint8_t status)
{
	outportb(status, vdev->iobase + VIRTIO_PCI_STATUS);
}

#endif

void
virtio_pci_modern_init(struct virtio_device *vdev, pci_info_t* pci_info)
{
	//vdev->set_status = vp_set_status;
}
