/*
 * Copyright (c) 2010-2015, Stefan Lankes, RWTH Aachen University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the University nor the names of its contributors
 *      may be used to endorse or promote products derived from this
 *      software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <hermit/stdio.h>
#include <hermit/string.h>
#include <hermit/errno.h>
#include <hermit/logging.h>
#include <asm/irqflags.h>
#include <asm/io.h>

#include <asm/pci.h>
#ifdef WITH_PCI_IDS
#include "pcihdr.h"
#endif

/*
 * PCI configuration registers
 */
#define	PCI_CFID	0x00	/* Configuration ID */
#define	PCI_CFCS	0x04	/* Configurtion Command/Status */
#define	PCI_CFRV	0x08	/* Configuration Revision */
#define	PCI_CFLT	0x0c	/* Configuration Latency Timer */
#define	PCI_CBIO	0x10	/* Configuration Base IO Address */
#define PCI_SUBSYSTEM_VENDOR_ID	0x2C	/* Configuration Subsystem Id & Subsystem Vendor Id */
#define	PCI_INTERRUPT_LINE	0x3c	/* Configuration Interrupt */
#define	PCI_INTERRUPT_PIN	0x3c
#define	PCI_CFDA	0x40	/* Configuration Driver Area */

#define PHYS_IO_MEM_START	0
#define	PCI_MEM			0
#define	PCI_INTA		0
#define PCI_NSLOTS		22
#define PCI_NBUS		0

#define	PCI_CONF_ADDR_REG	0xcf8
#define	PCI_CONF_FRWD_REG	0xcf8
#define	PCI_CONF_DATA_REG	0xcfc

#define PCI_IO_CONF_START	0xc000

#define MAX_BUS			1
#define MAX_SLOTS		32

static uint32_t mechanism = 0;
static uint32_t adapters[MAX_BUS][MAX_SLOTS] = {[0 ... MAX_BUS-1][0 ... MAX_SLOTS-1] = -1};

static void pci_conf_write(uint32_t bus, uint32_t slot, uint32_t off, uint32_t val)
{
	if (mechanism == 1) {
		outportl(PCI_CONF_FRWD_REG, bus);
		outportl(PCI_CONF_ADDR_REG, 0xf0);
		outportl(PCI_IO_CONF_START | (slot << 8) | off, val);
	} else {
		outportl(PCI_CONF_ADDR_REG,
		      (0x80000000 | (bus << 16) | (slot << 11) | off));
		outportl(PCI_CONF_DATA_REG, val);
	}
}

static uint32_t pci_conf_read(uint32_t bus, uint32_t slot, uint32_t off)
{
	uint32_t data = -1;

	outportl(PCI_CONF_ADDR_REG,
	      (0x80000000 | (bus << 16) | (slot << 11) | off));
	data = inportl(PCI_CONF_DATA_REG);

	if ((data == 0xffffffff) && (slot < 0x10)) {
		outportl(PCI_CONF_FRWD_REG, bus);
		outportl(PCI_CONF_ADDR_REG, 0xf0);
		data = inportl(PCI_IO_CONF_START | (slot << 8) | off);
		if (data == 0xffffffff)
			return data;
		if (!mechanism)
			mechanism = 1;
	} else if (!mechanism)
		mechanism = 2;

	return data;
}

static inline uint32_t pci_subid(uint32_t bus, uint32_t slot)
{
	return pci_conf_read(bus, slot, PCI_SUBSYSTEM_VENDOR_ID);
}

static inline uint32_t pci_what_irq(uint32_t bus, uint32_t slot)
{
	return pci_conf_read(bus, slot, PCI_INTERRUPT_LINE) & 0xFF;
}

static inline uint32_t pci_what_iobase(uint32_t bus, uint32_t slot, uint32_t nr)
{
	return pci_conf_read(bus, slot, PCI_CBIO + nr*4) & 0xFFFFFFFC;
}

static inline void pci_bus_master(uint32_t bus, uint32_t slot)
{
	// set the device to a bus master

	uint32_t cmd = pci_conf_read(bus, slot, PCI_CFCS) | 0x4;
	pci_conf_write(bus, slot, PCI_CFCS, cmd);
}

static inline uint32_t pci_what_size(uint32_t bus, uint32_t slot, uint32_t nr)
{
	uint32_t tmp, ret;

	// backup the original value
	tmp = pci_conf_read(bus, slot, PCI_CBIO + nr*4);

	// determine size
	pci_conf_write(bus, slot, PCI_CBIO + nr*4, 0xFFFFFFFF);
	ret = ~pci_conf_read(bus, slot, PCI_CBIO + nr*4) + 1;

	// restore original value
	pci_conf_write(bus, slot, PCI_CBIO + nr*4, tmp);

	return ret;
}

int pci_init(void)
{
	uint32_t slot, bus;

	for (bus = 0; bus < MAX_BUS; bus++)
		for (slot = 0; slot < MAX_SLOTS; slot++)
			adapters[bus][slot] = pci_conf_read(bus, slot, PCI_CFID);

	return 0;
}

#define PCI_COMMAND		0x04	/* 16 bits */
#define PCI_COMMAND_IO		0x1	/* Enable response in I/O space */
#define PCI_COMMAND_MEMORY	0x2	/* Enable response in Memory space */
#define PCI_COMMAND_DECODE_ENABLE	(PCI_COMMAND_MEMORY | PCI_COMMAND_IO)
/*
 * Base addresses specify locations in memory or I/O space.
 * Decoded size can be determined by writing a value of
 * 0xffffffff to the register, and reading it back.  Only
 * 1 bits are decoded.
 */
#define PCI_BASE_ADDRESS_0	0x10	/* 32 bits */
#define PCI_BASE_ADDRESS_1	0x14	/* 32 bits [htype 0,1 only] */
#define PCI_BASE_ADDRESS_2	0x18	/* 32 bits [htype 0 only] */
#define PCI_BASE_ADDRESS_3	0x1c	/* 32 bits */
#define PCI_BASE_ADDRESS_4	0x20	/* 32 bits */
#define PCI_BASE_ADDRESS_5	0x24	/* 32 bits */
#define  PCI_BASE_ADDRESS_SPACE		0x01	/* 0 = memory, 1 = I/O */
#define  PCI_BASE_ADDRESS_SPACE_IO	0x01
#define  PCI_BASE_ADDRESS_SPACE_MEMORY	0x00
#define  PCI_BASE_ADDRESS_MEM_TYPE_MASK	0x06
#define  PCI_BASE_ADDRESS_MEM_TYPE_32	0x00	/* 32 bit address */
#define  PCI_BASE_ADDRESS_MEM_TYPE_1M	0x02	/* Below 1M [obsolete] */
#define  PCI_BASE_ADDRESS_MEM_TYPE_64	0x04	/* 64 bit address */
#define  PCI_BASE_ADDRESS_MEM_PREFETCH	0x08	/* prefetchable? */
#define  PCI_BASE_ADDRESS_MEM_MASK	(~0x0fUL)
#define  PCI_BASE_ADDRESS_IO_MASK	(~0x03UL)
/* bit 1 is reserved if address_space = 1 */

/* Header type 0 (normal devices) */
#define PCI_CARDBUS_CIS		0x28
#define PCI_SUBSYSTEM_ID	0x2e
#define PCI_ROM_ADDRESS		0x30	/* Bits 31..11 are address, 10..1 reserved */
#define PCI_ROM_ADDRESS_ENABLE	0x01
#define PCI_ROM_ADDRESS_MASK	(~0x7ffU)

#define IO_SPACE_LIMIT 0xffffffff

#define pci_info(...)
#define pci_err(...)

enum pci_bar_type {
	pci_bar_unknown,	/* Standard PCI BAR probe */
	pci_bar_io,		/* An I/O port BAR */
	pci_bar_mem32,		/* A 32-bit memory BAR */
	pci_bar_mem64,		/* A 64-bit memory BAR */
};

struct pci_bus_region {
	pci_bus_addr_t	start;
	pci_bus_addr_t	end;
};

static inline unsigned long decode_bar(u32 bar)
{
	u32 mem_type;
	unsigned long flags;

	if ((bar & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
		flags = bar & ~PCI_BASE_ADDRESS_IO_MASK;
		flags |= IORESOURCE_IO;
		return flags;
	}

	flags = bar & ~PCI_BASE_ADDRESS_MEM_MASK;
	flags |= IORESOURCE_MEM;
	if (flags & PCI_BASE_ADDRESS_MEM_PREFETCH)
		flags |= IORESOURCE_PREFETCH;

	mem_type = bar & PCI_BASE_ADDRESS_MEM_TYPE_MASK;
	switch (mem_type) {
	case PCI_BASE_ADDRESS_MEM_TYPE_32:
		break;
	case PCI_BASE_ADDRESS_MEM_TYPE_1M:
		/* 1M mem BAR treated as 32-bit BAR */
		break;
	case PCI_BASE_ADDRESS_MEM_TYPE_64:
		flags |= IORESOURCE_MEM_64;
		break;
	default:
		/* mem unknown type treated as 32-bit BAR */
		break;
	}
	return flags;
}

static u64 pci_size(u64 base, u64 maxbase, u64 mask)
{
	u64 size = mask & maxbase;	/* Find the significant bits */
	if (!size)
		return 0;

	/*
	 * Get the lowest of them to find the decode size, and from that
	 * the extent.
	 */
	size = size & ~(size-1);

	/*
	 * base == maxbase can be valid only if the BAR has already been
	 * programmed with all 1s.
	 */
	if (base == maxbase && ((base | (size - 1)) & mask) != mask)
		return 0;

	return size;
}

/**
 * __pci_read_base - Read a PCI BAR
 * @dev: the PCI device
 * @type: type of the BAR
 * @res: resource buffer to be filled in
 * @pos: BAR position in the config space
 *
 * Returns 1 if the BAR is 64-bit, or 0 if 32-bit.
 */
int __pci_read_base(pci_info_t* dev, enum pci_bar_type type,
		    struct resource *res, unsigned int pos)
{
	u32 l = 0, sz = 0, mask;
	u64 l64, sz64, mask64;
	u16 orig_cmd;
	struct pci_bus_region region, inverted_region;
	bool mmio_always_on = false;

	mask = type ? PCI_ROM_ADDRESS_MASK : ~0;

	/* No printks while decoding is disabled! */
	if (!mmio_always_on) {
		pci_read_config_word(dev, PCI_COMMAND, &orig_cmd);
		if (orig_cmd & PCI_COMMAND_DECODE_ENABLE) {
			pci_write_config_word(dev, PCI_COMMAND,
				orig_cmd & ~PCI_COMMAND_DECODE_ENABLE);
		}
	}

	pci_read_config_dword(dev, pos, &l);
	pci_write_config_dword(dev, pos, l | mask);
	pci_read_config_dword(dev, pos, &sz);
	pci_write_config_dword(dev, pos, l);

	/*
	 * All bits set in sz means the device isn't working properly.
	 * If the BAR isn't implemented, all bits must be 0.  If it's a
	 * memory BAR or a ROM, bit 0 must be clear; if it's an io BAR, bit
	 * 1 must be clear.
	 */
	if (sz == 0xffffffff)
		sz = 0;

	/*
	 * I don't know how l can have all bits set.  Copied from old code.
	 * Maybe it fixes a bug on some ancient platform.
	 */
	if (l == 0xffffffff)
		l = 0;

	if (type == pci_bar_unknown) {
		res->flags = decode_bar(l);
		res->flags |= IORESOURCE_SIZEALIGN;
		if (res->flags & IORESOURCE_IO) {
			l64 = l & PCI_BASE_ADDRESS_IO_MASK;
			sz64 = sz & PCI_BASE_ADDRESS_IO_MASK;
			mask64 = PCI_BASE_ADDRESS_IO_MASK & (u32)IO_SPACE_LIMIT;
		} else {
			l64 = l & PCI_BASE_ADDRESS_MEM_MASK;
			sz64 = sz & PCI_BASE_ADDRESS_MEM_MASK;
			mask64 = (u32)PCI_BASE_ADDRESS_MEM_MASK;
		}
	} else {
		if (l & PCI_ROM_ADDRESS_ENABLE)
			res->flags |= IORESOURCE_ROM_ENABLE;
		l64 = l & PCI_ROM_ADDRESS_MASK;
		sz64 = sz & PCI_ROM_ADDRESS_MASK;
		mask64 = PCI_ROM_ADDRESS_MASK;
	}

	if (res->flags & IORESOURCE_MEM_64) {
		pci_read_config_dword(dev, pos + 4, &l);
		pci_write_config_dword(dev, pos + 4, ~0);
		pci_read_config_dword(dev, pos + 4, &sz);
		pci_write_config_dword(dev, pos + 4, l);

		l64 |= ((u64)l << 32);
		sz64 |= ((u64)sz << 32);
		mask64 |= ((u64)~0 << 32);
	}

	if (mmio_always_on && (orig_cmd & PCI_COMMAND_DECODE_ENABLE))
		pci_write_config_word(dev, PCI_COMMAND, orig_cmd);

	if (!sz64)
		goto fail;

	sz64 = pci_size(l64, sz64, mask64);
	if (!sz64) {
		pci_info(dev, FW_BUG "reg 0x%x: invalid BAR (can't size)\n",
			 pos);
		goto fail;
	}

	if (res->flags & IORESOURCE_MEM_64) {
		if ((sizeof(pci_bus_addr_t) < 8 || sizeof(resource_size_t) < 8)
		    && sz64 > 0x100000000ULL) {
			res->flags |= IORESOURCE_UNSET | IORESOURCE_DISABLED;
			res->start = 0;
			res->end = 0;
			pci_err(dev, "reg 0x%x: can't handle BAR larger than 4GB (size %#010llx)\n",
				pos, (unsigned long long)sz64);
			goto out;
		}

		if ((sizeof(pci_bus_addr_t) < 8) && l) {
			/* Above 32-bit boundary; try to reallocate */
			res->flags |= IORESOURCE_UNSET;
			res->start = 0;
			res->end = sz64 - 1;
			pci_info(dev, "reg 0x%x: can't handle BAR above 4GB (bus address %#010llx)\n",
				 pos, (unsigned long long)l64);
			goto out;
		}
	}

	region.start = l64;
	region.end = l64 + sz64 - 1;

	//XXX: Do not do this work because just support legacy bus
	//pcibios_bus_to_resource(dev->bus, res, &region);
	//pcibios_resource_to_bus(dev->bus, &inverted_region, res);

	/*
	 * If "A" is a BAR value (a bus address), "bus_to_resource(A)" is
	 * the corresponding resource address (the physical address used by
	 * the CPU.  Converting that resource address back to a bus address
	 * should yield the original BAR value:
	 *
	 *     resource_to_bus(bus_to_resource(A)) == A
	 *
	 * If it doesn't, CPU accesses to "bus_to_resource(A)" will not
	 * be claimed by the device.
	 */
	if (inverted_region.start != region.start) {
		res->flags |= IORESOURCE_UNSET;
		res->start = 0;
		res->end = region.end - region.start;
		pci_info(dev, "reg 0x%x: initial BAR value %#010llx invalid\n",
			 pos, (unsigned long long)region.start);
	}

	goto out;


fail:
	res->flags = 0;
out:
	if (res->flags)
		pci_info(dev, "reg 0x%x: %pR\n", pos, res);

	return (res->flags & IORESOURCE_MEM_64) ? 1 : 0;
}

void pci_read_bases(pci_info_t* info)
{
	unsigned int pos, reg;

	for (pos = 0; pos < 6; pos++) {
		struct resource *res = &info->resource[pos];
		reg = PCI_BASE_ADDRESS_0 + (pos << 2);
		pos += __pci_read_base(info, pci_bar_unknown, res, reg);
	}
}

int pci_get_device_info(uint32_t vendor_id, uint32_t device_id, uint32_t subsystem_id, pci_info_t* info, int8_t bus_master)
{
	uint32_t slot, bus, i;

	if (!info)
		return -EINVAL;

	if (!mechanism && !is_uhyve())
		pci_init();

	memset(info, 0, sizeof(pci_info_t));

	for (bus = 0; bus < MAX_BUS; bus++) {
		for (slot = 0; slot < MAX_SLOTS; slot++) {
			if (adapters[bus][slot] != -1) {
				if (((adapters[bus][slot] & 0xffff) == vendor_id) &&
				    (((adapters[bus][slot] & 0xffff0000) >> 16) == device_id)) {
					u8 hdr_type;

					if (subsystem_id != PCI_IGNORE_SUBID && pci_subid(bus, slot) != subsystem_id)
						continue;
					for(i=0; i<6; i++) {
						info->base[i] = pci_what_iobase(bus, slot, i);
						info->size[i] = (info->base[i]) ? pci_what_size(bus, slot, i) : 0;
					}
					info->irq = pci_what_irq(bus, slot);
					if (bus_master)
						pci_bus_master(bus, slot);
					info->devfn = slot << 3;
					info->bus = bus;

					pci_read_config_byte(info, PCI_HEADER_TYPE, &hdr_type);
					hdr_type = hdr_type & 0x7f;
					if (hdr_type != PCI_HEADER_TYPE_NORMAL)
						continue;

					return 0;
				}
			}
		}
	}

	return -EINVAL;
}

int print_pci_adapters(void)
{
	uint32_t slot, bus;
	uint32_t counter = 0;
#ifdef WITH_PCI_IDS
	uint32_t i;
#endif

	if (!mechanism)
		pci_init();

	for (bus = 0; bus < MAX_BUS; bus++) {
                for (slot = 0; slot < MAX_SLOTS; slot++) {

		if (adapters[bus][slot] != -1) {
				counter++;
				uint32_t csid = pci_subid(bus, slot);
				LOG_INFO("%d) Vendor ID: 0x%x  Device ID: 0x%x Subsystem Vendor ID: 0x%x Subsystem Device ID: 0x%x\n",
					counter, adapters[bus][slot] & 0xffff,
					(adapters[bus][slot] & 0xffff0000) >> 16, csid & 0xffff, csid >> 16);

#ifdef WITH_PCI_IDS
				for (i=0; i<PCI_VENTABLE_LEN; i++) {
					if ((adapters[bus][slot] & 0xffff) ==
					    (uint32_t)PciVenTable[i].VenId)
						LOG_INFO("\tVendor is %s\n",
							PciVenTable[i].VenShort);
				}

				for (i=0; i<PCI_DEVTABLE_LEN; i++) {
					if ((adapters[bus][slot] & 0xffff) ==
					    (uint32_t)PciDevTable[i].VenId) {
						if (((adapters[bus][slot] & 0xffff0000) >> 16) ==
						    PciDevTable[i].DevId) {
							LOG_INFO
							    ("\tChip: %s ChipDesc: %s\n",
							     PciDevTable[i].Chip,
							     PciDevTable[i].ChipDesc);
						}
					}
				}
#endif
			}
		}
	}

	return 0;
}
