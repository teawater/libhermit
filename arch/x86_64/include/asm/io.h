/*
 * Copyright (c) 2010, Stefan Lankes, RWTH Aachen University
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

/**
 * @author Stefan Lankes
 * @file arch/x86/include/asm/io.h
 * @brief Functions related to processor IO
 *
 * This file contains inline functions for processor IO operations.
 */

#ifndef __ARCH_IO_H__
#define __ARCH_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CMOS_PORT_ADDRESS
#define CMOS_PORT_ADDRESS	0x70
#endif
#ifndef CMOS_PORT_DATA
#define CMOS_PORT_DATA		0x71
#endif

/** @brief Read a byte from an IO port
 *
 * @param _port The port you want to read from
 * @return The value which reads out from this port
 */
inline static unsigned char inportb(unsigned short _port) {
	unsigned char rv;
	asm volatile("inb %1, %0":"=a"(rv):"dN"(_port));
	return rv;
} 

/** @brief Read a word (2 byte) from an IO port
 *
 * @param _port The port you want to read from
 * @return The value which reads out from this port
 */
inline static unsigned short inportw(unsigned short _port) {
	unsigned short rv;
	asm volatile("inw %1, %0":"=a"(rv):"dN"(_port));
	return rv;
}

/** @brief Read a double word (4 byte) from an IO port
 *
 * @param _port The port you want to read from
 * @return The value which reads out from this port
 */
inline static unsigned int inportl(unsigned short _port) {
	unsigned int rv;
	asm volatile("inl %1, %0":"=a"(rv):"dN"(_port));
	return rv;
}

/** @brief Write a byte to an IO port
 *
 * @param _port The port you want to write to
 * @param _data the 1 byte value you want to write
 */
inline static void outportb(unsigned short _port, unsigned char _data) {
	asm volatile("outb %1, %0"::"dN"(_port), "a"(_data));
}

/** @brief Write a word (2 bytes) to an IO port
 *
 * @param _port The port you want to write to
 * @param _data the 2 byte value you want to write
 */
inline static void outportw(unsigned short _port, unsigned short _data) {
	asm volatile("outw %1, %0"::"dN"(_port), "a"(_data));
}

/** @brief Write a double word (4 bytes) to an IO port
 *
 * @param _port The port you want to write to
 * @param _data the 4 byte value you want to write
 */
inline static void outportl(unsigned short _port, unsigned int _data)
{
	 asm volatile("outl %1, %0"::"dN"(_port), "a"(_data));
}

/**
 * write a byte in CMOS
 *  @param offset CMOS offset
 *  @param val the value you want wto write
 */
inline static void cmos_write(uint8_t offset, uint8_t val)
{
	outportb(CMOS_PORT_ADDRESS, offset);
	outportb(CMOS_PORT_DATA, val);
}

#define iowrite16(d, p)	outportw(p, d)
#define iowrite32(d, p)	outportl(p, d)
#define ioread32(p) inportl(p)

/*
 * Functions for accessing PCI base (first 256 bytes) and extended
 * (4096 bytes per PCI function) configuration space with type 1
 * accesses.
 */

#define PCI_CONF1_ADDRESS(bus, devfn, reg) \
	(0x80000000 | ((reg & 0xF00) << 16) | (bus << 16) \
	| (devfn << 8) | (reg & 0xFC))

inline static void
pci_bus_read_config_byte(unsigned int bus, unsigned int devfn, int reg, u8 *value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	*value = inportb(0xCFC + (reg & 3));
}

inline static void
pci_bus_read_config_word(unsigned int bus, unsigned int devfn, int reg, u16 *value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	*value = inportw(0xCFC + (reg & 2));
}

inline static void
pci_bus_read_config_dword(unsigned int bus, unsigned int devfn, int reg, u32 *value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	*value = inportl(0xCFC);
}

inline static void
pci_bus_write_config_byte(unsigned int bus, unsigned int devfn, int reg, u8 value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	outportb(0xCFC + (reg & 3), value);
}

inline static void
pci_bus_write_config_word(unsigned int bus, unsigned int devfn, int reg, u16 value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	outportw(0xCFC + (reg & 2), value);
}

inline static void
pci_bus_write_config_dword(unsigned int bus, unsigned int devfn, int reg, u32 value)
{
	outportl(PCI_CONF1_ADDRESS(bus, devfn, reg), 0xCF8);
	outportl(0xCFC, value);
}

#define pci_read_config_byte(dev, reg, value)	pci_bus_read_config_byte(dev->bus, dev->slot, reg, value)
#define pci_read_config_word(dev, reg, value)	pci_bus_read_config_word(dev->bus, dev->slot, reg, value)
#define pci_read_config_dword(dev, reg, value)	pci_bus_read_config_dword(dev->bus, dev->slot, reg, value)
#define pci_write_config_byte(dev, reg, value)	pci_bus_write_config_byte(dev->bus, dev->slot, reg, value)
#define pci_write_config_word(dev, reg, value)	pci_bus_write_config_word(dev->bus, dev->slot, reg, value)
#define pci_write_config_dword(dev, reg, value)	pci_bus_write_config_dword(dev->bus, dev->slot, reg, value)

#ifdef __cplusplus
}
#endif

#endif
