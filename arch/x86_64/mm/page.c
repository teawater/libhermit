/*
 * Copyright (c) 2010, Stefan Lankes, RWTH Aachen University
 *               2014, Steffen Vogel, RWTH Aachen University
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
 * This is a 32/64 bit portable paging implementation for the x86 architecture
 * using self-referenced page tables	i.
 * See http://www.noteblok.net/2014/06/14/bachelor/ for a detailed description.
 *
 * @author Steffen Vogel <steffen.vogel@rwth-aachen.de>
 */

#include <hermit/stdio.h>
#include <hermit/memory.h>
#include <hermit/errno.h>
#include <hermit/string.h>
#include <hermit/spinlock.h>
#include <hermit/tasks.h>
#include <hermit/logging.h>

#include <asm/multiboot.h>
#include <asm/irq.h>
#include <asm/page.h>

/* Note that linker symbols are not variables, they have no memory
 * allocated for maintaining a value, rather their address is their value. */
extern const void kernel_start;
extern const void __bss_start;

/** Single-address space operating system => one lock for all tasks */
static spinlock_irqsave_t page_lock = SPINLOCK_IRQSAVE_INIT;

/** A self-reference enables direct access to all page tables */
static size_t* const self[PAGE_LEVELS] = {
	(size_t *) 0xFFFFFF8000000000,
	(size_t *) 0xFFFFFFFFC0000000,
	(size_t *) 0xFFFFFFFFFFE00000,
	(size_t *) 0xFFFFFFFFFFFFF000
};

static uint8_t expect_zeroed_pages = 0;

size_t virt_to_phys(size_t addr)
{
	task_t* task = per_core(current_task);

	if ((addr > (size_t) &kernel_start) &&
	    (addr <= PAGE_2M_CEIL((size_t) &kernel_start + image_size)))
	{
		size_t vpn   = addr >> (PAGE_2M_BITS);	// virtual page number
		size_t entry = self[1][vpn];		// page table entry
		size_t off   = addr  & ~PAGE_2M_MASK;	// offset within page
		size_t phy   = entry &  PAGE_2M_MASK;	// physical page frame number

		return phy | off;
	}
#ifndef KATA
	else if ((task->heap) && (addr >= task->heap->start) && (addr < task->heap->end)) {
		size_t vpn   = addr >> (HUGE_PAGE_BITS); // virtual page number
		size_t entry = (HUGE_PAGE_SIZE == PAGE_2M_SIZE) ? self[1][vpn] : self[2][vpn];	// page table entry
		size_t off   = addr  & ~HUGE_PAGE_MASK;	// offset within page
		size_t phy   = entry &  HUGE_PAGE_MASK;	// physical page frame number

		return phy | off;
	}
#endif
	else {
		size_t vpn   = addr >> PAGE_BITS;	// virtual page number
		size_t entry = self[0][vpn];		// page table entry
		size_t off   = addr  & ~PAGE_MASK;	// offset within page
		size_t phy   = entry &  PAGE_MASK;	// physical page frame number

		return phy | off;
	}
}

/*
 * get memory page size
 */
int getpagesize(void)
{
	return PAGE_SIZE;
}

//TODO: code is missing
int page_set_flags(size_t viraddr, uint32_t npages, int flags)
{
	return -EINVAL;
}

int __page_map(size_t viraddr, size_t phyaddr, size_t npages, size_t bits, uint8_t do_ipi)
{
	ssize_t vpn = viraddr >> PAGE_BITS;
	ssize_t first[PAGE_LEVELS] = {[0 ... PAGE_LEVELS-1] = 0};
	ssize_t last[PAGE_LEVELS] = {[0 ... PAGE_LEVELS-1] = 0};
	size_t page_size = PAGE_SIZE;
	size_t page_bits = PAGE_BITS;
	int32_t offset = 0;
	int ret = -ENOMEM;
	int8_t send_ipi = 0;

	//kprintf("Map %d pages at 0x%zx (0x%zx)\n", npages, viraddr, phyaddr);

	if ((HUGE_PAGE_SIZE != PAGE_SIZE) && !(viraddr & (HUGE_PAGE_SIZE-1))
	   && !(phyaddr & (HUGE_PAGE_SIZE-1))
	   && (npages == HUGE_PAGE_SIZE/PAGE_SIZE)) {
		LOG_DEBUG("Map huge page...\n");

		npages = 1;
		page_size = HUGE_PAGE_SIZE;
		page_bits = HUGE_PAGE_BITS;

		if (HUGE_PAGE_SIZE == PAGE_2M_SIZE)
			offset = 1;
		else // => 1GB pages
			offset = 2;
	}

	/* Calculate index boundaries for page map traversal */
	for (int32_t lvl=offset; lvl<PAGE_LEVELS; lvl++) {
		first[lvl] = (vpn         ) >> (lvl * PAGE_MAP_BITS);
		last[lvl]  = (vpn+npages-1) >> (lvl * PAGE_MAP_BITS);
	}

	spinlock_irqsave_lock(&page_lock);

	/* Start iterating through the entries
	 * beginning at the root table (PGD or PML4) */
	for (int32_t lvl=PAGE_LEVELS-1; lvl>=offset; lvl--) {
		for (vpn=first[lvl]; vpn<=last[lvl]; vpn++) {
			if (lvl != offset) { /* PML4, PDPT, PGD */
				if (!(self[lvl][vpn] & PG_PRESENT)) {
					/* There's no table available which covers the region.
					 * Therefore we need to create a new empty table. */
					size_t paddr = get_pages(1);
					if (BUILTIN_EXPECT(!paddr, 0))
						goto out;

					/* Reference the new table within its parent */
					self[lvl][vpn] = (paddr | bits | PG_PRESENT | PG_USER | PG_RW | PG_ACCESSED | PG_DIRTY) & ~PG_XD;

					/* Fill new table with zeros */
					memset(&self[lvl-1][vpn<<PAGE_MAP_BITS], 0, PAGE_SIZE);
				}
			} else { /* last level page table */
				int8_t flush = 0;

				/* do we have to flush the TLB? */
				if (self[lvl][vpn] & PG_PRESENT) {
					//kprintf("Remap address 0x%zx at core %d\n", viraddr, CORE_ID);
					send_ipi = flush = 1;
				}

				self[lvl][vpn] = phyaddr | bits | PG_PRESENT | PG_ACCESSED | PG_DIRTY;

				// Do we map a huge page?
				if (offset)
					self[lvl][vpn] = self[lvl][vpn] | PG_PSE;

				if (flush)
					tlb_flush_one_page(vpn << page_bits, 0);

				phyaddr += page_size;
			}
		}
	}

	if (do_ipi && send_ipi)
		ipi_tlb_flush();

	ret = 0;
out:
	spinlock_irqsave_unlock(&page_lock);

	return ret;
}

int page_unmap(size_t viraddr, size_t npages)
{
	if (BUILTIN_EXPECT(!npages, 0))
		return 0;

	//kprintf("Unmap %d pages at 0x%zx\n", npages, viraddr);

	spinlock_irqsave_lock(&page_lock);

	/* Start iterating through the entries.
	 * Only the PGT entries are removed. Tables remain allocated. */
	size_t start = viraddr>>PAGE_BITS;
	for (size_t vpn=viraddr>>PAGE_BITS; vpn<start+npages; vpn++) {
		self[0][vpn] = 0;
		tlb_flush_one_page(vpn << PAGE_BITS, 0);
	}

	ipi_tlb_flush();

	spinlock_irqsave_unlock(&page_lock);

	/* This can't fail because we don't make checks here */
	return 0;
}

void page_fault_handler(struct state *s)
{
	size_t viraddr = read_cr2();
	task_t* task = per_core(current_task);

	int check_pagetables(size_t vaddr)
	{
		int lvl;
		int start = (HUGE_PAGE_SIZE == PAGE_2M_SIZE) ? 1 : 2;
		ssize_t vpn = vaddr >> PAGE_BITS;
		ssize_t index[PAGE_LEVELS];

		/* Calculate index boundaries for page map traversal */
		for (lvl=start; lvl<PAGE_LEVELS; lvl++)
			index[lvl] = vpn >> (lvl * PAGE_MAP_BITS);

		/* do we have already a valid entry in the page tables */
		for (lvl=PAGE_LEVELS-1; lvl>=start; lvl--) {
			vpn = index[lvl];

			if (!(self[lvl][vpn] & PG_PRESENT))
				return 0;
		}

		return 1;
	}

#ifndef KATA
	spinlock_irqsave_lock(&page_lock);

	if (((task->heap) && (viraddr >= task->heap->start) && (viraddr < task->heap->end))
	   || ((viraddr >= (size_t) &__bss_start) && (viraddr < (size_t) &kernel_start + image_size))) {
		size_t flags;
		int ret;

		/*
		 * do we have a valid page table entry? => flush TLB and return
		 */
		if (check_pagetables(viraddr)) {
			//tlb_flush_one_page(viraddr, 0);
			spinlock_irqsave_unlock(&page_lock);
			return;
		}

		 // on demand userspace heap mapping
		viraddr &= HUGE_PAGE_MASK;

		size_t phyaddr = expect_zeroed_pages ? get_zeroed_huge_page() : get_huge_page();
		if (BUILTIN_EXPECT(!phyaddr, 0)) {
			LOG_ERROR("out of memory: task = %u\n", task->id);
			goto default_handler;
		}

		flags = PG_USER|PG_RW;
		if (has_nx()) // set no execution flag to protect the heap
			flags |= PG_XD;
		ret = __page_map(viraddr, phyaddr, HUGE_PAGE_SIZE/PAGE_SIZE, flags, 0);

		if (BUILTIN_EXPECT(ret, 0)) {
			LOG_ERROR("map_region: could not map %#lx to %#lx, task = %u\n", phyaddr, viraddr, task->id);
			put_page(phyaddr);

			goto default_handler;
		}

		spinlock_irqsave_unlock(&page_lock);

		// clear cr2 to signalize that the pagefault is solved by the pagefault handler
		write_cr2(0);

		return;
	}

default_handler:
	spinlock_irqsave_unlock(&page_lock);
#endif

	LOG_ERROR("Page Fault Exception (%d) on core %d at cs:ip = %#x:%#lx, fs = %#lx, gs = %#lx, rflags 0x%lx, task = %u, addr = %#lx, error = %#x [ %s %s %s %s %s ]\n",
		s->int_no, CORE_ID, s->cs, s->rip, s->fs, s->gs, s->rflags, task->id, viraddr, s->error,
		(s->error & 0x4) ? "user" : "supervisor",
		(s->error & 0x10) ? "instruction" : "data",
		(s->error & 0x2) ? "write" : ((s->error & 0x10) ? "fetch" : "read"),
		(s->error & 0x1) ? "protection" : "not present",
		(s->error & 0x8) ? "reserved bit" : "\b");
	LOG_ERROR("rax %#lx, rbx %#lx, rcx %#lx, rdx %#lx, rbp, %#lx, rsp %#lx rdi %#lx, rsi %#lx, r8 %#lx, r9 %#lx, r10 %#lx, r11 %#lx, r12 %#lx, r13 %#lx, r14 %#lx, r15 %#lx\n",
		s->rax, s->rbx, s->rcx, s->rdx, s->rbp, s->rsp, s->rdi, s->rsi, s->r8, s->r9, s->r10, s->r11, s->r12, s->r13, s->r14, s->r15);
#ifndef KATA
	if (task->heap)
		LOG_ERROR("Heap 0x%zx - 0x%zx\n", task->heap->start, task->heap->end);
#endif

	// clear cr2 to signalize that the pagefault is solved by the pagefault handler
	write_cr2(0);

	//do_abort();
	sys_exit(-EFAULT);
}

// weak symbol is used to detect a Go application
void __attribute__((weak)) runtime_osinit();

int page_init(void)
{
	// do we have Go application? => weak symbol isn't zeroe
	// => Go expect zeroed pages => set zeroed_pages to true
	if (runtime_osinit) {
		expect_zeroed_pages = 1;
		LOG_INFO("Detect Go runtime! Consequently, HermitCore zeroed heap.\n");
	}

	if (mb_info && (mb_info->flags & MULTIBOOT_INFO_CMDLINE) && (cmdline))
	{
		size_t i = 0;

		while(((size_t) cmdline + i) <= ((size_t) cmdline + cmdsize))
		{
			page_map(((size_t) cmdline + i) & PAGE_MASK, ((size_t) cmdline + i) & PAGE_MASK,
				1, PG_NX|PG_GLOBAL|PG_RW|PG_PRESENT);
			i += PAGE_SIZE;
		}
	} else cmdline = 0;

	/* Replace default pagefault handler */
	irq_uninstall_handler(14);
	irq_install_handler(14, page_fault_handler);

	return 0;
}
