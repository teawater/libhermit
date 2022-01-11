/*
 * Copyright (c) 2021 Ant Group
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

#ifndef _VIRTIO_H
#define _VIRTIO_H

#include <hermit/stddef.h>

#include <hermit/stddef.h>
#include <hermit/errno.h>
#include <hermit/stdio.h>
#include <hermit/string.h>
#include <hermit/processor.h>
#include <hermit/mailbox.h>
#include <hermit/logging.h>
#include <hermit/virtio_net.h>
#include <hermit/virtio_ring.h>
#include <hermit/virtio_pci.h>
#include <hermit/virtio_net.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pci.h>

/* This feature indicates support for the packed virtqueue layout. */
#define VIRTIO_F_RING_PACKED		34

/*
 * This feature indicates that memory accesses by the driver and the
 * device are ordered in a way described by the platform.
 */
#define VIRTIO_F_ORDER_PLATFORM		36

enum dma_data_direction {
	DMA_BIDIRECTIONAL = 0,
	DMA_TO_DEVICE = 1,
	DMA_FROM_DEVICE = 2,
	DMA_NONE = 3,
};

struct virtqueue {
	void (*callback)(struct virtqueue *vq);
	const char *name;
	struct virtio_device *vdev;
        unsigned int index;
        unsigned int num_free;
	void *priv;
};

struct virtio_device {
	uint64_t features;

	union {
		struct {
			// For pci legacy
			uint32_t iobase;
		};
		struct {
			// For pci modern
			struct virtio_pci_common_cfg *cfg;
			void *device;
			u8 *isr;
			size_t device_len;
			void *notify_base;
			int notify_map_cap;
			size_t notify_len;
			u32 notify_offset_multiplier;
		};
	};

	void (*set_status)(struct virtio_device *vdev, uint8_t status);
	u8 (*get_status)(struct virtio_device *vdev);
	u64 (*get_features)(struct virtio_device *vdev);
	void (*set_features)(struct virtio_device *vdev);
	void (*get)(struct virtio_device *vdev, unsigned offset, u8 *buf,
		    unsigned len);
	u8 (*get_isr)(struct virtio_device *vdev);
	struct virtqueue *(*setup_vq)(pci_info_t* pci_info,
					struct virtio_device *vdev,
					int index,
					void (*callback)(struct virtqueue *vq),
					const char *name,
					bool ctx);
};

struct scatterlist {
	uint64_t 	phy;
	unsigned int	length;
	bool		is_last;
};

extern int virtio_device_setup(struct virtio_device *vdev, pci_info_t* info, bool is_legacy);
extern struct virtqueue *virtio_setup_vq(pci_info_t* pci_info,
					 struct virtio_device *vdev,
					 int index,
					 void (*callback)(struct virtqueue *vq),
					 const char *name,
					 bool ctx);

static inline __virtio16 cpu_to_virtio16(struct virtio_device *vdev, u16 val)
{
	return val;
}

static inline u16 virtio16_to_cpu(struct virtio_device *vdev, __virtio16 val)
{
	return val;
}

static inline u32 virtio32_to_cpu(struct virtio_device *vdev, __virtio32 val)
{
	return val;
}

static inline __virtio32 cpu_to_virtio32(struct virtio_device *vdev, u32 val)
{
	return val;
}

static inline u64 virtio64_to_cpu(struct virtio_device *vdev, __virtio64 val)
{
	return val;
}

static inline __virtio64 cpu_to_virtio64(struct virtio_device *vdev, u64 val)
{
	return val;
}

static inline bool virtio_has_feature(const struct virtio_device *vdev,
				      unsigned int fbit)
{
	return !!(vdev->features & (1ul << fbit));
}

static inline void sg_set_buf(struct scatterlist *sg, const void *buf,
			      unsigned int buflen)
{
	sg->phy = virt_to_phys((size_t)buf);
	sg->length = buflen;
	sg->is_last = false;
}

static inline void sg_init_one(struct scatterlist *sg,
			       const void *buf, unsigned int buflen)
{
	sg_set_buf(sg, buf, buflen);
	sg[0].is_last = true;
}

/*
 * Constants for operation sizes. On 32-bit, the 64-bit size it set to
 * -1 because sizeof will never return -1, thereby making those switch
 * case statements guaranteed dead code which the compiler will
 * eliminate, and allowing the "missing symbol in the default case" to
 * indicate a usage error.
 */
#define __X86_CASE_B	1
#define __X86_CASE_W	2
#define __X86_CASE_L	4
//#ifdef CONFIG_64BIT
#define __X86_CASE_Q	8
//#else
//#define	__X86_CASE_Q	-1		/* sizeof will never return -1 */
//#endif

#define __xchg_wrong_size() do {} while(1)

/* 
 * An exchange-type operation, which takes a value and a pointer, and
 * returns the old value.
 */
#define __xchg_op(ptr, arg, op, lock)					\
	({								\
	        __typeof__ (*(ptr)) __ret = (arg);			\
		switch (sizeof(*(ptr))) {				\
		case __X86_CASE_B:					\
			asm volatile (lock #op "b %b0, %1\n"		\
				      : "+q" (__ret), "+m" (*(ptr))	\
				      : : "memory", "cc");		\
			break;						\
		case __X86_CASE_W:					\
			asm volatile (lock #op "w %w0, %1\n"		\
				      : "+r" (__ret), "+m" (*(ptr))	\
				      : : "memory", "cc");		\
			break;						\
		case __X86_CASE_L:					\
			asm volatile (lock #op "l %0, %1\n"		\
				      : "+r" (__ret), "+m" (*(ptr))	\
				      : : "memory", "cc");		\
			break;						\
		case __X86_CASE_Q:					\
			asm volatile (lock #op "q %q0, %1\n"		\
				      : "+r" (__ret), "+m" (*(ptr))	\
				      : : "memory", "cc");		\
			break;						\
		default:						\
			__ ## op ## _wrong_size();			\
		}							\
		__ret;							\
	})

/*
 * Note: no "lock" prefix even on SMP: xchg always implies lock anyway.
 * Since this is generally used to protect other memory information, we
 * use "asm volatile" and "memory" clobbers to prevent gcc from moving
 * information around.
 */
#define xchg(ptr, v)	__xchg_op((ptr), (v), xchg, "")

#define barrier() asm volatile("" ::: "memory")
#define virt_mb() __sync_synchronize()
#define virt_rmb() barrier()
#define virt_wmb() barrier()
#define __smp_store_mb(var, value) do { (void)xchg(&var, value); } while (0)
#define virt_store_mb(var, value) __smp_store_mb(var, value)


/* Compile time object size, -1 for unknown */
#ifndef __compiletime_object_size
# define __compiletime_object_size(obj) -1
#endif

/* Is this type a native word size -- useful for atomic operations */
#define __native_word(t) \
	(sizeof(t) == sizeof(char) || sizeof(t) == sizeof(short) || \
	 sizeof(t) == sizeof(int) || sizeof(t) == sizeof(long))

#ifdef __OPTIMIZE__
#define __compiletime_error(message)    __attribute__((error(message)))

# define __compiletime_assert(condition, msg, prefix, suffix)		\
	do {								\
		extern void prefix ## suffix(void) __compiletime_error(msg); \
		if (!(condition))					\
			prefix ## suffix();				\
	} while (0)
#else
# define __compiletime_assert(condition, msg, prefix, suffix) do { } while (0)
#endif

#define _compiletime_assert(condition, msg, prefix, suffix) \
	__compiletime_assert(condition, msg, prefix, suffix)
#define compiletime_assert(condition, msg) \
	_compiletime_assert(condition, msg, __compiletime_assert_, __COUNTER__)
#define compiletime_assert_rwonce_type(t)					\
	compiletime_assert(__native_word(t) || sizeof(t) == sizeof(long long),	\
		"Unsupported access size for {READ,WRITE}_ONCE().")

#define __scalar_type_to_expr_cases(type)				\
		unsigned type:	(unsigned type)0,			\
		signed type:	(signed type)0

#define __unqual_scalar_typeof(x) typeof(				\
		_Generic((x),						\
			 char:	(char)0,				\
			 __scalar_type_to_expr_cases(char),		\
			 __scalar_type_to_expr_cases(short),		\
			 __scalar_type_to_expr_cases(int),		\
			 __scalar_type_to_expr_cases(long),		\
			 __scalar_type_to_expr_cases(long long),	\
			 default: (x)))

#ifndef __READ_ONCE
#define __READ_ONCE(x)	(*(const volatile __unqual_scalar_typeof(x) *)&(x))
#endif

#define READ_ONCE(x)							\
({									\
	compiletime_assert_rwonce_type(x);				\
	__READ_ONCE(x);							\
})

#define __WRITE_ONCE(x, val)						\
do {									\
	*(volatile typeof(x) *)&(x) = (val);				\
} while (0)

#define WRITE_ONCE(x, val)						\
do {									\
	compiletime_assert_rwonce_type(x);				\
	__WRITE_ONCE(x, val);						\
} while (0)

static inline void virtio_mb(bool weak_barriers)
{
	if (weak_barriers)
		virt_mb();
	else
		mb();
}

static inline void virtio_rmb(bool weak_barriers)
{
	if (weak_barriers)
		virt_rmb();
	//else
	//	dma_rmb();
}

static inline void virtio_wmb(bool weak_barriers)
{
	if (weak_barriers)
		virt_wmb();
	//else
	//	dma_wmb();
}

#define virtio_store_mb(weak_barriers, p, v) \
do { \
	if (weak_barriers) { \
		virt_store_mb(*p, v); \
	} else { \
		WRITE_ONCE(*p, v); \
		mb(); \
	} \
} while (0) \

//#define pr_debug do {} while (0)

extern struct virtqueue *vring_create_virtqueue_split(unsigned int index,
						      unsigned int num,
						      unsigned int vring_align,
						      struct virtio_device *vdev,
						      bool weak_barriers,
						      bool may_reduce_num,
						      bool context,
						      bool (*notify)(struct virtqueue *),
						      void (*callback)(struct virtqueue *),
						      const char *name);
extern dma_addr_t virtqueue_get_desc_addr(struct virtqueue *vq);
extern irqreturn_t vring_interrupt(void *vq);
extern int virtqueue_add_split(struct virtqueue *_vq,
			struct scatterlist *sgs[],
			unsigned int total_sg,
			unsigned int out_sgs,
			unsigned int in_sgs,
			void *data,
			void *ctx);
extern int virtqueue_add_outbuf(struct virtqueue *vq,
				struct scatterlist *sg, unsigned int num,
				void *data);
extern bool virtqueue_kick(struct virtqueue *vq);
extern void *virtqueue_get_buf(struct virtqueue *vq, unsigned int *len);
extern bool virtqueue_is_broken(struct virtqueue *vq);
unsigned int virtqueue_get_vring_size(struct virtqueue *_vq);
dma_addr_t virtqueue_get_avail_addr(struct virtqueue *_vq);
dma_addr_t virtqueue_get_used_addr(struct virtqueue *_vq);

extern int virtio_device_find(pci_info_t *pci_info, bool *is_legacy, uint32_t vendor_id,
			      uint32_t device_id);

extern void virtio_device_ready(struct virtio_device *vdev);

extern int virtio_pci_modern_init(struct virtio_device *vdev, pci_info_t* pci_info);
extern void virtio_pci_legacy_init(struct virtio_device *vdev, pci_info_t* pci_info);

#endif
