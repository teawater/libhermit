/*
 * Copyright (c) 2017, Annika Wierichs, RWTH Aachen University
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
 *
 * TODO: Documentation
 *
 */


#include "uhyve-ibv-guest-host.h"


struct ibv_context * guest_to_host_ibv_context(struct ibv_context * context) {
	ibv_context_virt_ptrs.device = context->device,
	ibv_context_virt_ptrs.abi_compat = context->abi_compat,

	context->device = guest_to_host_ibv_device(context->device);
	context->abi_compat = guest_to_host_ibv_abi_compat_v2(context->abi_compat);
	guest_to_host_ibv_context_ops(&context->ops);
	/*guest_to_host_pthread_mutex_t(&context->mutex); // TODO*/

	return (struct ibv_context *) guest_to_host((size_t) context);
}

void phys_to_virt_ibv_context(struct ibv_context * context) {
	context->device = ibv_context_virt_ptrs.device;
	context->abi_compat = ibv_context_virt_ptrs.abi_compat;

	phys_to_virt_ibv_device(context->device);
	phys_to_virt_ibv_abi_compat_v2(context->abi_compat);
	phys_to_virt_ibv_context_ops(&context->ops);
}