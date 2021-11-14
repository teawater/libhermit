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

#include <hermit/stddef.h>
#include <hermit/stdio.h>
#include <hermit/string.h>
#include <hermit/time.h>
#include <hermit/tasks.h>
#include <hermit/processor.h>
#include <hermit/tasks.h>
#include <hermit/syscall.h>
#include <hermit/memory.h>
#include <hermit/logging.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/uart.h>
#ifdef __x86_64__
#include <asm/multiboot.h>
#endif
#include <asm/uhyve.h>

#include <lwip/init.h>
#include <lwip/sys.h>
#include <lwip/stats.h>
#include <lwip/ip_addr.h>
#include <lwip/udp.h>
#include <lwip/tcp.h>
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>
#include <lwip/netifapi.h>
#include <lwip/ip_addr.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/stats.h>
#include <netif/etharp.h>
#include <net/mmnif.h>
#include <net/rtl8139.h>
#include <net/e1000.h>
#include <net/vioif.h>
#include <net/uhyve-net.h>

#define HERMIT_PORT	0x494E
#define HERMIT_MAGIC	0x7E317

typedef struct {
	int argc;
	int argsz[MAX_ARGC_ENVC];
	int envc;
	int envsz[MAX_ARGC_ENVC];
} __attribute__ ((packed)) uhyve_cmdsize_t;

typedef struct {
	char **argv;
	char **envp;
} __attribute__ ((packed)) uhyve_cmdval_t;

static struct netif	default_netif;
static const int sobufsize = 131072;

/*
 * Note that linker symbols are not variables, they have no memory allocated for
 * maintaining a value, rather their address is their value.
 */
extern const void kernel_start;
extern const void tdata_end;
extern const void tls_start;
extern const void tls_end;
extern const void __bss_start;
extern const void percore_start;
extern const void percore_end0;
extern const void percore_end;

/* Page frame counters */
extern atomic_int64_t total_pages;
extern atomic_int64_t total_allocated_pages;
extern atomic_int64_t total_available_pages;

extern atomic_int32_t cpu_online;
extern atomic_int32_t possible_cpus;
extern int32_t isle;
extern int32_t possible_isles;
extern uint32_t boot_processor;
extern volatile int libc_sd;
extern uint8_t hcip[4];
extern uint8_t hcgateway[4];
extern uint8_t hcmask[4];

extern void signal_init();

static int hermit_init(void)
{
	clock_init();

	size_t sz = (size_t) &percore_end0 - (size_t) &percore_start;

	// initialize .kbss sections
	memset((void*)&tdata_end, 0x00, (size_t) &__bss_start - (size_t) &tdata_end);

	// initialize .percore section => copy first section to all other sections
	for(uint32_t i=1; i<MAX_CORES; i++)
		memcpy((char*) &percore_start + i*sz, (char*) &percore_start, sz);

	koutput_init();

	system_init();
	irq_init();
	timer_init();
	multitasking_init();
	memory_init();
	signal_init();

	return 0;
}

static void tcpip_init_done(void* arg)
{
	sys_sem_t* sem = (sys_sem_t*)arg;

	LOG_INFO("LwIP's tcpip thread has task id %d\n", per_core(current_task)->id);

	sys_sem_signal(sem);
}

static int init_netifs(void)
{
	ip_addr_t	ipaddr;
	ip_addr_t	netmask;
	ip_addr_t	gw;
	sys_sem_t	sem;
	err_t		err;

	if(sys_sem_new(&sem, 0) != ERR_OK)
		LWIP_ASSERT("Failed to create semaphore", 0);

	tcpip_init(tcpip_init_done, &sem);
	sys_sem_wait(&sem);
	LOG_INFO("TCP/IP initialized.\n");
	sys_sem_free(&sem);

	if (is_uhyve()) {
		LOG_INFO("HermitCore is running on uhyve!\n");
		if (uhyve_net_stat()) {
			/* Set network address variables */
			IP_ADDR4(&gw, hcgateway[0], hcgateway[1], hcgateway[2], hcgateway[3]);
			IP_ADDR4(&ipaddr, hcip[0], hcip[1], hcip[2], hcip[3]);
			IP_ADDR4(&netmask, hcmask[0], hcmask[1], hcmask[2], hcmask[3]);

			if ((err = netifapi_netif_add(&default_netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, uhyve_netif_init, ethernet_input)) != ERR_OK) {
				LOG_ERROR("Unable to add the uhyve_net network interface: err = %d\n", err);
				return -ENODEV;
			}
			/*tell lqip all initialization is done and we want to set it up */
			netifapi_netif_set_default(&default_netif);
			LOG_INFO("set_default\n");
			netifapi_netif_set_up(&default_netif);
			LOG_INFO("set_up\n");
		} else {
			return -ENODEV;
		}
	} else if (!is_single_kernel()) {
		LOG_INFO("HermitCore is running side-by-side to Linux!\n");

		/* Set network address variables */
		IP_ADDR4(&gw, 192,168,28,1);
		IP_ADDR4(&ipaddr, 192,168,28,isle+2);
		IP_ADDR4(&netmask, 255,255,255,0);

		/* register our Memory Mapped Virtual IP interface in the lwip stack
		 * and tell him how to use the interface:
		 *  - mmnif_dev : the device data storage
		 *  - ipaddr : the ip address wich should be used
		 *  - gw : the gateway wicht should be used
		 *  - mmnif_init : the initialization which has to be done in order to use our interface
		 *  - ip_input : tells him that he should use ip_input
		 *
		 * Note: Our drivers guarantee that the input function will be called in the context of the tcpip thread.
		 * => Therefore, we are able to use ip_input instead of tcpip_input
		 */
		if ((err = netifapi_netif_add(&default_netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, mmnif_init, ip_input)) != ERR_OK)
		{
			LOG_ERROR("Unable to add the intra network interface: err = %d\n", err);
			return -ENODEV;
		}

		/* tell lwip all initialization is done and we want to set it up */
		netifapi_netif_set_default(&default_netif);
		netifapi_netif_set_up(&default_netif);
	} else {
#ifdef __aarch64__
		LOG_ERROR("Unable to add the network interface\n");

		return -ENODEV;
#else
		/* Clear network address because we use DHCP to get an ip address */
		IP_ADDR4(&gw, 0,0,0,0);
		IP_ADDR4(&ipaddr, 0,0,0,0);
		IP_ADDR4(&netmask, 0,0,0,0);

		/* Note: Our drivers guarantee that the input function will be called in the context of the tcpip thread.
		 * => Therefore, we are able to use ethernet_input instead of tcpip_input */
		if ((err = netifapi_netif_add(&default_netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, vioif_init, ethernet_input)) == ERR_OK)
			goto success;
		if ((err = netifapi_netif_add(&default_netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, rtl8139if_init, ethernet_input)) == ERR_OK)
			goto success;
#ifdef USE_E1000
		if ((err = netifapi_netif_add(&default_netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, e1000if_init, ethernet_input)) == ERR_OK)
			goto success;
#endif

		LOG_ERROR("Unable to add the network interface: err = %d\n", err);

		return -ENODEV;

success:
		netifapi_netif_set_default(&default_netif);
		netifapi_netif_set_up(&default_netif);

		LOG_INFO("Starting DHCPD...\n");
		netifapi_dhcp_start(&default_netif);

		int mscnt = 0;
		int ip_counter = 0;
		/* wait for ip address */
		while(!ip_2_ip4(&default_netif.ip_addr)->addr && (ip_counter < 20)) {
			uint64_t end_tsc, start_tsc = get_rdtsc();

			do {
				if (ip_2_ip4(&default_netif.ip_addr)->addr)
					return 0;
				check_workqueues();
				end_tsc = get_rdtsc();
			} while(((end_tsc - start_tsc) / (get_cpu_frequency() * 1000)) < DHCP_FINE_TIMER_MSECS);

			dhcp_fine_tmr();
			mscnt += DHCP_FINE_TIMER_MSECS;
			if (mscnt >= DHCP_COARSE_TIMER_SECS*1000) {
				dhcp_coarse_tmr();
				mscnt = 0;
			}

			ip_counter++;
		}

		if (!ip_2_ip4(&default_netif.ip_addr)->addr)
			return -ENODEV;
#endif
	}

	return 0;
}

int network_shutdown(void)
{
	LOG_INFO("Shutdown LwIP\n");

	if (libc_sd >= 0) {
		int s = libc_sd;
		libc_sd = -1;
		lwip_close(s);
	}

	//mmnif_shutdown();
	//stats_display();

	return 0;
}

#if MAX_CORES > 1
int smp_main(void)
{
	timer_init();
#ifdef DYNAMIC_TICKS
	enable_dynticks();
#endif

	print_cpu_status(isle);

	/* wait for the other cpus */
	while(atomic_int32_read(&cpu_online) < atomic_int32_read(&possible_cpus)) {
		PAUSE;
	}

	while(1) {
		check_workqueues();
		wait_for_task();
	}

	return 0;
}
#endif

int libc_start(int argc, char** argv, char** env);

char* itoa(uint64_t input, char* str);

// init task => creates all other tasks and initializes the LwIP
static int initd(void* arg)
{
	int s = -1, c = -1;
	int i, j, flag;
	int len, err;
	int magic = 0;
	struct sockaddr_in6 server, client;
	task_t* curr_task = per_core(current_task);
	int argc, envc;
	char** argv = NULL;
	char **environ = NULL;

	LOG_INFO("Initd is running\n");

	// initialized bss section
	memset((void*)&__bss_start, 0x00, (size_t) &kernel_start + image_size - (size_t) &__bss_start);

	// setup heap
	if (!curr_task->heap)
		curr_task->heap = (vma_t*) kmalloc(sizeof(vma_t));

	if (BUILTIN_EXPECT(!curr_task->heap, 0)) {
		LOG_ERROR("load_task: heap is missing!\n");
		return -ENOMEM;
	}

	curr_task->heap->flags = VMA_HEAP|VMA_USER;
	curr_task->heap->start = HEAP_START;
	curr_task->heap->end = HEAP_START;

	// region is already reserved for the heap, we have to change the
	// property of the first page
	vma_free(curr_task->heap->start, curr_task->heap->start+PAGE_SIZE);
	vma_add(curr_task->heap->start, curr_task->heap->start+PAGE_SIZE, VMA_HEAP|VMA_USER);

#ifdef KATA
#else
#ifndef __aarch64__
	// initialize network
	err = init_netifs();
#else
	err = -EINVAL;
#endif

	if (is_uhyve()) {
		int i;
		uhyve_cmdsize_t uhyve_cmdsize;
		uhyve_cmdval_t uhyve_cmdval;
		uhyve_cmdval_t uhyve_cmdval_phys;

		uhyve_send(UHYVE_PORT_CMDSIZE,
				(unsigned)virt_to_phys((size_t)&uhyve_cmdsize));

		uhyve_cmdval.argv = kmalloc(uhyve_cmdsize.argc * sizeof(char *));
		for(i=0; i<uhyve_cmdsize.argc; i++)
			uhyve_cmdval.argv[i] = kmalloc(uhyve_cmdsize.argsz[i] * sizeof(char));
		uhyve_cmdval.envp = kmalloc(uhyve_cmdsize.envc * sizeof(char *));
		for(i=0; i<uhyve_cmdsize.envc; i++)
			uhyve_cmdval.envp[i] = kmalloc(uhyve_cmdsize.envsz[i] * sizeof(char));

		// create a similar structure with guest physical addresses
		char** argv_virt = uhyve_cmdval_phys.argv = kmalloc(uhyve_cmdsize.argc * sizeof(char *));
		for(i=0; i<uhyve_cmdsize.argc; i++)
			uhyve_cmdval_phys.argv[i] = (char*) virt_to_phys((size_t) uhyve_cmdval.argv[i]);
		uhyve_cmdval_phys.argv = (char**) virt_to_phys((size_t) uhyve_cmdval_phys.argv);

		char** envp_virt = uhyve_cmdval_phys.envp = kmalloc(uhyve_cmdsize.envc * sizeof(char *));
		for(i=0; i<uhyve_cmdsize.envc-1; i++)
			uhyve_cmdval_phys.envp[i] = (char*) virt_to_phys((size_t) uhyve_cmdval.envp[i]);
		// the last element is always NULL
		uhyve_cmdval.envp[uhyve_cmdsize.envc-1] = NULL;
		uhyve_cmdval_phys.envp = (char**) virt_to_phys((size_t) uhyve_cmdval_phys.envp);

		uhyve_send(UHYVE_PORT_CMDVAL,
				(unsigned)virt_to_phys((size_t)&uhyve_cmdval_phys));

		LOG_INFO("Boot time: %d ms\n", get_uptime());
		libc_start(uhyve_cmdsize.argc, uhyve_cmdval.argv, uhyve_cmdval.envp);

		for(i=0; i<uhyve_cmdsize.argc; i++)
			kfree(uhyve_cmdval.argv[i]);
		kfree(uhyve_cmdval.argv);
		for(i=0; i<uhyve_cmdsize.envc; i++)
			kfree(uhyve_cmdval.envp[i]);
		kfree(uhyve_cmdval.envp);
		kfree(argv_virt);
		kfree(envp_virt);

		return 0;
	}

	if ((err != 0) || !is_proxy())
	{
		char* dummy[] = {"app_name", NULL};

		LOG_INFO("Boot time: %d ms\n", (get_clock_tick() * 1000) / TIMER_FREQ);
		// call user code
		libc_start(1, dummy, NULL); //argc, argv, environ);

		return 0;
	}

	// initialize iRCCE
	if (!is_single_kernel())
		init_rcce();

	s = lwip_socket(AF_INET6, SOCK_STREAM , 0);
	if (s < 0) {
		LOG_ERROR("socket failed: %d\n", server);
		return -1;
	}

	// prepare the sockaddr_in structure
	memset((char *) &server, 0x00, sizeof(server));
	server.sin6_family = AF_INET6;
	server.sin6_addr = in6addr_any;
	server.sin6_port = htons(HERMIT_PORT);

	if ((err = lwip_bind(s, (struct sockaddr *) &server, sizeof(server))) < 0)
	{
		LOG_ERROR("bind failed: %d\n", errno);
		lwip_close(s);
		return -1;
	}

	if ((err = lwip_listen(s, 2)) < 0)
	{
		LOG_ERROR("listen failed: %d\n", errno);
		lwip_close(s);
		return -1;
	}

	len = sizeof(struct sockaddr_in);

	LOG_INFO("Boot time: %d ms\n", (get_clock_tick() * 1000) / TIMER_FREQ);
	LOG_INFO("TCP server is listening.\n");

	if ((c = lwip_accept(s, (struct sockaddr *)&client, (socklen_t*)&len)) < 0)
	{
		LOG_ERROR("accept faild: %d\n", errno);
		lwip_close(s);
		return -1;
	}

	LOG_INFO("Establish IP connection\n");

	lwip_setsockopt(c, SOL_SOCKET, SO_RCVBUF, (char *) &sobufsize, sizeof(sobufsize));
	lwip_setsockopt(c, SOL_SOCKET, SO_SNDBUF, (char *) &sobufsize, sizeof(sobufsize));
	flag = 1;
	lwip_setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(flag));
	flag = 0;
	lwip_setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, (char *) &flag, sizeof(flag));

	magic = 0;
	lwip_read(c, &magic, sizeof(magic));
	if (magic != HERMIT_MAGIC)
	{
		LOG_ERROR("Invalid magic number %d\n", magic);
		lwip_close(c);
		return -1;
	}

	err = lwip_read(c, &argc, sizeof(argc));
	if (err != sizeof(argc))
		goto out;

	argv = kmalloc((argc+1)*sizeof(char*));
	if (!argv)
		goto out;
	memset(argv, 0x00, (argc+1)*sizeof(char*));

	for(i=0; i<argc; i++)
	{
		err = lwip_read(c, &len, sizeof(len));
		if (err != sizeof(len))
			goto out;

		argv[i] = kmalloc(len);
		if (!argv[i])
			goto out;

		j = 0;
		while(j < len) {
			err = lwip_read(c, argv[i]+j, len-j);
			if (err < 0)
				goto out;
			j += err;
		}

	}

	err = lwip_read(c, &envc, sizeof(envc));
	if (err != sizeof(envc))
		goto out;

	environ = kmalloc((envc+1)*sizeof(char**));
	if (!environ)
		goto out;
	memset(environ, 0x00, (envc+1)*sizeof(char*));

	for(i=0; i<envc; i++)
	{
		err = lwip_read(c, &len, sizeof(len));
		if (err != sizeof(len))
			goto out;

		environ[i] = kmalloc(len);
		if (!environ[i])
			goto out;

		j = 0;
		while(j < len) {
			err = lwip_read(c, environ[i]+j, len-j);
			if (err < 0)
				goto out;
			j += err;
		}
	}

	// call user code
	libc_sd = c;
	libc_start(argc, argv, environ);

out:
	if (argv) {
		for(i=0; i<argc; i++) {
			if (argv[i])
				kfree(argv[i]);
		}

		kfree(argv);
	}

	if (environ) {
		i = 0;
		while(environ[i]) {
			kfree(environ[i]);
			i++;
		}

		kfree(environ);
	}

	if (c > 0)
		lwip_close(c);
	libc_sd = -1;

	if (s > 0)
		lwip_close(s);
#endif
	return 0;
}

//#define MEASURE_CONTEXT

#ifdef MEASURE_CONTEXT

#define N	10000
volatile int finished = 0;
volatile int started1 = 0;
volatile int started2 = 0;

static int dummy_task(void* arg)
{
	kprintf("Enter dummy loop at core %d\n", CORE_ID);

	// cache warm up
	reschedule();
	reschedule();

	// synchronize start
	started2 = 1;
	mb();
	while(started1 == 0)
		reschedule();

	while (finished == 0)
	{
		reschedule();
	}

	kprintf("Leave dummy loop\n");

	return 0;
}

static int measure_context(void* arg)
{
	unsigned long long start, end;

	kprintf("Enter function at core %d to measure the time for a context switch\n", CORE_ID);

	// cache warm up
	reschedule();
	reschedule();

	// synchronize start
	started1 = 1;
	mb();
	while(started2 == 0)
	reschedule();

	// Save the current Time Stamp Counter value and switch to the second task.
	start = rdtsc();
	mb();

	for(uint32_t i = 0; i < N; i++)
	{
		reschedule();
	}

	// Calculate the cycle difference and add it to the sum.
	end = rdtsc();
	mb();

	finished = 1;

	reschedule();
	reschedule();

	kprintf("Average time for a task switch: %lld cycles\n", (end - start) / (N * 2));

	sys_exit(0);

	return 0;
}
#endif

int hermit_main(void)
{
	hermit_init();
	system_calibration(); // enables also interrupts

	LOG_INFO("This is Hermit %s, build on %s\n", PACKAGE_VERSION, __DATE__);
	//LOG_INFO("Isle %d of %d possible isles\n", isle, possible_isles);
	LOG_INFO("Kernel starts at %p and ends at %p\n", &kernel_start, (size_t)&kernel_start + image_size);
	LOG_INFO("TLS image starts at %p and ends at %p (size 0x%zx)\n", &tls_start, &tls_end, ((size_t) &tls_end) - ((size_t) &tls_start));
	LOG_INFO("BBS starts at %p and ends at %p\n", &tdata_end, (size_t)&kernel_start + image_size);
	LOG_INFO("Per core data starts at %p and ends at %p\n", &percore_start, &percore_end);
	LOG_INFO("Per core size 0x%zx\n", (size_t) &percore_end0 - (size_t) &percore_start);
	if (get_cpu_frequency() > 0)
		LOG_INFO("Processor frequency: %u MHz\n", get_cpu_frequency());
	LOG_INFO("Total memory: %zd MiB\n", atomic_int64_read(&total_pages) * PAGE_SIZE / (1024ULL*1024ULL));
	LOG_INFO("Current allocated memory: %zd KiB\n", atomic_int64_read(&total_allocated_pages) * PAGE_SIZE / 1024ULL);
	LOG_INFO("Current available memory: %zd MiB\n", atomic_int64_read(&total_available_pages) * PAGE_SIZE / (1024ULL*1024ULL));
	LOG_INFO("Core %d is the boot processor\n", boot_processor);
	LOG_INFO("System is able to use %d processors\n", possible_cpus);
	if (get_cmdline())
		LOG_INFO("Kernel cmdline: %s\n", get_cmdline());
	if (has_hbmem())
		LOG_INFO("Found high bandwidth memory at 0x%zx (size 0x%zx)\n", get_hbmem_base(), get_hbmem_size());
#ifdef KATA
	LOG_INFO("This build is for KATA\n");
#endif

#if 0
	print_pci_adapters();
#endif

#ifdef DYNAMIC_TICKS
	enable_dynticks();
#endif

	/* wait for the other cpus */
	while(atomic_int32_read(&cpu_online) < atomic_int32_read(&possible_cpus))
		PAUSE;

	print_cpu_status(isle);
	//vma_dump();

#ifdef MEASURE_CONTEXT
	create_kernel_task_on_core(NULL, dummy_task, NULL, NORMAL_PRIO, boot_processor);
	create_kernel_task_on_core(NULL, measure_context, NULL, NORMAL_PRIO, boot_processor);
#else
	create_kernel_task_on_core(NULL, initd, NULL, NORMAL_PRIO, boot_processor);
#endif

	while(1) {
		check_workqueues();
		wait_for_task();
	}

	return 0;
}
