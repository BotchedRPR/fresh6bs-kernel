/*
 * Linux OS Independent Layer
 *
 * Copyright (C) 2024 Synaptics Incorporated. All rights reserved.
 *
 * This software is licensed to you under the terms of the
 * GNU General Public License version 2 (the "GPL") with Broadcom special exception.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION
 * DOES NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES,
 * SYNAPTICS' TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT
 * EXCEED ONE HUNDRED U.S. DOLLARS
 *
 * Copyright (C) 2024, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 */

#define LINUX_PORT

#include <typedefs.h>
#include <bcmendian.h>
#include <linuxver.h>
#include <bcmdefs.h>

#if defined(__ARM_ARCH_7A__) && !defined(DHD_USE_COHERENT_MEM_FOR_RING)
#include <asm/cacheflush.h>
#endif /* __ARM_ARCH_7A__ && !DHD_USE_COHERENT_MEM_FOR_RING */

#include <linux/random.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
#include <linux/sched/clock.h>
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0) */

#include <osl.h>
#include <bcmutils.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <pcicfg.h>

#if defined(BCMASSERT_LOG) && !defined(OEM_ANDROID)
#include <bcm_assert_log.h>
#endif /* BCMASSERT_LOG && !OEM_ANDROID */

#include <linux/fs.h>

#ifdef BCM_OBJECT_TRACE
#include <bcmutils.h>
#endif /* BCM_OBJECT_TRACE */
#include "linux_osl_priv.h"

#define PCI_CFG_RETRY		10	/* PR15065: retry count for pci cfg accesses */

#define DUMPBUFSZ 1024

#ifdef CUSTOMER_HW4_DEBUG
uint32 g_assert_type = 1; /* By Default not cause Kernel Panic */
#else
uint32 g_assert_type = 0; /* By Default Kernel Panic */
#endif /* CUSTOMER_HW4_DEBUG */

module_param(g_assert_type, int, 0);

#ifdef USE_DMA_LOCK
static void osl_dma_lock(osl_t *osh);
static void osl_dma_unlock(osl_t *osh);
static void osl_dma_lock_init(osl_t *osh);

#define DMA_LOCK(osh)		osl_dma_lock(osh)
#define DMA_UNLOCK(osh)		osl_dma_unlock(osh)
#define DMA_LOCK_INIT(osh)	osl_dma_lock_init(osh);
#else
#define DMA_LOCK(osh)		do { /* noop */ } while(0)
#define DMA_UNLOCK(osh)		do { /* noop */ } while(0)
#define DMA_LOCK_INIT(osh)	do { /* noop */ } while(0)
#endif /* USE_DMA_LOCK */

static int16 linuxbcmerrormap[] =
{	0,				/* 0 */
	-EINVAL,		/* BCME_ERROR */
	-EINVAL,		/* BCME_BADARG */
	-EINVAL,		/* BCME_BADOPTION */
	-EINVAL,		/* BCME_NOTUP */
	-EINVAL,		/* BCME_NOTDOWN */
	-EINVAL,		/* BCME_NOTAP */
	-EINVAL,		/* BCME_NOTSTA */
	-EINVAL,		/* BCME_BADKEYIDX */
	-EINVAL,		/* BCME_RADIOOFF */
	-EINVAL,		/* BCME_NOTBANDLOCKED */
	-EINVAL,		/* BCME_NOCLK */
	-EINVAL,		/* BCME_BADRATESET */
	-EINVAL,		/* BCME_BADBAND */
	-E2BIG,			/* BCME_BUFTOOSHORT */
	-E2BIG,			/* BCME_BUFTOOLONG */
	-EBUSY,			/* BCME_BUSY */
	-EINVAL,		/* BCME_NOTASSOCIATED */
	-EINVAL,		/* BCME_BADSSIDLEN */
	-EINVAL,		/* BCME_OUTOFRANGECHAN */
	-EINVAL,		/* BCME_BADCHAN */
	-EFAULT,		/* BCME_BADADDR */
	-ENOMEM,		/* BCME_NORESOURCE */
	-EOPNOTSUPP,		/* BCME_UNSUPPORTED */
	-EMSGSIZE,		/* BCME_BADLENGTH */
	-EINVAL,		/* BCME_NOTREADY */
	-EPERM,			/* BCME_EPERM */
	-ENOMEM,		/* BCME_NOMEM */
	-EINVAL,		/* BCME_ASSOCIATED */
	-ERANGE,		/* BCME_RANGE */
	-EINVAL,		/* BCME_NOTFOUND */
	-EINVAL,		/* BCME_WME_NOT_ENABLED */
	-EINVAL,		/* BCME_TSPEC_NOTFOUND */
	-EINVAL,		/* BCME_ACM_NOTSUPPORTED */
	-EINVAL,		/* BCME_NOT_WME_ASSOCIATION */
	-EIO,			/* BCME_SDIO_ERROR */
	-ENODEV,		/* BCME_DONGLE_DOWN */
	-EINVAL,		/* BCME_VERSION */
	-EIO,			/* BCME_TXFAIL */
	-EIO,			/* BCME_RXFAIL */
	-ENODEV,		/* BCME_NODEVICE */
	-EINVAL,		/* BCME_NMODE_DISABLED */
	-ENODATA,		/* BCME_NONRESIDENT */
	-EINVAL,		/* BCME_SCANREJECT */
	-EINVAL,		/* BCME_USAGE_ERROR */
	-EIO,			/* BCME_IOCTL_ERROR */
	-EIO,			/* BCME_SERIAL_PORT_ERR */
	-EOPNOTSUPP,		/* BCME_DISABLED, BCME_NOTENABLED */
	-EIO,			/* BCME_DECERR */
	-EIO,			/* BCME_ENCERR */
	-EIO,			/* BCME_MICERR */
	-ERANGE,		/* BCME_REPLAY */
	-EINVAL,		/* BCME_IE_NOTFOUND */
	-EINVAL,		/* BCME_DATA_NOTFOUND */
	-EINVAL,		/* BCME_NOT_GC */
	-EINVAL,		/* BCME_PRS_REQ_FAILED */
	-EINVAL,		/* BCME_NO_P2P_SE */
	-EINVAL,		/* BCME_NOA_PND */
	-EINVAL,		/* BCME_FRAG_Q_FAILED */
	-EINVAL,		/* BCME_GET_AF_FAILED */
	-EINVAL,		/* BCME_MSCH_NOTREADY */
	-EINVAL,		/* BCME_IOV_LAST_CMD */
	-EINVAL,		/* BCME_MINIPMU_CAL_FAIL */
	-EINVAL,		/* BCME_RCAL_FAIL */
	-EINVAL,		/* BCME_LPF_RCCAL_FAIL */
	-EINVAL,		/* BCME_DACBUF_RCCAL_FAIL */
	-EINVAL,		/* BCME_VCOCAL_FAIL */
	-EINVAL,		/* BCME_BANDLOCKED */
	-EINVAL,		/* BCME_BAD_IE_DATA */
	-EINVAL,		/* BCME_REG_FAILED */
	-EINVAL,		/* BCME_NOCHAN */
	-EINVAL,		/* BCME_PKTTOSS */
	-EINVAL,		/* BCME_DNGL_DEVRESET */
	-EINVAL,		/* BCME_ROAM */
	-EOPNOTSUPP,		/* BCME_NO_SIG_FILE */

/* When an new error code is added to bcmutils.h, add os
 * specific error translation here as well
 */
/* check if BCME_LAST changed since the last time this function was updated */
#if BCME_LAST != BCME_NO_SIG_FILE
#error "You need to add a OS error translation in the linuxbcmerrormap \
	for new error code defined in bcmutils.h"
#endif
};
uint lmtest = FALSE;

#ifdef DHD_MAP_LOGGING
#define DHD_MAP_LOG_SIZE 2048

typedef struct dhd_map_item {
	dmaaddr_t pa;		/* DMA address (physical) */
	uint64 ts_nsec;		/* timestamp: nsec */
	uint32 size;		/* mapping size */
	uint8 rsvd[4];		/* reserved for future use */
} dhd_map_item_t;

typedef struct dhd_map_record {
	uint32 items;		/* number of total items */
	uint32 idx;		/* current index of metadata */
	dhd_map_item_t map[0];	/* metadata storage */
} dhd_map_log_t;

void
osl_dma_map_dump(osl_t *osh)
{
	dhd_map_log_t *map_log, *unmap_log;
	uint64 ts_sec, ts_usec;

	map_log = (dhd_map_log_t *)(osh->dhd_map_log);
	unmap_log = (dhd_map_log_t *)(osh->dhd_unmap_log);
	osl_get_localtime(&ts_sec, &ts_usec);

	if (map_log && unmap_log) {
		printk("%s: map_idx=%d unmap_idx=%d "
			"current time=[%5lu.%06lu]\n", __FUNCTION__,
			map_log->idx, unmap_log->idx, (unsigned long)ts_sec,
			(unsigned long)ts_usec);
		printk("%s: dhd_map_log(pa)=0x%llx size=%d,"
			" dma_unmap_log(pa)=0x%llx size=%d\n", __FUNCTION__,
			(uint64)__virt_to_phys((ulong)(map_log->map)),
			(uint32)(sizeof(dhd_map_item_t) * map_log->items),
			(uint64)__virt_to_phys((ulong)(unmap_log->map)),
			(uint32)(sizeof(dhd_map_item_t) * unmap_log->items));
	}
}

static void *
osl_dma_map_log_init(uint32 item_len)
{
	dhd_map_log_t *map_log;
	gfp_t flags;
	uint32 alloc_size = (uint32)(sizeof(dhd_map_log_t) +
		(item_len * sizeof(dhd_map_item_t)));

	flags = CAN_SLEEP() ? GFP_KERNEL : GFP_ATOMIC;
	map_log = (dhd_map_log_t *)KVMALLOC(alloc_size, flags);
	if (map_log) {
		memset(map_log, 0, alloc_size);
		map_log->items = item_len;
		map_log->idx = 0;
	}

	return (void *)map_log;
}

static void
osl_dma_map_log_deinit(osl_t *osh)
{
	if (osh->dhd_map_log) {
		KVFREE(osh->dhd_map_log);
		osh->dhd_map_log = NULL;
	}

	if (osh->dhd_unmap_log) {
		KVFREE(osh->dhd_unmap_log);
		osh->dhd_unmap_log = NULL;
	}
}

static void
osl_dma_map_logging(osl_t *osh, void *handle, dmaaddr_t pa, uint32 len)
{
	dhd_map_log_t *log = (dhd_map_log_t *)handle;
	uint32 idx;

	if (log == NULL) {
		printk("%s: log is NULL\n", __FUNCTION__);
		return;
	}

	idx = log->idx;
	log->map[idx].ts_nsec = osl_localtime_ns();
	log->map[idx].pa = pa;
	log->map[idx].size = len;
	log->idx = (idx + 1) % log->items;
}
#endif /* DHD_MAP_LOGGING */

/* translate bcmerrors into linux errors */
int
osl_error(int bcmerror)
{
	if (bcmerror > 0)
		bcmerror = 0;
	else if (bcmerror < BCME_LAST)
		bcmerror = BCME_ERROR;

	/* Array bounds covered by ASSERT in osl_attach */
	return linuxbcmerrormap[-bcmerror];
}
osl_t *
osl_attach(void *pdev, uint bustype, bool pkttag)
{
	void **osl_cmn = NULL;
	osl_t *osh;
	gfp_t flags;

	flags = CAN_SLEEP() ? GFP_KERNEL: GFP_ATOMIC;
	if (!(osh = KVMALLOC(sizeof(osl_t), flags)))
		return osh;

	ASSERT(osh);

	bzero(osh, sizeof(osl_t));

	if (osl_cmn == NULL || *osl_cmn == NULL) {
		if (!(osh->cmn = KVMALLOC(sizeof(osl_cmn_t), flags))) {
			KVFREE(osh);
			return NULL;
		}
		bzero(osh->cmn, sizeof(osl_cmn_t));
		if (osl_cmn)
			*osl_cmn = osh->cmn;
		atomic_set(&osh->cmn->malloced, 0);
		osh->cmn->dbgmem_list = NULL;
		spin_lock_init(&(osh->cmn->dbgmem_lock));

		spin_lock_init(&(osh->cmn->pktalloc_lock));

	} else {
		osh->cmn = *osl_cmn;
	}
	atomic_add(1, &osh->cmn->refcount);

	bcm_object_trace_init();
	/* Check that error map has the right number of entries in it */
	ASSERT(ABS(BCME_LAST) == (ARRAYSIZE(linuxbcmerrormap) - 1));
	osh->failed = 0;
	osh->pdev = pdev;
	osh->pub.pkttag = pkttag;
	osh->bustype = bustype;
	osh->magic = OS_HANDLE_MAGIC;

	switch (bustype) {
		case PCI_BUS:
		case SI_BUS:
			osh->pub.mmbus = TRUE;
			break;
		case SDIO_BUS:
		case USB_BUS:
		case SPI_BUS:
		case RPC_BUS:
			osh->pub.mmbus = FALSE;
			break;
		default:
			ASSERT(FALSE);
			break;
	}

	DMA_LOCK_INIT(osh);

#ifdef DHD_MAP_LOGGING
	osh->dhd_map_log = osl_dma_map_log_init(DHD_MAP_LOG_SIZE);
	if (osh->dhd_map_log == NULL) {
		printk("%s: Failed to alloc dhd_map_log\n", __FUNCTION__);
	}

	osh->dhd_unmap_log = osl_dma_map_log_init(DHD_MAP_LOG_SIZE);
	if (osh->dhd_unmap_log == NULL) {
		printk("%s: Failed to alloc dhd_unmap_log\n", __FUNCTION__);
	}
#endif /* DHD_MAP_LOGGING */

	return osh;
}

void osl_set_bus_handle(osl_t *osh, void *bus_handle)
{
	osh->bus_handle = bus_handle;
}

void* osl_get_bus_handle(osl_t *osh)
{
	return osh->bus_handle;
}

#if defined(AXI_TIMEOUTS_NIC)
void osl_set_bpt_cb(osl_t *osh, void *bpt_cb, void *bpt_ctx)
{
	if (osh) {
		osh->bpt_cb = (bpt_cb_fn)bpt_cb;
		osh->sih = bpt_ctx;
	}
}
#endif /* AXI_TIMEOUTS_NIC */

void
osl_detach(osl_t *osh)
{
	if (osh == NULL) {
		bcm_object_trace_deinit();
	} else {
		bcm_object_trace_deinit();

#ifdef DHD_MAP_LOGGING
		osl_dma_map_log_deinit(osh);
#endif /* DHD_MAP_LOGGING */

		ASSERT(osh->magic == OS_HANDLE_MAGIC);
		atomic_sub(1, &osh->cmn->refcount);
		if (atomic_read(&osh->cmn->refcount) == 0) {
				KVFREE(osh->cmn);
		}
		KVFREE(osh);
	}
}

/* APIs to set/get specific quirks in OSL layer */
void
BCMFASTPATH(osl_flag_set)(osl_t *osh, uint32 mask)
{
	osh->flags |= mask;
}

void
osl_flag_clr(osl_t *osh, uint32 mask)
{
	osh->flags &= ~mask;
}

bool
osl_is_flag_set(osl_t *osh, uint32 mask)
{
	return (osh->flags & mask);
}

#if (defined(__ARM_ARCH_7A__) && !defined(DHD_USE_COHERENT_MEM_FOR_RING))

inline void
BCMFASTPATH(osl_cache_flush)(void *va, uint size)
{
	if (size > 0)
		dma_sync_single_for_device(OSH_NULL, virt_to_phys(va), size,
			DMA_TO_DEVICE);
}

inline void
BCMFASTPATH(osl_cache_inv)(void *va, uint size)
{
	dma_sync_single_for_cpu(OSH_NULL, virt_to_phys(va), size, DMA_FROM_DEVICE);
}

inline void
BCMFASTPATH(osl_prefetch)(const void *ptr)
{
	__asm__ __volatile__("pld\t%0" :: "o"(*(const char *)ptr) : "cc");
}

#endif /* !__ARM_ARCH_7A__ */

uint32
osl_pci_read_config(osl_t *osh, uint offset, uint size)
{
	uint val = 0;
	uint retry = PCI_CFG_RETRY;	 /* PR15065: faulty cardbus controller bug */

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	/* only 4byte access supported */
	ASSERT(size == 4);

	do {
		pci_read_config_dword(osh->pdev, offset, &val);
		if (val != 0xffffffff)
			break;
	} while (retry--);

	return (val);
}

void
osl_pci_write_config(osl_t *osh, uint offset, uint size, uint val)
{
	uint retry = PCI_CFG_RETRY;	 /* PR15065: faulty cardbus controller bug */

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	/* only 4byte access supported */
	ASSERT(size == 4);

	do {
		pci_write_config_dword(osh->pdev, offset, val);
		/* PR15065: PCI_BAR0_WIN is believed to be the only pci cfg write that can occur
		 * when dma activity is possible
		 */
		if (offset != PCI_BAR0_WIN)
			break;
		if (osl_pci_read_config(osh, offset, size) == val)
			break;
	} while (retry--);

}

/* return bus # for the pci device pointed by osh->pdev */
uint
osl_pci_bus(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

#if defined(__ARM_ARCH_7A__)
	return pci_domain_nr(((struct pci_dev *)osh->pdev)->bus);
#else
	return ((struct pci_dev *)osh->pdev)->bus->number;
#endif
}

/* return slot # for the pci device pointed by osh->pdev */
uint
osl_pci_slot(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

#if defined(__ARM_ARCH_7A__)
	return PCI_SLOT(((struct pci_dev *)osh->pdev)->devfn) + 1;
#else
	return PCI_SLOT(((struct pci_dev *)osh->pdev)->devfn);
#endif
}

/* return domain # for the pci device pointed by osh->pdev */
uint
osl_pcie_domain(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return pci_domain_nr(((struct pci_dev *)osh->pdev)->bus);
}

/* return bus # for the pci device pointed by osh->pdev */
uint
osl_pcie_bus(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return ((struct pci_dev *)osh->pdev)->bus->number;
}

struct device *osl_get_device(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return osh->pdev;
}

/* return the pci device pointed by osh->pdev */
struct pci_dev *
osl_pci_device(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return osh->pdev;
}

void *
osl_malloc(osl_t *osh, uint size)
{
	void *addr;
	gfp_t flags;

	/* only ASSERT if osh is defined */
	if (osh)
		ASSERT(osh->magic == OS_HANDLE_MAGIC);
#ifdef CONFIG_DHD_USE_STATIC_BUF
	if (bcm_static_buf)
	{
		unsigned long irq_flags;
		int i = 0;
		if ((size >= PAGE_SIZE)&&(size <= STATIC_BUF_SIZE))
		{
			OSL_STATIC_BUF_LOCK(&bcm_static_buf->static_lock, irq_flags);

			for (i = 0; i < STATIC_BUF_MAX_NUM; i++)
			{
				if (bcm_static_buf->buf_use[i] == 0)
					break;
			}

			if (i == STATIC_BUF_MAX_NUM)
			{
				OSL_STATIC_BUF_UNLOCK(&bcm_static_buf->static_lock, irq_flags);
				printk("all static buff in use!\n");
				goto original;
			}

			bcm_static_buf->buf_use[i] = 1;
			OSL_STATIC_BUF_UNLOCK(&bcm_static_buf->static_lock, irq_flags);

			bzero(bcm_static_buf->buf_ptr+STATIC_BUF_SIZE*i, size);
			if (osh)
				atomic_add(size, &osh->cmn->malloced);

			return ((void *)(bcm_static_buf->buf_ptr+STATIC_BUF_SIZE*i));
		}
	}
original:
#endif /* CONFIG_DHD_USE_STATIC_BUF */

	flags = CAN_SLEEP() ? GFP_KERNEL: GFP_ATOMIC;
	if ((addr = KVMALLOC(size, flags)) == NULL) {
		if (osh)
			osh->failed++;
		return (NULL);
	}
	if (osh && osh->cmn)
		atomic_add(size, &osh->cmn->malloced);

	return (addr);
}

void *
osl_mallocz(osl_t *osh, uint size)
{
	void *ptr;

	ptr = osl_malloc(osh, size);

	if (ptr != NULL) {
		bzero(ptr, size);
	}

	return ptr;
}

void
osl_mfree(osl_t *osh, void *addr, uint size)
{
#ifdef CONFIG_DHD_USE_STATIC_BUF
	unsigned long flags;

	if (addr == NULL) {
		return;
	}

	if (bcm_static_buf)
	{
		if ((addr > (void *)bcm_static_buf) && ((unsigned char *)addr
			<= ((unsigned char *)bcm_static_buf + STATIC_BUF_TOTAL_LEN)))
		{
			int buf_idx = 0;

			buf_idx = ((unsigned char *)addr - bcm_static_buf->buf_ptr)/STATIC_BUF_SIZE;

			OSL_STATIC_BUF_LOCK(&bcm_static_buf->static_lock, flags);
			bcm_static_buf->buf_use[buf_idx] = 0;
			OSL_STATIC_BUF_UNLOCK(&bcm_static_buf->static_lock, flags);

			if (osh && osh->cmn) {
				ASSERT(osh->magic == OS_HANDLE_MAGIC);
				atomic_sub(size, &osh->cmn->malloced);
			}
			return;
		}
	}
#endif /* CONFIG_DHD_USE_STATIC_BUF */
	if (osh && osh->cmn) {
		ASSERT(osh->magic == OS_HANDLE_MAGIC);

		ASSERT(size <= osl_malloced(osh));

		atomic_sub(size, &osh->cmn->malloced);
	}
	KVFREE(addr);
}

void *
osl_vmalloc(osl_t *osh, uint size)
{
	void *addr;

	/* only ASSERT if osh is defined */
	if (osh)
		ASSERT(osh->magic == OS_HANDLE_MAGIC);
	if ((addr = vmalloc(size)) == NULL) {
		if (osh)
			osh->failed++;
		return (NULL);
	}
	if (osh && osh->cmn)
		atomic_add(size, &osh->cmn->malloced);

	return (addr);
}

void *
osl_vmallocz(osl_t *osh, uint size)
{
	void *ptr;

	ptr = osl_vmalloc(osh, size);

	if (ptr != NULL) {
		bzero(ptr, size);
	}

	return ptr;
}

void
osl_vmfree(osl_t *osh, void *addr, uint size)
{
	if (osh && osh->cmn) {
		ASSERT(osh->magic == OS_HANDLE_MAGIC);

		ASSERT(size <= osl_malloced(osh));

		atomic_sub(size, &osh->cmn->malloced);
	}
	vfree(addr);
}

uint
osl_check_memleak(osl_t *osh)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	if (atomic_read(&osh->cmn->refcount) == 1)
		return (atomic_read(&osh->cmn->malloced));
	else
		return 0;
}

uint
osl_malloced(osl_t *osh)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	return (atomic_read(&osh->cmn->malloced));
}

uint
osl_malloc_failed(osl_t *osh)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	return (osh->failed);
}

uint
osl_dma_consistent_align(void)
{
	return (PAGE_SIZE);
}

void*
osl_dma_alloc_consistent(osl_t *osh, uint size, uint16 align_bits, uint *alloced, dmaaddr_t *pap)
{
	void *va;
	uint16 align = (1 << align_bits);
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	if (!ISALIGNED(DMA_CONSISTENT_ALIGN, align))
		size += align;
	*alloced = size;

#if (defined(__ARM_ARCH_7A__) && !defined(DHD_USE_COHERENT_MEM_FOR_RING))
	va = KVMALLOC(size, GFP_ATOMIC | __GFP_ZERO);
	if (va)
		*pap = (ulong)__virt_to_phys((ulong)va);
#else
	{
		dma_addr_t pap_lin;
		struct pci_dev *hwdev = osh->pdev;
		gfp_t flags;
#ifdef DHD_ALLOC_COHERENT_MEM_FROM_ATOMIC_POOL
		flags = GFP_ATOMIC;
#else
		flags = CAN_SLEEP() ? GFP_KERNEL: GFP_ATOMIC;
#endif /* DHD_ALLOC_COHERENT_MEM_FROM_ATOMIC_POOL */
#ifdef DHD_ALLOC_COHERENT_MEM_WITH_GFP_COMP
		flags |= __GFP_COMP;
#endif /* DHD_ALLOC_COHERENT_MEM_WITH_GFP_COMP */
		va = dma_alloc_coherent(&hwdev->dev, size, &pap_lin, flags);
#ifdef BCMDMA64OSL
		PHYSADDRLOSET(*pap, pap_lin & 0xffffffff);
		PHYSADDRHISET(*pap, (pap_lin >> 32) & 0xffffffff);
#else
		*pap = (dmaaddr_t)pap_lin;
#endif /* BCMDMA64OSL */
	}
#endif /* __ARM_ARCH_7A__ && !DHD_USE_COHERENT_MEM_FOR_RING */

	return va;
}

void
osl_dma_free_consistent(osl_t *osh, void *va, uint size, dmaaddr_t pa)
{
	struct pci_dev *hwdev = NULL;
#ifdef BCMDMA64OSL
	dma_addr_t paddr;
#endif /* BCMDMA64OSL */
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	UNUSED_PARAMETER(hwdev);

#if (defined(__ARM_ARCH_7A__) && !defined(DHD_USE_COHERENT_MEM_FOR_RING))
	KVFREE(va);
#else /* (defined(__ARM_ARCH_7A__) && !defined(DHD_USE_COHERENT_MEM_FOR_RING)) */
	hwdev = osh->pdev;
#ifdef BCMDMA64OSL
	PHYSADDRTOULONG(pa, paddr);
	dma_free_coherent(&hwdev->dev, size, va, paddr);
#else /* BCMDMA64OSL */
	dma_free_coherent(&hwdev->dev, size, va, (dma_addr_t)pa);
#endif /* BCMDMA64OSL */
#endif /* __ARM_ARCH_7A__ && !DHD_USE_COHERENT_MEM_FOR_RING */
}

void *
osl_virt_to_phys(void *va)
{
	return (void *)(uintptr)virt_to_phys(va);
}

#include <asm/cacheflush.h>
void
BCMFASTPATH(osl_dma_flush)(osl_t *osh, void *va, uint size, int direction, void *p,
	hnddma_seg_map_t *dmah)
{
	return;
}

dmaaddr_t
BCMFASTPATH(osl_dma_map)(osl_t *osh, void *va, uint size, int direction, void *p,
	hnddma_seg_map_t *dmah)
{
	struct pci_dev *hwdev = osh->pdev;
	dmaaddr_t ret_addr;
	dma_addr_t map_addr;
	int ret;

	DMA_LOCK(osh);

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	map_addr = dma_map_single(&hwdev->dev, va, size, direction);

	ret = dma_mapping_error(&hwdev->dev, map_addr);

	if (ret) {
		printk("%s: Failed to map memory:%d\n", __FUNCTION__, ret);
		PHYSADDRLOSET(ret_addr, 0);
		PHYSADDRHISET(ret_addr, 0);
	} else {
		PHYSADDRLOSET(ret_addr, map_addr & 0xffffffff);
		PHYSADDRHISET(ret_addr, (map_addr >> 32) & 0xffffffff);
	}

#ifdef DHD_MAP_LOGGING
	osl_dma_map_logging(osh, osh->dhd_map_log, ret_addr, size);
#endif /* DHD_MAP_LOGGING */

	DMA_UNLOCK(osh);

	return ret_addr;
}

void
BCMFASTPATH(osl_dma_unmap)(osl_t *osh, dmaaddr_t pa, uint size, int direction)
{
#ifdef BCMDMA64OSL
	dma_addr_t paddr;
#endif /* BCMDMA64OSL */
	struct pci_dev *hwdev = osh->pdev;

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	DMA_LOCK(osh);

#ifdef DHD_MAP_LOGGING
	osl_dma_map_logging(osh, osh->dhd_unmap_log, pa, size);
#endif /* DHD_MAP_LOGGING */

#ifdef BCMDMA64OSL
	PHYSADDRTOULONG(pa, paddr);
	dma_unmap_single(&hwdev->dev, paddr, size, direction);
#else /* BCMDMA64OSL */
	dma_unmap_single(&hwdev->dev, (uint32)pa, size, direction);
#endif /* BCMDMA64OSL */

	DMA_UNLOCK(osh);
}

/* OSL function for CPU relax */
inline void
BCMFASTPATH(osl_cpu_relax)(void)
{
	cpu_relax();
}

extern void osl_preempt_disable(osl_t *osh)
{
	preempt_disable();
}

extern void osl_preempt_enable(osl_t *osh)
{
	preempt_enable();
}

#if defined(BCMASSERT_LOG)
void
osl_assert(const char *exp, const char *file, int line)
{
	char tempbuf[256];
	const char *basename;

	basename = strrchr(file, '/');
	/* skip the '/' */
	if (basename)
		basename++;

	if (!basename)
		basename = file;

#ifdef BCMASSERT_LOG
	snprintf(tempbuf, 64, "\"%s\": file \"%s\", line %d\n",
		exp, basename, line);
#ifndef OEM_ANDROID
	bcm_assert_log(tempbuf);
#endif /* OEM_ANDROID */
#endif /* BCMASSERT_LOG */

	switch (g_assert_type) {
	case 0:
		printk("%s", tempbuf);
		WARN_ON(1);
		break;
	case 1:
		/* fall through */
	case 3:
		printk("%s", tempbuf);
		break;
	case 2:
		printk("%s", tempbuf);
		WARN_ON(1);
		break;
	default:
		break;
	}
}
#endif

void
osl_delay(uint usec)
{
	uint d;

	while (usec > 0) {
		d = MIN(usec, 1000);
		udelay(d);
		usec -= d;
	}
}

void
osl_sleep(uint ms)
{

	if (ms < 20)
		usleep_range(ms*1000, ms*1000 + 1000);
	else
		msleep(ms);
}

uint64
osl_sysuptime_us(void)
{
	struct timespec64 ts;
	uint64 usec;

	ktime_get_real_ts64(&ts);
	/* tv_nsec content is fraction of a second */
	usec = (uint64)ts.tv_sec * USEC_PER_SEC + (ts.tv_nsec / NSEC_PER_USEC);
	return usec;
}

uint64
osl_localtime_ns(void)
{
	uint64 ts_nsec = 0;

	/* Some Linux based platform cannot use local_clock()
	 * since it is defined by EXPORT_SYMBOL_GPL().
	 * GPL-incompatible module (NIC builds wl.ko)
	 * cannnot use the GPL-only symbol.
	 */
	ts_nsec = local_clock();
	return ts_nsec;
}

void
osl_get_localtime(uint64 *sec, uint64 *usec)
{
	uint64 ts_nsec = 0;
	unsigned long rem_nsec = 0;

	/* Some Linux based platform cannot use local_clock()
	 * since it is defined by EXPORT_SYMBOL_GPL().
	 * GPL-incompatible module (NIC builds wl.ko) can
	 * not use the GPL-only symbol.
	 */
	ts_nsec = local_clock();
	rem_nsec = do_div(ts_nsec, NSEC_PER_SEC);
	*sec = (uint64)ts_nsec;
	*usec = (uint64)(rem_nsec / MSEC_PER_SEC);
}

uint64
osl_systztime_us(void)
{
	struct timespec64 ts;
	uint64 tzusec;

	ktime_get_real_ts64(&ts);
	/* apply timezone */
	tzusec = (uint64)((ts.tv_sec - (sys_tz.tz_minuteswest * 60u)) * USEC_PER_SEC);
	tzusec += ts.tv_nsec / NSEC_PER_USEC;

	return tzusec;
}

char *
osl_get_rtctime(void)
{
	static char timebuf[RTC_TIME_BUF_LEN];
	struct timespec64 ts;
	struct rtc_time tm;

	memset_s(timebuf, RTC_TIME_BUF_LEN, 0, RTC_TIME_BUF_LEN);
	ktime_get_real_ts64(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	scnprintf(timebuf, RTC_TIME_BUF_LEN,
			"%02d:%02d:%02d.%06lu",
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec/NSEC_PER_USEC);
	return timebuf;
}

/*
 * OSLREGOPS specifies the use of osl_XXX routines to be used for register access
 */

/*
 * BINOSL selects the slightly slower function-call-based binary compatible osl.
 */

uint32
osl_rand(void)
{
	uint32 rand;

	get_random_bytes(&rand, sizeof(rand));

	return rand;
}

#ifdef DHD_SUPPORT_VFS_CALL
/* Linux Kernel: File Operations: start */
void *
osl_os_open_image(char *filename)
{
	struct file *fp;

	fp = filp_open(filename, O_RDONLY, 0);
	/*
	 * 2.6.11 (FC4) supports filp_open() but later revs don't?
	 * Alternative:
	 * fp = open_namei(AT_FDCWD, filename, O_RD, 0);
	 * ???
	 */
	if (IS_ERR_OR_NULL(fp)) {
		printk("ERROR %ld: Unable to open file %s\n", PTR_ERR(fp), filename);
		fp = NULL;
	}

	return fp;
}

int
osl_os_get_image_block(char *buf, int len, void *image)
{
	struct file *fp = (struct file *)image;
	int rdlen;

	if (fp == NULL) {
		return 0;
	}

	rdlen = kernel_read_compat(fp, fp->f_pos, buf, len);
	if (rdlen > 0) {
		fp->f_pos += rdlen;
	}

	return rdlen;
}

void
osl_os_close_image(void *image)
{
	struct file *fp = (struct file *)image;

	if (fp != NULL) {
		filp_close(fp, NULL);
	}
}

int
osl_os_image_size(void *image)
{
	int len = 0, curroffset;

	if (image) {
		/* store the current offset */
		curroffset = generic_file_llseek(image, 0, 1);
		/* goto end of file to get length */
		len = generic_file_llseek(image, 0, 2);
		/* restore back the offset */
		generic_file_llseek(image, curroffset, 0);
	}
	return len;
}
#endif /* DHD_SUPPORT_VFS_CALL */

/* Linux Kernel: File Operations: end */

#if defined(AXI_TIMEOUTS_NIC)
inline void osl_bpt_rreg(osl_t *osh, ulong addr, volatile void *v, uint size)
{
	bool poll_timeout = FALSE;
	static int in_si_clear = FALSE;

	switch (size) {
	case sizeof(uint8):
		*(volatile uint8*)v = readb((volatile uint8*)(addr));
		if (*(volatile uint8*)v == 0xff)
			poll_timeout = TRUE;
		break;
	case sizeof(uint16):
		*(volatile uint16*)v = readw((volatile uint16*)(addr));
		if (*(volatile uint16*)v == 0xffff)
			poll_timeout = TRUE;
		break;
	case sizeof(uint32):
		*(volatile uint32*)v = readl((volatile uint32*)(addr));
		if (*(volatile uint32*)v == 0xffffffff)
			poll_timeout = TRUE;
		break;
	case sizeof(uint64):
		*(volatile uint64*)v = *((volatile uint64*)(addr));
		if (*(volatile uint64*)v == 0xffffffffffffffff)
			poll_timeout = TRUE;
		break;
	}

	if (osh && osh->sih && (in_si_clear == FALSE) && poll_timeout && osh->bpt_cb) {
		in_si_clear = TRUE;
		osh->bpt_cb((void *)osh->sih, (void *)addr);
		in_si_clear = FALSE;
	}
}
#endif /* AXI_TIMEOUTS_NIC */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
void
timer_cb_compat(struct timer_list *tl)
{
	timer_list_compat_t *t = container_of(tl, timer_list_compat_t, timer);
	t->callback((void*)t->arg);
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0) */

/* timer apis */
/* Note: All timer api's are thread unsafe and should be protected with locks by caller */

osl_timer_t *
osl_timer_init(osl_t *osh, const char *name, void (*fn)(void *arg), void *arg)
{
	osl_timer_t *t;
	BCM_REFERENCE(fn);
	if ((t = MALLOCZ(osh, sizeof(osl_timer_t))) == NULL) {
		printk(KERN_ERR "osl_timer_init: out of memory, malloced %d bytes\n",
			(int)sizeof(osl_timer_t));
		return (NULL);
	}
	bzero(t, sizeof(osl_timer_t));
	if ((t->timer = MALLOCZ(osh, sizeof(timer_list_compat_t))) == NULL) {
		printf("osl_timer_init: malloc failed\n");
		MFREE(NULL, t, sizeof(osl_timer_t));
		return (NULL);
	}

	t->set = TRUE;

	init_timer_compat(t->timer, (linux_timer_fn)fn, arg);

	return (t);
}

void
osl_timer_add(osl_t *osh, osl_timer_t *t, uint32 ms, bool periodic)
{
	if (t == NULL) {
		printf("%s: Timer handle is NULL\n", __FUNCTION__);
		return;
	}
	ASSERT(!t->set);

	t->set = TRUE;
	if (periodic) {
		printf("Periodic timers are not supported by Linux timer apis\n");
	}
	timer_expires(t->timer) = jiffies + ms*HZ/1000;

	add_timer(t->timer);

	return;
}

void
osl_timer_update(osl_t *osh, osl_timer_t *t, uint32 ms, bool periodic)
{
	if (t == NULL) {
		printf("%s: Timer handle is NULL\n", __FUNCTION__);
		return;
	}
	if (periodic) {
		printf("Periodic timers are not supported by Linux timer apis\n");
	}
	t->set = TRUE;
	timer_expires(t->timer) = jiffies + ms*HZ/1000;

	mod_timer(t->timer, timer_expires(t->timer));

	return;
}

/*
 * Return TRUE if timer successfully deleted, FALSE if still pending
 */
bool
osl_timer_del(osl_t *osh, osl_timer_t *t)
{
	if (t == NULL) {
		printf("%s: Timer handle is NULL\n", __FUNCTION__);
		return (FALSE);
	}
	if (t->set) {
		t->set = FALSE;
		if (t->timer) {
			// use 'del_timer' rather than 'del_timer_sync'
			// to avoid kernel blocking during exception
			del_timer(t->timer);
			// fix misatch bug
			MFREE(osh, t->timer, sizeof(timer_list_compat_t));
		}
		MFREE(osh, t, sizeof(osl_timer_t));
	}
	return (TRUE);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
int
kernel_read_compat(struct file *file, loff_t offset, char *addr, unsigned long count)
{
#ifdef DHD_SUPPORT_VFS_CALL
	return (int)kernel_read(file, addr, (size_t)count, &offset);
#else
	return 0;
#endif /* DHD_SUPPORT_VFS_CALL */
}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)) */

/* Linux specific multipurpose spinlock API */
void *
osl_spin_lock_init(osl_t *osh)
{
	/* Adding 4 bytes since the sizeof(spinlock_t) could be 0 */
	/* if CONFIG_SMP and CONFIG_DEBUG_SPINLOCK are not defined */
	/* and this results in kernel asserts in internal builds */
	spinlock_t * lock = MALLOC(osh, sizeof(spinlock_t) + 4);
	if (lock)
		spin_lock_init(lock);
	return ((void *)lock);
}
void
osl_spin_lock_deinit(osl_t *osh, void *lock)
{
	if (lock)
		MFREE(osh, lock, sizeof(spinlock_t) + 4);
}

unsigned long
osl_spin_lock(void *lock)
{
	unsigned long flags = 0;

	if (lock) {
#ifdef DHD_USE_SPIN_LOCK_BH
		/* Calling spin_lock_bh with both irq and non-irq context will lead to deadlock */
		ASSERT(!in_irq());
		spin_lock_bh((spinlock_t *)lock);
#else
		spin_lock_irqsave((spinlock_t *)lock, flags);
#endif /* DHD_USE_SPIN_LOCK_BH */
	}

	return flags;
}

void
osl_spin_unlock(void *lock, unsigned long flags)
{
	if (lock) {
#ifdef DHD_USE_SPIN_LOCK_BH
		/* Calling spin_lock_bh with both irq and non-irq context will lead to deadlock */
		ASSERT(!in_irq());
		spin_unlock_bh((spinlock_t *)lock);
#else
		spin_unlock_irqrestore((spinlock_t *)lock, flags);
#endif /* DHD_USE_SPIN_LOCK_BH */
	}
}

unsigned long
osl_spin_lock_irq(void *lock)
{
	unsigned long flags = 0;

	if (lock)
		spin_lock_irqsave((spinlock_t *)lock, flags);

	return flags;
}

void
osl_spin_unlock_irq(void *lock, unsigned long flags)
{
	if (lock)
		spin_unlock_irqrestore((spinlock_t *)lock, flags);
}

unsigned long
osl_spin_lock_bh(void *lock)
{
	unsigned long flags = 0;

	if (lock) {
		/* Calling spin_lock_bh with both irq and non-irq context will lead to deadlock */
		ASSERT(!in_irq());
		spin_lock_bh((spinlock_t *)lock);
	}

	return flags;
}

void
osl_spin_unlock_bh(void *lock, unsigned long flags)
{
	if (lock) {
		/* Calling spin_lock_bh with both irq and non-irq context will lead to deadlock */
		ASSERT(!in_irq());
		spin_unlock_bh((spinlock_t *)lock);
	}
}

void *
osl_mutex_lock_init(osl_t *osh)
{
	struct mutex *mtx = NULL;

	mtx = MALLOCZ(osh, sizeof(*mtx));
	if (mtx)
		mutex_init(mtx);

	return mtx;
}

void
osl_mutex_lock_deinit(osl_t *osh, void *mtx)
{
	if (mtx) {
		mutex_destroy(mtx);
		MFREE(osh, mtx, sizeof(struct mutex));
	}
}

/* For mutex lock/unlock unsigned long flags is used,
 * this is to keep in sync with spin lock apis, so that
 * locks can be easily interchanged based on contexts
 */
unsigned long
osl_mutex_lock(void *lock)
{
	if (lock)
		mutex_lock((struct mutex *)lock);

	return 0;
}

void
osl_mutex_unlock(void *lock, unsigned long flags)
{
	if (lock)
		mutex_unlock((struct mutex *)lock);
	return;
}

#ifdef USE_DMA_LOCK
static void
osl_dma_lock(osl_t *osh)
{
	/* The conditional check is to avoid the scheduling bug.
	 * If the spin_lock_bh is used under the spin_lock_irqsave,
	 * Kernel triggered the warning message as the spin_lock_irqsave
	 * disables the interrupt and the spin_lock_bh doesn't use in case
	 * interrupt is disabled.
	 * Please refer to the __local_bh_enable_ip() function
	 * in kernel/softirq.c to understand the condtion.
	 */
	if (likely(in_irq() || irqs_disabled())) {
		spin_lock(&osh->dma_lock);
	} else {
		spin_lock_bh(&osh->dma_lock);
		osh->dma_lock_bh = TRUE;
	}
}

static void
osl_dma_unlock(osl_t *osh)
{
	if (unlikely(osh->dma_lock_bh)) {
		osh->dma_lock_bh = FALSE;
		spin_unlock_bh(&osh->dma_lock);
	} else {
		spin_unlock(&osh->dma_lock);
	}
}

static void
osl_dma_lock_init(osl_t *osh)
{
	spin_lock_init(&osh->dma_lock);
	osh->dma_lock_bh = FALSE;
}
#endif /* USE_DMA_LOCK */
