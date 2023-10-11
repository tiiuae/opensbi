/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_barrier.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_fp.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_csr_detect.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_math.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_pmu.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_hfence.h>

extern void __sbi_expected_trap(void);
extern void __sbi_expected_trap_hext(void);

void (*sbi_hart_expected_trap)(void) = &__sbi_expected_trap;

static unsigned long hart_features_offset;

static void mstatus_init(struct sbi_scratch *scratch)
{
	unsigned long mstatus_val = 0;

	/* Enable FPU */
	if (misa_extension('D') || misa_extension('F'))
		mstatus_val |=  MSTATUS_FS;

	csr_write(CSR_MSTATUS, mstatus_val);

	/* Disable all interrupts */
	csr_write(CSR_MIE, 0);

	/* Disable S-mode paging */
	if (misa_extension('S'))
		csr_write(CSR_SATP, 0);
}

static int fp_init(struct sbi_scratch *scratch)
{
#ifdef __riscv_flen
	int i;
#endif

	if (!misa_extension('D') && !misa_extension('F'))
		return 0;

	if (!(csr_read(CSR_MSTATUS) & MSTATUS_FS))
		return SBI_EINVAL;

#ifdef __riscv_flen
	for (i = 0; i < 32; i++)
		init_fp_reg(i);
	csr_write(CSR_FCSR, 0);
#endif

	return 0;
}

static int delegate_traps(struct sbi_scratch *scratch)
{
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);
	unsigned long interrupts, exceptions;

	if (!misa_extension('S'))
		/* No delegation possible as mideleg does not exist */
		return 0;

	/* Send M-mode interrupts and most exceptions to S-mode */
	interrupts = MIP_SSIP | MIP_STIP | MIP_SEIP;
	//interrupts |= sbi_pmu_irq_bit();

	exceptions = (1U << CAUSE_MISALIGNED_FETCH) | (1U << CAUSE_BREAKPOINT) |
		     (1U << CAUSE_USER_ECALL);
	if (sbi_platform_has_mfaults_delegation(plat))
		exceptions |= (1U << CAUSE_FETCH_PAGE_FAULT) |
			      (1U << CAUSE_LOAD_PAGE_FAULT) |
			      (1U << CAUSE_STORE_PAGE_FAULT);

	/*
	 * If hypervisor extension available then we only handle hypervisor
	 * calls (i.e. ecalls from HS-mode) in M-mode.
	 *
	 * The HS-mode will additionally handle supervisor calls (i.e. ecalls
	 * from VS-mode), Guest page faults and Virtual interrupts.
	 */
	if (misa_extension('H')) {
		exceptions |= (1U << CAUSE_VIRTUAL_SUPERVISOR_ECALL);
		exceptions |= (1U << CAUSE_FETCH_GUEST_PAGE_FAULT);
		exceptions |= (1U << CAUSE_LOAD_GUEST_PAGE_FAULT);
		exceptions |= (1U << CAUSE_VIRTUAL_INST_FAULT);
		exceptions |= (1U << CAUSE_STORE_GUEST_PAGE_FAULT);
	}

	csr_write(CSR_MIDELEG, interrupts);
	csr_write(CSR_MEDELEG, exceptions);

	return 0;
}

unsigned int sbi_hart_mhpm_count(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->mhpm_count;
}

unsigned int sbi_hart_pmp_count(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_count;
}

unsigned long sbi_hart_pmp_granularity(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_gran;
}

unsigned int sbi_hart_pmp_addrbits(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->pmp_addr_bits;
}

unsigned int sbi_hart_mhpm_bits(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->mhpm_bits;
}

int sbi_hart_pmp_configure(struct sbi_scratch *scratch)
{
	struct sbi_domain_memregion *reg;
	struct sbi_domain *dom = sbi_domain_thishart_ptr();
	unsigned int pmp_idx = 0, pmp_flags, pmp_bits, pmp_gran_log2;
	unsigned int pmp_count = sbi_hart_pmp_count(scratch);
	unsigned long pmp_addr = 0, pmp_addr_max = 0;

	if (!pmp_count)
		return 0;

	pmp_gran_log2 = log2roundup(sbi_hart_pmp_granularity(scratch));
	pmp_bits = sbi_hart_pmp_addrbits(scratch) - 1;
	pmp_addr_max = (1UL << pmp_bits) | ((1UL << pmp_bits) - 1);

	sbi_domain_for_each_memregion(dom, reg) {
		if (pmp_count <= pmp_idx)
			break;

		pmp_flags = 0;

		/*
		 * If permissions are to be enforced for all modes on this
		 * region, the lock bit should be set.
		 */
		if (reg->flags & SBI_DOMAIN_MEMREGION_ENF_PERMISSIONS)
			pmp_flags |= PMP_L;

		if (reg->flags & SBI_DOMAIN_MEMREGION_SU_READABLE)
			pmp_flags |= PMP_R;
		if (reg->flags & SBI_DOMAIN_MEMREGION_SU_WRITABLE)
			pmp_flags |= PMP_W;
		if (reg->flags & SBI_DOMAIN_MEMREGION_SU_EXECUTABLE)
			pmp_flags |= PMP_X;

		pmp_addr =  reg->base >> PMP_SHIFT;
		if (pmp_gran_log2 <= reg->order && pmp_addr < pmp_addr_max)
			pmp_set(pmp_idx++, pmp_flags, reg->base, reg->order);
		else {
			sbi_printf("Can not configure pmp for domain %s", dom->name);
			sbi_printf(" because memory region address %lx or size %lx is not in range\n",
				    reg->base, reg->order);
		}
	}

	/*
	 * As per section 3.7.2 of privileged specification v1.12,
	 * virtual address translations can be speculatively performed
	 * (even before actual access). These, along with PMP traslations,
	 * can be cached. This can pose a problem with CPU hotplug
	 * and non-retentive suspend scenario because PMP states are
	 * not preserved.
	 * It is advisable to flush the caching structures under such
	 * conditions.
	 */
	if (misa_extension('S')) {
		__asm__ __volatile__("sfence.vma");

		/*
		 * If hypervisor mode is supported, flush caching
		 * structures in guest mode too.
		 */
		if (misa_extension('H'))
			__sbi_hfence_gvma_all();
	}

	return 0;
}

int sbi_hart_priv_version(struct sbi_scratch *scratch)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	return hfeatures->priv_version;
}

void sbi_hart_get_priv_version_str(struct sbi_scratch *scratch,
				   char *version_str, int nvstr)
{
	char *temp;
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	switch (hfeatures->priv_version) {
	case SBI_HART_PRIV_VER_1_10:
		temp = "v1.10";
		break;
	case SBI_HART_PRIV_VER_1_11:
		temp = "v1.11";
		break;
	case SBI_HART_PRIV_VER_1_12:
		temp = "v1.12";
		break;
	default:
		temp = "unknown";
		break;
	}

	sbi_snprintf(version_str, nvstr, "%s", temp);
}

static inline void __sbi_hart_update_extension(
					struct sbi_hart_features *hfeatures,
					enum sbi_hart_extensions ext,
					bool enable)
{
	if (enable)
		hfeatures->extensions |= BIT(ext);
	else
		hfeatures->extensions &= ~BIT(ext);
}

/**
 * Enable/Disable a particular hart extension
 *
 * @param scratch pointer to the HART scratch space
 * @param ext the extension number to check
 * @param enable new state of hart extension
 */
void sbi_hart_update_extension(struct sbi_scratch *scratch,
			       enum sbi_hart_extensions ext,
			       bool enable)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	__sbi_hart_update_extension(hfeatures, ext, enable);
}

/**
 * Check whether a particular hart extension is available
 *
 * @param scratch pointer to the HART scratch space
 * @param ext the extension number to check
 * @returns true (available) or false (not available)
 */
bool sbi_hart_has_extension(struct sbi_scratch *scratch,
			    enum sbi_hart_extensions ext)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);

	if (hfeatures->extensions & BIT(ext))
		return true;
	else
		return false;
}

static inline char *sbi_hart_extension_id2string(int ext)
{
	char *estr = NULL;

	switch (ext) {
	case SBI_HART_EXT_SSCOFPMF:
		estr = "sscofpmf";
		break;
	case SBI_HART_EXT_TIME:
		estr = "time";
		break;
	case SBI_HART_EXT_SMAIA:
		estr = "smaia";
		break;
	case SBI_HART_EXT_SSTC:
		estr = "sstc";
		break;
	case SBI_HART_EXT_SMSTATEEN:
		estr = "smstateen";
		break;
	default:
		break;
	}

	return estr;
}

/**
 * Get the hart extensions in string format
 *
 * @param scratch pointer to the HART scratch space
 * @param extensions_str pointer to a char array where the extensions string
 *			 will be updated
 * @param nestr length of the features_str. The feature string will be
 *		truncated if nestr is not long enough.
 */
void sbi_hart_get_extensions_str(struct sbi_scratch *scratch,
				 char *extensions_str, int nestr)
{
	struct sbi_hart_features *hfeatures =
			sbi_scratch_offset_ptr(scratch, hart_features_offset);
	int offset = 0, ext = 0;
	char *temp;

	if (!extensions_str || nestr <= 0)
		return;
	sbi_memset(extensions_str, 0, nestr);

	if (!hfeatures->extensions)
		goto done;

	do {
		if (hfeatures->extensions & BIT(ext)) {
			temp = sbi_hart_extension_id2string(ext);
			if (temp) {
				sbi_snprintf(extensions_str + offset,
					     nestr - offset,
					     "%s,", temp);
				offset = offset + sbi_strlen(temp) + 1;
			}
		}

		ext++;
	} while (ext < SBI_HART_EXT_MAX);

done:
	if (offset)
		extensions_str[offset - 1] = '\0';
	else
		sbi_strncpy(extensions_str, "none", nestr);
}

static int hart_detect_features(struct sbi_scratch *scratch)
{
	struct sbi_trap_info trap = {0};
	struct sbi_hart_features *hfeatures =
		sbi_scratch_offset_ptr(scratch, hart_features_offset);
	int rc;

	/* If hart features already detected then do nothing */
	if (hfeatures->detected)
		return 0;

	/* Clear hart features */
	hfeatures->extensions = 0;
	hfeatures->pmp_count = 0;
	hfeatures->mhpm_count = 0;

	/* Detect if hart supports time CSR */
	csr_read_allowed(CSR_TIME, (unsigned long)&trap);
	if (!trap.cause)
		__sbi_hart_update_extension(hfeatures,
					SBI_HART_EXT_TIME, true);

	/* Let platform populate extensions */
	rc = sbi_platform_extensions_init(sbi_platform_thishart_ptr(),
					  hfeatures);
	if (rc)
		return rc;

	/* Mark hart feature detection done */
	hfeatures->detected = true;

	return 0;
}

int sbi_hart_reinit(struct sbi_scratch *scratch)
{
	int rc;

	mstatus_init(scratch);

	rc = fp_init(scratch);
	if (rc)
		return rc;

	rc = delegate_traps(scratch);
	if (rc)
		return rc;

	return 0;
}

int sbi_hart_init(struct sbi_scratch *scratch, bool cold_boot)
{
	int rc;

	/*
	 * Clear mip CSR before proceeding with init to avoid any spurious
	 * external interrupts in S-mode.
	 */
	csr_write(CSR_MIP, 0);

	if (cold_boot) {
		if (misa_extension('H'))
			sbi_hart_expected_trap = &__sbi_expected_trap_hext;

		hart_features_offset = sbi_scratch_alloc_offset(
					sizeof(struct sbi_hart_features));
		if (!hart_features_offset)
			return SBI_ENOMEM;
	}

	rc = hart_detect_features(scratch);
	if (rc)
		return rc;

	return sbi_hart_reinit(scratch);
}

void __attribute__((noreturn)) sbi_hart_hang(void)
{
	while (1)
		wfi();
	__builtin_unreachable();
}

void __attribute__((noreturn))
sbi_hart_switch_mode(unsigned long arg0, unsigned long arg1,
		     unsigned long next_addr, unsigned long next_mode,
		     bool next_virt)
{
#if __riscv_xlen == 32
	unsigned long val, valH;
#else
	unsigned long val;
#endif

	switch (next_mode) {
	case PRV_M:
		break;
	case PRV_S:
		if (!misa_extension('S'))
			sbi_hart_hang();
		break;
	case PRV_U:
		if (!misa_extension('U'))
			sbi_hart_hang();
		break;
	default:
		sbi_hart_hang();
	}

	val = csr_read(CSR_MSTATUS);
	val = INSERT_FIELD(val, MSTATUS_MPP, next_mode);
	val = INSERT_FIELD(val, MSTATUS_MPIE, 0);
#if __riscv_xlen == 32
	if (misa_extension('H')) {
		valH = csr_read(CSR_MSTATUSH);
		valH = INSERT_FIELD(valH, MSTATUSH_MPV, next_virt);
		csr_write(CSR_MSTATUSH, valH);
	}
#else
	if (misa_extension('H'))
		val = INSERT_FIELD(val, MSTATUS_MPV, next_virt);
#endif
	csr_write(CSR_MSTATUS, val);
	csr_write(CSR_MEPC, next_addr);

	if (next_mode == PRV_S) {
		csr_write(CSR_STVEC, next_addr);
		csr_write(CSR_SSCRATCH, 0);
		csr_write(CSR_SIE, 0);
		csr_write(CSR_SATP, 0);
	} else if (next_mode == PRV_U) {
		if (misa_extension('N')) {
			csr_write(CSR_UTVEC, next_addr);
			csr_write(CSR_USCRATCH, 0);
			csr_write(CSR_UIE, 0);
		}
	}

	register unsigned long a0 asm("a0") = arg0;
	register unsigned long a1 asm("a1") = arg1;
	__asm__ __volatile__("mret" : : "r"(a0), "r"(a1));
	__builtin_unreachable();
}
