// SPDX-License-Identifier: GPL-2.0-only
// Miscellaneous Arm SMMU implementation and integration quirks
// Copyright (C) 2019 Arm Limited

#define pr_fmt(fmt) "arm-smmu: " fmt

#include <linux/bitfield.h>
#include <linux/of.h>

#include "arm-smmu.h"


static int arm_smmu_gr0_ns(int offset)
{
	switch(offset) {
	case ARM_SMMU_GR0_sCR0:
	case ARM_SMMU_GR0_sACR:
	case ARM_SMMU_GR0_sGFSR:
	case ARM_SMMU_GR0_sGFSYNR0:
	case ARM_SMMU_GR0_sGFSYNR1:
	case ARM_SMMU_GR0_sGFSYNR2:
		return offset + 0x400;
	default:
		return offset;
	}
}

static u32 arm_smmu_read_ns(struct arm_smmu_device *smmu, int page,
			    int offset)
{
	if (page == 0)
		offset = arm_smmu_gr0_ns(offset);
	return readl_relaxed(arm_smmu_page(smmu, page) + offset);
}

static void arm_smmu_write_ns(struct arm_smmu_device *smmu, int page,
			      int offset, u32 val)
{
	if (page == 0)
		offset = arm_smmu_gr0_ns(offset);
	writel_relaxed(val, arm_smmu_page(smmu, page) + offset);
}

/* Since we don't care for sGFAR, we can do without 64-bit accessors */
const struct arm_smmu_impl calxeda_impl = {
	.read_reg = arm_smmu_read_ns,
	.write_reg = arm_smmu_write_ns,
};


struct cavium_smmu {
	struct arm_smmu_device smmu;
	u32 id_base;
};
#define to_csmmu(s)	container_of(s, struct cavium_smmu, smmu)

static int cavium_cfg_probe(struct arm_smmu_device *smmu)
{
	static atomic_t context_count = ATOMIC_INIT(0);
	/*
	 * Cavium CN88xx erratum #27704.
	 * Ensure ASID and VMID allocation is unique across all SMMUs in
	 * the system.
	 */
	to_csmmu(smmu)->id_base = atomic_fetch_add(smmu->num_context_banks,
						   &context_count);
	dev_notice(smmu->dev, "\tenabling workaround for Cavium erratum 27704\n");

	return 0;
}

int cavium_init_context(struct arm_smmu_domain *smmu_domain)
{
	u32 id_base = to_csmmu(smmu_domain->smmu)->id_base;

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_S2)
		smmu_domain->cfg.vmid += id_base;
	else
		smmu_domain->cfg.asid += id_base;

	return 0;
}

const struct arm_smmu_impl cavium_impl = {
	.cfg_probe = cavium_cfg_probe,
	.init_context = cavium_init_context,
};

struct arm_smmu_device *cavium_smmu_impl_init(struct arm_smmu_device *smmu)
{
	struct cavium_smmu *csmmu;

	csmmu = devm_kzalloc(smmu->dev, sizeof(*csmmu), GFP_KERNEL);
	if (!csmmu)
		return ERR_PTR(-ENOMEM);

	csmmu->smmu = *smmu;
	csmmu->smmu.impl = &cavium_impl;

	devm_kfree(smmu->dev, smmu);

	return &csmmu->smmu;
}


#define ARM_MMU500_ACTLR_CPRE		(1 << 1)

#define ARM_MMU500_ACR_CACHE_LOCK	(1 << 26)
#define ARM_MMU500_ACR_S2CRB_TLBEN	(1 << 10)
#define ARM_MMU500_ACR_SMTNMB_TLBEN	(1 << 8)

static int arm_mmu500_reset(struct arm_smmu_device *smmu)
{
	u32 reg, major;
	int i;
	/*
	 * On MMU-500 r2p0 onwards we need to clear ACR.CACHE_LOCK before
	 * writes to the context bank ACTLRs will stick. And we just hope that
	 * Secure has also cleared SACR.CACHE_LOCK for this to take effect...
	 */
	reg = arm_smmu_gr0_read(smmu, ARM_SMMU_GR0_ID7);
	major = FIELD_GET(ID7_MAJOR, reg);
	reg = arm_smmu_gr0_read(smmu, ARM_SMMU_GR0_sACR);
	if (major >= 2)
		reg &= ~ARM_MMU500_ACR_CACHE_LOCK;
	/*
	 * Allow unmatched Stream IDs to allocate bypass
	 * TLB entries for reduced latency.
	 */
	reg |= ARM_MMU500_ACR_SMTNMB_TLBEN | ARM_MMU500_ACR_S2CRB_TLBEN;
	arm_smmu_gr0_write(smmu, ARM_SMMU_GR0_sACR, reg);

	/*
	 * Disable MMU-500's not-particularly-beneficial next-page
	 * prefetcher for the sake of errata #841119 and #826419.
	 */
	for (i = 0; i < smmu->num_context_banks; ++i) {
		reg = arm_smmu_cb_read(smmu, i, ARM_SMMU_CB_ACTLR);
		reg &= ~ARM_MMU500_ACTLR_CPRE;
		arm_smmu_cb_write(smmu, i, ARM_SMMU_CB_ACTLR, reg);
	}

	return 0;
}

const struct arm_smmu_impl arm_mmu500_impl = {
	.reset = arm_mmu500_reset,
};


struct arm_smmu_device *arm_smmu_impl_init(struct arm_smmu_device *smmu)
{
	/* The current quirks happen to be mutually-exclusive */
	if (of_property_read_bool(smmu->dev->of_node,
				  "calxeda,smmu-secure-config-access"))
		smmu->impl = &calxeda_impl;

	if (smmu->model == CAVIUM_SMMUV2)
		return cavium_smmu_impl_init(smmu);

	if (smmu->model == ARM_MMU500)
		smmu->impl = &arm_mmu500_impl;

	if (smmu->model == NVIDIA_SMMUV2)
		return nvidia_smmu_impl_init(smmu);

	return smmu;
}
