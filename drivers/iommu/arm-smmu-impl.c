// SPDX-License-Identifier: GPL-2.0-only
// Miscellaneous Arm SMMU implementation quirks
// Copyright (C) 2019 Arm Limited

#define pr_fmt(fmt) "arm-smmu: " fmt

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

static u64 arm_smmu_read_ns(struct arm_smmu_device *smmu, int page,
			    int offset, bool q)
{
	if (page == 0)
		offset = arm_smmu_gr0_ns(offset);
	if (q)
		return readq_relaxed(arm_smmu_page(smmu, page) + offset);
	else
		return readl_relaxed(arm_smmu_page(smmu, page) + offset);
}

static void arm_smmu_write_ns(struct arm_smmu_device *smmu, int page,
			      int offset, u64 val, bool q)
{
	if (page == 0)
		offset = arm_smmu_gr0_ns(offset);
	if (q)
		writeq_relaxed(val, arm_smmu_page(smmu, page) + offset);
	else
		writel_relaxed(val, arm_smmu_page(smmu, page) + offset);
}

const struct arm_smmu_impl calxeda_impl = {
	.read_reg = arm_smmu_read_ns,
	.write_reg = arm_smmu_write_ns,
};


static atomic_t cavium_smmu_context_count = ATOMIC_INIT(0);

static int cavium_cfg_probe(struct arm_smmu_device *smmu)
{
	/*
	 * Cavium CN88xx erratum #27704.
	 * Ensure ASID and VMID allocation is unique across all SMMUs in
	 * the system.
	 */
	smmu->cavium_id_base = atomic_fetch_add(smmu->num_context_banks,
						&cavium_smmu_context_count);
	dev_notice(smmu->dev, "\tenabling workaround for Cavium erratum 27704\n");

	return 0;
}

const struct arm_smmu_impl cavium_impl = {
	.cfg_probe = cavium_cfg_probe,
};


int arm_smmu_impl_init(struct arm_smmu_device *smmu)
{
	/* The current quirks happen to be mutually-exclusive */
	if (of_property_read_bool(smmu->dev->of_node,
				  "calxeda,smmu-secure-config-access"))
		smmu->impl = &calxeda_impl;

	if (smmu->model == CAVIUM_SMMUV2)
		smmu->impl = &cavium_impl;

	return 0;
}
