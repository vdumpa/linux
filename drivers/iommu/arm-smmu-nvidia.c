// SPDX-License-Identifier: GPL-2.0-only
// Tegra194 ARM SMMU implementation quirks
// Copyright (C) 2019 NVIDIA CORPORATION.  All rights reserved.

#define pr_fmt(fmt) "nvidia-smmu: " fmt

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "arm-smmu.h"

#define NUM_SMMU_INSTANCES 3

struct nvidia_smmu {
	struct arm_smmu_device	smmu;
	int			num_inst;
	void __iomem		*bases[NUM_SMMU_INSTANCES];
};

#define to_nsmmu(s)	container_of(s, struct nvidia_smmu, smmu)

#define nsmmu_page(smmu, inst, page) \
	(((inst) ? to_nsmmu(smmu)->bases[(inst)] : smmu->base) + \
	((page) << smmu->pgshift))

static u32 nsmmu_read_reg(struct arm_smmu_device *smmu,
			      int page, int offset)
{
	return readl_relaxed(nsmmu_page(smmu, 0, page) + offset);
}

static void nsmmu_write_reg(struct arm_smmu_device *smmu,
				int page, int offset, u32 val)
{
	int i;

	for (i = 0; i < to_nsmmu(smmu)->num_inst; i++)
		writel_relaxed(val, nsmmu_page(smmu, i, page) + offset);
}

static u64 nsmmu_read_reg64(struct arm_smmu_device *smmu,
				int page, int offset)
{
	return readq_relaxed(nsmmu_page(smmu, 0, page) + offset);
}

static void nsmmu_write_reg64(struct arm_smmu_device *smmu,
				  int page, int offset, u64 val)
{
	int i;

	for (i = 0; i < to_nsmmu(smmu)->num_inst; i++)
		writeq_relaxed(val, nsmmu_page(smmu, i, page) + offset);
}

static void nsmmu_tlb_sync_wait(struct arm_smmu_device *smmu, int page,
				int sync, int status, int inst)
{
	u32 reg;
	unsigned int spin_cnt, delay;

	for (delay = 1; delay < TLB_LOOP_TIMEOUT; delay *= 2) {
		for (spin_cnt = TLB_SPIN_COUNT; spin_cnt > 0; spin_cnt--) {
			reg = readl_relaxed(
			      nsmmu_page(smmu, inst, page) + status);
			if (!(reg & sTLBGSTATUS_GSACTIVE))
				return;
			cpu_relax();
		}
		udelay(delay);
	}
	dev_err_ratelimited(smmu->dev,
			    "TLB sync timed out -- SMMU may be deadlocked\n");
}

static void nsmmu_tlb_sync(struct arm_smmu_device *smmu, int page,
			   int sync, int status)
{
	int i;

	arm_smmu_writel(smmu, page, sync, 0);

	for (i = 0; i < to_nsmmu(smmu)->num_inst; i++)
		nsmmu_tlb_sync_wait(smmu, page, sync, status, i);
}

static const struct arm_smmu_impl nsmmu_impl = {
	.read_reg = nsmmu_read_reg,
	.write_reg = nsmmu_write_reg,
	.read_reg64 = nsmmu_read_reg64,
	.write_reg64 = nsmmu_write_reg64,
	.tlb_sync = nsmmu_tlb_sync,
};

struct arm_smmu_device *nvidia_smmu_impl_init(struct arm_smmu_device *smmu)
{
	int i;
	struct nvidia_smmu *nsmmu;
	struct resource *res;
	struct device *dev = smmu->dev;
	struct platform_device *pdev = to_platform_device(smmu->dev);

	nsmmu = devm_kzalloc(smmu->dev, sizeof(*nsmmu), GFP_KERNEL);
	if (!nsmmu)
		return ERR_PTR(-ENOMEM);

	nsmmu->smmu = *smmu;
	/* Instance 0 is ioremapped by arm-smmu.c */
	nsmmu->num_inst = 1;

	for (i = 1; i < NUM_SMMU_INSTANCES; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;
		nsmmu->bases[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(nsmmu->bases[i]))
			return (struct arm_smmu_device *)nsmmu->bases[i];
		nsmmu->num_inst++;
	}

	nsmmu->smmu.impl = &nsmmu_impl;
	devm_kfree(smmu->dev, smmu);
	pr_info("Nvidia SMMUv2, Instances=%d\n", nsmmu->num_inst);

	return &nsmmu->smmu;
}
