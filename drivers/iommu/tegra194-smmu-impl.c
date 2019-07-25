// SPDX-License-Identifier: GPL-2.0-only
// Tegra194 ARM SMMU implementation quirks
// Copyright (C) 2019 NVIDIA CORPORATION.  All rights reserved.

#define pr_fmt(fmt) "tegra194-smmu: " fmt

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "arm-smmu.h"

#define NUM_SMMU_INSTANCES 3

struct t194_smmu_device {
	int				num_inst;
	void __iomem			*bases[NUM_SMMU_INSTANCES];
};

static struct t194_smmu_device t194_smmu = {
	.num_inst = 1,
};

#define t194_page(inst, page) \
	(((inst) ? t194_smmu.bases[(inst)] : smmu->base) + \
	((page) << smmu->pgshift))

static u32 t194_smmu_read_reg(struct arm_smmu_device *smmu,
			      int page, int offset)
{
	return readl_relaxed(t194_page(0, page) + offset);
}

static void t194_smmu_write_reg(struct arm_smmu_device *smmu,
				int page, int offset, u32 val)
{
	int i;

	for (i = 0; i < t194_smmu.num_inst; i++)
		writel_relaxed(val, t194_page(i, page) + offset);
}

static u64 t194_smmu_read_reg64(struct arm_smmu_device *smmu,
				int page, int offset)
{
	return readq_relaxed(t194_page(0, page) + offset);
}

static void t194_smmu_write_reg64(struct arm_smmu_device *smmu,
				  int page, int offset, u64 val)
{
	int i;

	for (i = 0; i < t194_smmu.num_inst; i++)
		writeq_relaxed(val, t194_page(i, page) + offset);
}

static const struct arm_smmu_impl nvidia_impl = {
	.read_reg = t194_smmu_read_reg,
	.write_reg = t194_smmu_write_reg,
	.read_reg64 = t194_smmu_read_reg64,
	.write_reg64 = t194_smmu_write_reg64,
};

int t194_smmu_impl_init(struct platform_device *pdev,
			struct arm_smmu_device *smmu)
{
	int i;
	struct resource *res;
	struct device *dev = &pdev->dev;

	for (i = 1; i < NUM_SMMU_INSTANCES; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;
		t194_smmu.bases[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(t194_smmu.bases[i]))
			return PTR_ERR(t194_smmu.bases[i]);
		t194_smmu.num_inst++;
	}

	smmu->impl = &nvidia_impl;
	pr_info("Tegra194 SMMU, instances=%d\n", t194_smmu.num_inst);

	return 0;
}
