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

static irqreturn_t t194_smmu_context_fault_inst(int irq,
						struct arm_smmu_device *smmu,
						int idx, int inst);

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

static void t194_smmu_tlb_sync_wait(struct arm_smmu_device *smmu, int page,
				    int sync, int status, int inst)
{
	u32 reg;
	unsigned int spin_cnt, delay;

	for (delay = 1; delay < TLB_LOOP_TIMEOUT; delay *= 2) {
		for (spin_cnt = TLB_SPIN_COUNT; spin_cnt > 0; spin_cnt--) {
			reg = readl_relaxed(
			      t194_page(inst, page) + status);
			if (!(reg & sTLBGSTATUS_GSACTIVE))
				return;
			cpu_relax();
		}
		udelay(delay);
	}
	dev_err_ratelimited(smmu->dev,
			    "TLB sync timed out -- SMMU may be deadlocked\n");
}

static void t194_smmu_tlb_sync(struct arm_smmu_device *smmu, int page,
			       int sync, int status)
{
	int i;

	arm_smmu_writel(smmu, page, sync, 0);

	for (i = 0; i < t194_smmu.num_inst; i++)
		t194_smmu_tlb_sync_wait(smmu, page, sync, status, i);
}

static irqreturn_t t194_smmu_global_fault_inst(int irq,
					       struct arm_smmu_device *smmu,
					       int inst)
{
	u32 gfsr, gfsynr0, gfsynr1, gfsynr2;

	gfsr = readl_relaxed(t194_page(inst, 0) + ARM_SMMU_GR0_sGFSR);
	gfsynr0 = readl_relaxed(t194_page(inst, 0) + ARM_SMMU_GR0_sGFSYNR0);
	gfsynr1 = readl_relaxed(t194_page(inst, 0) + ARM_SMMU_GR0_sGFSYNR1);
	gfsynr2 = readl_relaxed(t194_page(inst, 0) + ARM_SMMU_GR0_sGFSYNR2);

	if (!gfsr)
		return IRQ_NONE;

	dev_err_ratelimited(smmu->dev,
		"Unexpected global fault, this could be serious\n");
	dev_err_ratelimited(smmu->dev,
		"\tGFSR 0x%08x, GFSYNR0 0x%08x, GFSYNR1 0x%08x, GFSYNR2 0x%08x\n",
		gfsr, gfsynr0, gfsynr1, gfsynr2);

	writel_relaxed(gfsr, t194_page(inst, 0) + ARM_SMMU_GR0_sGFSR);
	return IRQ_HANDLED;
}

static irqreturn_t t194_smmu_global_fault(int irq, struct arm_smmu_device *smmu)
{
	int i;
	irqreturn_t irq_ret = IRQ_NONE;

	/* Interrupt line is shared between global and context faults.
	 * Check for both type of interrupts on either fault handlers.
	 */
	for (i = 0; i < t194_smmu.num_inst; i++) {
		irq_ret = t194_smmu_context_fault_inst(irq, smmu, 0, i);
		if (irq_ret == IRQ_HANDLED)
			return irq_ret;
	}

	for (i = 0; i < t194_smmu.num_inst; i++) {
		irq_ret = t194_smmu_global_fault_inst(irq, smmu, i);
		if (irq_ret == IRQ_HANDLED)
			return irq_ret;
	}

	return irq_ret;
}

static irqreturn_t t194_smmu_context_fault_bank(int irq,
						struct arm_smmu_device *smmu,
						int idx, int inst)
{
	u32 fsr, fsynr, cbfrsynra;
	unsigned long iova;

	fsr = arm_smmu_read_cb(smmu, idx, ARM_SMMU_CB_FSR);
	if (!(fsr & FSR_FAULT))
		return IRQ_NONE;

	fsynr = readl_relaxed(t194_page(inst, smmu->cb_base + idx) +
			      ARM_SMMU_CB_FSYNR0);
	iova = readq_relaxed(t194_page(inst, smmu->cb_base + idx) +
			     ARM_SMMU_CB_FAR);
	cbfrsynra = readl_relaxed(t194_page(inst, 1) +
				  ARM_SMMU_GR1_CBFRSYNRA(idx));

	dev_err_ratelimited(smmu->dev,
	"Unhandled context fault: fsr=0x%x, iova=0x%08lx, fsynr=0x%x, cbfrsynra=0x%x, cb=%d\n",
			    fsr, iova, fsynr, cbfrsynra, idx);

	writel_relaxed(fsr, t194_page(inst, smmu->cb_base + idx) +
			    ARM_SMMU_CB_FSR);
	return IRQ_HANDLED;
}

static irqreturn_t t194_smmu_context_fault_inst(int irq,
						struct arm_smmu_device *smmu,
						int idx, int inst)
{
	irqreturn_t irq_ret = IRQ_NONE;

	/* Interrupt line shared between global and all context faults.
	 * Check for faults across all contexts.
	 */
	for (idx = 0; idx < smmu->num_context_banks; idx++) {
		irq_ret = t194_smmu_context_fault_bank(irq, smmu, idx, inst);

		if (irq_ret == IRQ_HANDLED)
			break;
	}

	return irq_ret;
}

static irqreturn_t t194_smmu_context_fault(int irq,
					   struct arm_smmu_device *smmu,
					   int cbndx)
{
	int i;
	irqreturn_t irq_ret = IRQ_NONE;

	/* Interrupt line is shared between global and context faults.
	 * Check for both type of interrupts on either fault handlers.
	 */
	for (i = 0; i < t194_smmu.num_inst; i++) {
		irq_ret = t194_smmu_global_fault_inst(irq, smmu, i);
		if (irq_ret == IRQ_HANDLED)
			return irq_ret;
	}

	for (i = 0; i < t194_smmu.num_inst; i++) {
		irq_ret = t194_smmu_context_fault_inst(irq, smmu, cbndx, i);
		if (irq_ret == IRQ_HANDLED)
			return irq_ret;
	}

	return irq_ret;
}

static const struct arm_smmu_impl nvidia_impl = {
	.read_reg = t194_smmu_read_reg,
	.write_reg = t194_smmu_write_reg,
	.read_reg64 = t194_smmu_read_reg64,
	.write_reg64 = t194_smmu_write_reg64,
	.tlb_sync = t194_smmu_tlb_sync,
	.global_fault = t194_smmu_global_fault,
	.context_fault = t194_smmu_context_fault,
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
