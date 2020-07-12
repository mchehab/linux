// SPDX-License-Identifier: GPL-2.0
/*
 * hisi_smmu_lpae.c -- 3 layer pagetable
 *
 * Copyright (c) 2014 Huawei Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <asm/pgalloc.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include "hisi_smmu.h"

struct hisi_smmu_device_lpae *hisi_smmu_dev;

/* transfer 64bit pte table pointer to struct page */
static pgtable_t smmu_pgd_to_pte_lpae(unsigned int ppte_table)
{
	unsigned long page_table_addr;

	if (!ppte_table) {
		dbg("error: the pointer of pte_table is NULL\n");
		return NULL;
	}
	page_table_addr = (unsigned long)ppte_table;
	return phys_to_page(page_table_addr);
}

/* transfer 64bit pte table pointer to struct page */
static pgtable_t smmu_pmd_to_pte_lpae(unsigned long ppte_table)
{
	struct page *table = NULL;

	if (!ppte_table) {
		dbg("error: the pointer of pte_table is NULL\n");
		return NULL;
	}
	table = phys_to_page(ppte_table);
	return table;
}

static int get_domain_data_lpae(struct device_node *np,
				struct iommu_domain_data *data)
{
	unsigned long long align;
	struct device_node *node = NULL;
	int ret = 0;

	data->phy_pgd_base = hisi_smmu_dev->smmu_phy_pgtable_addr;

	if (!np)
		return 0;

	node = of_find_node_by_name(np, "iommu_info");
	if (!node) {
		dbg("find iommu_info node error\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(node, "start-addr",
				   &data->iova_start);
	if (ret) {
		dbg("read iova start address error\n");
		goto read_error;
	}
	ret = of_property_read_u32(node, "size", &data->iova_size);
	if (ret) {
		dbg("read iova size error\n");
		goto read_error;
	}
	ret = of_property_read_u64(node, "iova-align", &align);
	if (!ret)
		data->iova_align = (unsigned long)align;
	else
		data->iova_align = SZ_256K;

	pr_err("%s:start_addr 0x%x, size 0x%x align 0x%lx\n",
	       __func__, data->iova_start,
		data->iova_size, data->iova_align);

	return 0;

read_error:
	return ret;
}

static struct iommu_domain
*hisi_smmu_domain_alloc_lpae(unsigned int iommu_domain_type)
{
	struct iommu_domain *domain;

	if (iommu_domain_type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);

	return domain;
}

static void hisi_smmu_flush_pgtable_lpae(void *addr, size_t size)
{
	__flush_dcache_area(addr, size);
}

static void hisi_smmu_free_ptes_lpae(smmu_pgd_t pmd)
{
	pgtable_t table = smmu_pgd_to_pte_lpae(pmd);

	if (!table) {
		dbg("pte table is null\n");
		return;
	}
	__free_page(table);
	smmu_set_pmd_lpae(&pmd, 0);
}

static void hisi_smmu_free_pmds_lpae(smmu_pgd_t pgd)
{
	pgtable_t table = smmu_pmd_to_pte_lpae(pgd);

	if (!table) {
		dbg("pte table is null\n");
		return;
	}
	__free_page(table);
	smmu_set_pgd_lpae(&pgd, 0);
}

static void hisi_smmu_free_pgtables_lpae(unsigned long *page_table_addr)
{
	int i, j;
	smmu_pgd_t *pgd;
	smmu_pmd_t *pmd;
	unsigned long flags;

	pgd = (smmu_pgd_t *)page_table_addr;
	pmd = (smmu_pmd_t *)page_table_addr;

	spin_lock_irqsave(&hisi_smmu_dev->lock, flags);
	for (i = 0; i < SMMU_PTRS_PER_PGD; ++i) {
		if ((smmu_pgd_none_lpae(*pgd)) & (smmu_pmd_none_lpae(*pmd)))
			continue;
		for (j = 0; j < SMMU_PTRS_PER_PMD; ++j) {
			hisi_smmu_free_pmds_lpae(*pgd);
			pmd++;
		}
		hisi_smmu_free_ptes_lpae(*pmd);
		pgd++;
	}
	memset((void *)page_table_addr, 0, PAGE_SIZE);
	spin_unlock_irqrestore(&hisi_smmu_dev->lock, flags);
}

static void hisi_smmu_domain_free_lpae(struct iommu_domain *domain)
{
	if (list_empty(&hisi_smmu_dev->domain_list))
		hisi_smmu_free_pgtables_lpae((unsigned long *)hisi_smmu_dev->va_pgtable_addr);

	kfree(domain);
}

static int hisi_smmu_alloc_init_pte_lpae(smmu_pmd_t *ppmd,
					 unsigned long addr, unsigned long end,
		unsigned long pfn, u64 prot, unsigned long *flags)
{
	smmu_pte_t *pte, *start;
	pgtable_t table;
	u64 pteval = SMMU_PTE_TYPE;

	if (!smmu_pmd_none_lpae(*ppmd))
		goto pte_ready;

	/* Allocate a new set of tables */
	spin_unlock_irqrestore(&hisi_smmu_dev->lock, *flags);
	table = alloc_page(GFP_KERNEL | __GFP_ZERO | __GFP_DMA);
	spin_lock_irqsave(&hisi_smmu_dev->lock, *flags);
	if (!table) {
		dbg("%s: alloc page fail\n", __func__);
		return -ENOMEM;
	}

	if (smmu_pmd_none_lpae(*ppmd)) {
		hisi_smmu_flush_pgtable_lpae(page_address(table),
					     SMMU_PAGE_SIZE);
		smmu_pmd_populate_lpae(ppmd, table, SMMU_PMD_TYPE | SMMU_PMD_NS);
		hisi_smmu_flush_pgtable_lpae(ppmd, sizeof(*ppmd));
	} else {
		__free_page(table);
	}

pte_ready:
	start = (smmu_pte_t *)smmu_pte_page_vaddr_lpae(ppmd)
		+ smmu_pte_index(addr);
	pte = start;
	if (!prot) {
		pteval |= SMMU_PROT_NORMAL;
		pteval |= SMMU_PTE_NS;
	} else {
		if (prot & IOMMU_CACHE)
			pteval |= SMMU_PROT_NORMAL_CACHE;
		else
			pteval |= SMMU_PROT_NORMAL_NC;

		if ((prot & IOMMU_READ) && (prot & IOMMU_WRITE))
			pteval |= SMMU_PAGE_READWRITE;
		else if ((prot & IOMMU_READ) && !(prot & IOMMU_WRITE))
			pteval |= SMMU_PAGE_READONLY;
		else
			WARN_ON("you do not set read attribute!");

		if (!(prot & IOMMU_NOEXEC)) {
			pteval |= SMMU_PAGE_READONLY_EXEC;
			pteval &= ~(SMMU_PTE_PXN | SMMU_PTE_UXN);
		}

		pteval |= SMMU_PTE_NS;
	}

	do {
		if (!pte_is_valid_lpae(pte))
			*pte = (u64)(__pfn_to_phys(pfn) | pteval);
		else
			WARN_ONCE(1, "map to same VA more times!\n");
		pte++;
		pfn++;
		addr += SMMU_PAGE_SIZE;
	} while (addr < end);

	hisi_smmu_flush_pgtable_lpae(start, sizeof(*pte) * (pte - start));
	return 0;
}

static int hisi_smmu_alloc_init_pmd_lpae(smmu_pgd_t *ppgd,
					 unsigned long addr, unsigned long end,
					 unsigned long paddr, int prot,
					 unsigned long *flags)
{
	int ret = 0;
	smmu_pmd_t *ppmd, *start;
	u64 next;
	pgtable_t table;

	if (!smmu_pgd_none_lpae(*ppgd))
		goto pmd_ready;

	/* Allocate a new set of tables */
	spin_unlock_irqrestore(&hisi_smmu_dev->lock, *flags);
	table = alloc_page(GFP_KERNEL | __GFP_ZERO | __GFP_DMA);
	spin_lock_irqsave(&hisi_smmu_dev->lock, *flags);
	if (!table) {
		dbg("%s: alloc page fail\n", __func__);
		return -ENOMEM;
	}

	if (smmu_pgd_none_lpae(*ppgd)) {
		hisi_smmu_flush_pgtable_lpae(page_address(table),
					     SMMU_PAGE_SIZE);
		smmu_pgd_populate_lpae(ppgd, table, SMMU_PGD_TYPE | SMMU_PGD_NS);
		hisi_smmu_flush_pgtable_lpae(ppgd, sizeof(*ppgd));
	} else {
		__free_page(table);
	}

pmd_ready:
	start = (smmu_pmd_t *)smmu_pmd_page_vaddr_lpae(ppgd)
		+ smmu_pmd_index(addr);
	ppmd = start;

	do {
		next = smmu_pmd_addr_end_lpae(addr, end);
		ret = hisi_smmu_alloc_init_pte_lpae(ppmd, addr, next,
						    __phys_to_pfn(paddr),
						    prot, flags);
		if (ret)
			goto error;
		paddr += (next - addr);
		addr = next;
	} while (ppmd++, addr < end);
error:
	return ret;
}

int hisi_smmu_handle_mapping_lpae(struct iommu_domain *domain,
				  unsigned long iova, phys_addr_t paddr,
				  size_t size, int prot)
{
	int ret;
	unsigned long end;
	unsigned long next;
	unsigned long flags;
	smmu_pgd_t *pgd = (smmu_pgd_t *)hisi_smmu_dev->va_pgtable_addr;

	if (!pgd) {
		dbg("pgd is null\n");
		return -EINVAL;
	}
	iova = ALIGN(iova, SMMU_PAGE_SIZE);
	size = ALIGN(size, SMMU_PAGE_SIZE);
	spin_lock_irqsave(&hisi_smmu_dev->lock, flags);
	pgd += smmu_pgd_index(iova);
	end = iova + size;
	do {
		next = smmu_pgd_addr_end_lpae(iova, end);
		ret = hisi_smmu_alloc_init_pmd_lpae(pgd,
						    iova, next, paddr, prot, &flags);
		if (ret)
			goto out_unlock;
		paddr += next - iova;
		iova = next;
	} while (pgd++, iova < end);
out_unlock:
	spin_unlock_irqrestore(&hisi_smmu_dev->lock, flags);
	return ret;
}

static int hisi_smmu_map_lpae(struct iommu_domain *domain,
			      unsigned long iova,
			      phys_addr_t paddr, size_t size,
			      int prot,
			      gfp_t gfp)
{
	unsigned long max_iova;
	struct iommu_domain_data *data;

	if (!domain) {
		dbg("domain is null\n");
		return -ENODEV;
	}
	data = domain->priv;
	max_iova = data->iova_start + data->iova_size;
	if (iova < data->iova_start) {
		dbg("iova failed: iova = 0x%lx, start = 0x%8x\n",
		    iova, data->iova_start);
		goto error;
	}
	if ((iova + size) > max_iova) {
		dbg("iova out of domain range, iova+size=0x%lx, end=0x%lx\n",
		    iova + size, max_iova);
		goto error;
	}
	return hisi_smmu_handle_mapping_lpae(domain, iova, paddr, size, prot);
error:
	dbg("iova is not in this range\n");
	return -EINVAL;
}

static unsigned int hisi_smmu_clear_pte_lpae(smmu_pgd_t *pmdp,
					     unsigned int iova, unsigned int end)
{
	smmu_pte_t *ptep = NULL;
	smmu_pte_t *ppte = NULL;
	unsigned int size = end - iova;

	ptep = smmu_pte_page_vaddr_lpae(pmdp);
	ppte = ptep + smmu_pte_index(iova);

	if (!!size)
		memset(ppte, 0x0, (size / SMMU_PAGE_SIZE) * sizeof(*ppte));

	return size;
}

static unsigned int hisi_smmu_clear_pmd_lpae(smmu_pgd_t *pgdp,
					     unsigned int iova, unsigned int end)
{
	smmu_pmd_t *pmdp = NULL;
	smmu_pmd_t *ppmd = NULL;
	unsigned int next = 0;
	unsigned int size = end - iova;

	pmdp = smmu_pmd_page_vaddr_lpae(pgdp);
	ppmd = pmdp + smmu_pmd_index(iova);
	do {
		next = smmu_pmd_addr_end_lpae(iova, end);
		hisi_smmu_clear_pte_lpae(ppmd, iova, next);
		iova = next;
		dbg("%s: iova=0x%lx, end=0x%lx\n", __func__, iova, end);
	} while (ppmd++, iova < end);

	return size;
}

unsigned int hisi_smmu_handle_unmapping_lpae(struct iommu_domain *domain,
					     unsigned long iova, size_t size)
{
	smmu_pgd_t *pgdp = NULL;
	unsigned int end = 0;
	unsigned int next = 0;
	unsigned int unmap_size = 0;
	unsigned long flags;

	iova = SMMU_PAGE_ALIGN(iova);
	size = SMMU_PAGE_ALIGN(size);
	pgdp = (smmu_pgd_t *)hisi_smmu_dev->va_pgtable_addr;
	end = iova + size;
	dbg("%s:end=0x%x\n", __func__, end);
	pgdp += smmu_pgd_index(iova);
	spin_lock_irqsave(&hisi_smmu_dev->lock, flags);
	do {
		next = smmu_pgd_addr_end_lpae(iova, end);
		unmap_size += hisi_smmu_clear_pmd_lpae(pgdp, iova, next);
		iova = next;
		dbg("%s: pgdp=%p, iova=0x%lx\n", __func__, pgdp, iova);
	} while (pgdp++, iova < end);

	spin_unlock_irqrestore(&hisi_smmu_dev->lock, flags);
	return unmap_size;
}

static size_t hisi_smmu_unmap_lpae(struct iommu_domain *domain,
				   unsigned long iova, size_t size,
				   struct iommu_iotlb_gather *iotlb_gather)
{
	unsigned long max_iova;
	unsigned int ret;
	struct iommu_domain_data *data;

	if (!domain) {
		dbg("domain is null\n");
		return -ENODEV;
	}
	data = domain->priv;
	/*calculate the max io virtual address */
	max_iova = data->iova_start + data->iova_size;
	/*check the iova */
	if (iova < data->iova_start)
		goto error;
	if ((iova + size) > max_iova) {
		dbg("iova out of domain range, iova+size=0x%lx, end=0x%lx\n",
		    iova + size, max_iova);
		goto error;
	}
	/*unmapping the range of iova*/
	ret = hisi_smmu_handle_unmapping_lpae(domain, iova, size);
	if (ret == size) {
		dbg("%s:unmap size:0x%x\n", __func__, (unsigned int)size);
		return size;
	} else {
		return 0;
	}
error:
	dbg("%s:the range of io address is wrong\n", __func__);
	return -EINVAL;
}

static phys_addr_t hisi_smmu_iova_to_phys_lpae(struct iommu_domain *domain,
					       dma_addr_t iova)
{
	smmu_pgd_t *pgdp, pgd;
	smmu_pmd_t pmd;
	smmu_pte_t pte;

	pgdp = (smmu_pgd_t *)hisi_smmu_dev->va_pgtable_addr;
	if (!pgdp)
		return 0;

	pgd = *(pgdp + smmu_pgd_index(iova));
	if (smmu_pgd_none_lpae(pgd))
		return 0;

	pmd = *((smmu_pmd_t *)smmu_pmd_page_vaddr_lpae(&pgd) +
			smmu_pmd_index(iova));
	if (smmu_pmd_none_lpae(pmd))
		return 0;

	pte = *((u64 *)smmu_pte_page_vaddr_lpae(&pmd) + smmu_pte_index(iova));
	if (smmu_pte_none_lpae(pte))
		return 0;

	return __pfn_to_phys(pte_pfn(__pte(pte))) | (iova & ~SMMU_PAGE_MASK);
}

static int hisi_attach_dev_lpae(struct iommu_domain *domain, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	struct iommu_domain_data *iommu_info = NULL;

	iommu_info = kzalloc(sizeof(*iommu_info), GFP_KERNEL);
	if (!iommu_info) {
		dbg("alloc iommu_domain_data fail\n");
		return -EINVAL;
	}
	list_add(&iommu_info->list, &hisi_smmu_dev->domain_list);
	domain->priv = iommu_info;
	ret = get_domain_data_lpae(np, domain->priv);
	return ret;
}

static void hisi_detach_dev_lpae(struct iommu_domain *domain,
				 struct device *dev)
{
	struct iommu_domain_data *data;

	data = (struct iommu_domain_data *)domain->priv;
	if (data) {
		list_del(&data->list);
		domain->priv = NULL;
		kfree(data);
	} else {
		dbg("%s:error! data entry has been delected\n", __func__);
	}
}

static bool hisi_smmu_capable(enum iommu_cap cap)
{
	return false;
}

static int hisi_smmu_add_device(struct device *dev)
{
	struct hisi_smmu_device_lpae *iommu = hisi_smmu_dev;
	struct iommu_group *group;

	if (iommu)
		iommu_device_link(&iommu->iommu, dev);
	else
		return -ENODEV;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	iommu_group_put(group);

	return 0;
}

static void hisi_smmu_remove_device(struct device *dev)
{
	struct hisi_smmu_device_lpae *iommu = hisi_smmu_dev;

	if (iommu)
		iommu_device_unlink(&iommu->iommu, dev);

	iommu_group_remove_device(dev);
}

static struct iommu_ops hisi_smmu_ops = {
	.capable	= hisi_smmu_capable,
	.domain_alloc	= hisi_smmu_domain_alloc_lpae,
	.domain_free	= hisi_smmu_domain_free_lpae,
	.attach_dev	= hisi_attach_dev_lpae,
	.detach_dev	= hisi_detach_dev_lpae,
	.map		= hisi_smmu_map_lpae,
	.unmap		= hisi_smmu_unmap_lpae,
	.iova_to_phys	= hisi_smmu_iova_to_phys_lpae,
	.add_device	= hisi_smmu_add_device,
	.remove_device	= hisi_smmu_remove_device,
	.device_group	= generic_device_group,
	.pgsize_bitmap	= SMMU_PAGE_SIZE,
};

static int hisi_smmu_probe_lpae(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	dbg("enter %s\n", __func__);
	hisi_smmu_dev = devm_kzalloc(dev,
				     sizeof(struct hisi_smmu_device_lpae),
				     GFP_KERNEL);

	hisi_smmu_dev->smmu_pgd = devm_kzalloc(dev, SZ_64,
					       GFP_KERNEL | __GFP_DMA);
	if (!hisi_smmu_dev)
		return -ENOMEM;

	hisi_smmu_dev->dev = dev;
	INIT_LIST_HEAD(&hisi_smmu_dev->domain_list);
	spin_lock_init(&hisi_smmu_dev->lock);

	hisi_smmu_dev->smmu_pgd =  (smmu_pgd_t *)(ALIGN((unsigned long)(hisi_smmu_dev->smmu_pgd), SZ_32));

	hisi_smmu_dev->smmu_phy_pgtable_addr =
		virt_to_phys(hisi_smmu_dev->smmu_pgd);
	dev_info(&pdev->dev, "%s, smmu_phy_pgtable_addr is = 0x%llx\n",
		 __func__, hisi_smmu_dev->smmu_phy_pgtable_addr);

	hisi_smmu_dev->va_pgtable_addr = (unsigned long)(hisi_smmu_dev->smmu_pgd);

	ret = iommu_device_sysfs_add(&hisi_smmu_dev->iommu, NULL, NULL,
				     "hisi-iommu");
	if (ret)
		goto fail_register;

	iommu_device_set_ops(&hisi_smmu_dev->iommu, &hisi_smmu_ops);

	ret = iommu_device_register(&hisi_smmu_dev->iommu);
	if (ret) {
		dev_info(&pdev->dev, "Could not register hisi-smmu\n");
		goto fail_register;
	}

	bus_set_iommu(&platform_bus_type, &hisi_smmu_ops);
	return 0;

fail_register:
	iommu_device_sysfs_remove(&hisi_smmu_dev->iommu);
	return ret;
}

static int hisi_smmu_remove_lpae(struct platform_device *pdev)
{
	iommu_device_unregister(&hisi_smmu_dev->iommu);
	iommu_device_sysfs_remove(&hisi_smmu_dev->iommu);

	return 0;
}

static const struct of_device_id hisi_smmu_of_match_lpae[] = {
	{ .compatible = "hisi,hisi-smmu-lpae"},
	{ },
};
MODULE_DEVICE_TABLE(of, hisi_smmu_of_match_lpae);

static struct platform_driver hisi_smmu_driver_lpae = {
	.driver	= {
		.name		= "hisi-smmu-lpae",
		.of_match_table	= of_match_ptr(hisi_smmu_of_match_lpae),
	},
	.probe	= hisi_smmu_probe_lpae,
	.remove	= hisi_smmu_remove_lpae,
};

static int __init hisi_smmu_init_lpae(void)
{
	return platform_driver_register(&hisi_smmu_driver_lpae);
}

static void __exit hisi_smmu_exit_lpae(void)
{
	return platform_driver_unregister(&hisi_smmu_driver_lpae);
}

subsys_initcall(hisi_smmu_init_lpae);
module_exit(hisi_smmu_exit_lpae);

MODULE_DESCRIPTION("IOMMU API for HI3660 architected SMMU implementations");
MODULE_AUTHOR("huawei hisilicon company");
MODULE_LICENSE("GPL v2");
