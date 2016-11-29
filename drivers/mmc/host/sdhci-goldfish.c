/*
 *  Copyright 2007, Google Inc.
 *  Copyright 2012, Intel Inc.
 *
 *  based on omap.c driver, which was
 *  Copyright (C) 2004 Nokia Corporation
 *  Written by Tuukka Tikkanen and Juha Yrjölä <juha.yrjola@nokia.com>
 *  Misc hacks here and there by Tony Lindgren <tony@atomide.com>
 *  Other hacks (DMA, SD, etc) by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "sdhci-pltfm.h"

#define DRIVER_NAME "sdhci_goldfish"

#define SDHCI_CAPS				0x100

#define SDHCI_CAPS2				0x104

#define SDHCI_BUFFER_SIZE		0x108

#define SDHCI_QUIRKs			0x10C


//#define MMC_CAP2_CAN_DO_CMDQ	(1 << 26)
//#define MMC_CAP2_HYBRID_MODE	(1 << 27)	/* Support Hybrid mode */

static unsigned int sdhci_goldfish_get_min_clock(struct sdhci_host *host) {
	//TOOD: fill with right values
	//according to android-goldfish.c
	//f_max & f_min
	printk("sdhci_goldfish_get_min_clock\n");
	return 0;
}

static unsigned int sdhci_goldfish_get_max_clock(struct sdhci_host *host) {
	//TOOD: fill with right values
	//according to android-goldfish.c
	//f_max & f_min
	printk("sdhci_goldfish_get_max_clock\n");
	return 0;
}

static void sdhci_goldfish_reset(struct sdhci_host *host, u8 mask){
	printk("sdhci_goldfish_reset\n");
}

static void sdhci_goldfish_set_clock(struct sdhci_host *host, unsigned int clock)
{
	printk("sdhci_goldfish_set_clock\n");  
}

static void sdhci_goldfish_set_bus_width (struct sdhci_host *host, int width) {
	printk("set_bus_width\n");  	
}

static struct sdhci_ops sdhci_goldfish_ops = {
	.get_min_clock = sdhci_goldfish_get_min_clock,
	.get_max_clock = sdhci_goldfish_get_max_clock,
	.set_clock = sdhci_goldfish_set_clock,
};

static int goldfish_mmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct sdhci_host *host = NULL;

	struct resource *res;
	int ret = 0;
	int irq;
	dma_addr_t buf_addr;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	mmc = mmc_alloc_host(sizeof(struct sdhci_host), &pdev->dev);
	if (mmc == NULL) {
		ret = -ENOMEM;
		goto err_alloc_host_failed;
	}
	host = mmc_priv(mmc);
	host->mmc = mmc;
	/*
	TODO: investigate later - removed to get successfult compilation in ranchu kernel
	drivers/mmc/host/sdhci-goldfish.c:119:18: error: ‘struct sdhci_host’ has no member named ‘ios_mutex’
        */
	//mutex_init(&host->ios_mutex); // it got used in sdhci.c as mutex_lock() and mutex_unlock()

	pr_err("mmc: Mapping %lX to %lX\n", (long)res->start, (long)res->end);
	
	host->ioaddr = ioremap(res->start, resource_size(res));

	if (host->ioaddr == NULL) {
		ret = -ENOMEM;
		goto ioremap_failed;
	}

	host->irq = irq;
	
	mmc->f_min = 400000;
	mmc->f_max = 24000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;

	#ifdef CONFIG_LGE_MMC_CQ_ENABLE
	//mmc->caps2 |= MMC_CAP2_CAN_DO_CMDQ;
	//mmc->caps2 |= MMC_CAP2_HYBRID_MODE;
	#endif
	/* Use scatterlist DMA to reduce per-transfer costs.
	 * NOTE max_seg_size assumption that small blocks aren't
	 * normally used (except e.g. for reading SD registers).
	 */

	platform_set_drvdata(pdev, host);

	//VERY Important in order to force "sdhci_do_get_cd" to return 1
	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	host->ops = &sdhci_goldfish_ops;

	//host->quirks2 |= SDHCI_QUIRK2_HOST_NO_CMD23;

	ret = sdhci_add_host(host);

	printk("sdhci-goldfish driver configuration after adding sdhci host:\n");
	printk("mmc->caps:  %X\n", mmc->caps);
	printk("mmc->caps2: %X\n", mmc->caps2);
	printk("buffer_size %u\n\n", mmc->max_req_size);

	sdhci_writel(host, mmc->caps, SDHCI_CAPS);
	sdhci_writel(host, mmc->caps2, SDHCI_CAPS2);
	mmc->caps = sdhci_readl(host, SDHCI_CAPS);
	mmc->caps2 = sdhci_readl(host, SDHCI_CAPS2);
	mmc->max_req_size = sdhci_readl(host, SDHCI_BUFFER_SIZE);
	
	printk("The modified sdhci-goldfish driver configuration:\n");
	printk("mmc->caps:  %X\n", mmc->caps);
	printk("mmc->caps2: %X\n", mmc->caps2);
	printk("buffer_size %u\n\n", mmc->max_req_size);


	return ret;

ioremap_failed:
	mmc_free_host(host->mmc);
err_alloc_host_failed:
	return ret;
}

static int goldfish_mmc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	BUG_ON(host == NULL);
	mmc_remove_host(host->mmc);
	free_irq(host->irq, host);
	iounmap(host->ioaddr);
	mmc_free_host(host->mmc);
	return 0;
}

static struct platform_driver goldfish_mmc_driver = {
	.probe		= goldfish_mmc_probe,
	.remove		= goldfish_mmc_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};
module_platform_driver(goldfish_mmc_driver);
MODULE_LICENSE("GPL v2");
