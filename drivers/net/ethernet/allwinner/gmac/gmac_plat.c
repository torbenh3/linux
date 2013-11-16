/*******************************************************************************
  This contains the functions to handle the platform driver.

  Copyright (C) 2012 Shuge

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/clk.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_net.h>
//#include <mach/irqs.h>
//#include <mach/clock.h>

#include "sunxi_gmac.h"

static int gmac_pltfr_remove(struct platform_device *pdev);

#ifdef CONFIG_OF
static int gmac_probe_config_dt(struct platform_device *pdev,
				  struct gmac_plat_data *plat,
				  const char **mac)
{
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return -ENODEV;

	*mac = of_get_mac_address(np);
	plat->phy_interface = of_get_phy_mode(np);

	plat->bus_id = of_alias_get_id(np, "ethernet");
	if (plat->bus_id < 0)
		plat->bus_id = 0;

	if (of_property_read_u32(np, "snps,phy-addr", &plat->phy_addr))
		plat->phy_addr = -1;

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(struct gmac_mdio_bus_data),
					   GFP_KERNEL);

	if (of_device_is_compatible(np, "allwinner,gmac")) {
		plat->clk_csr = 2;
		plat->tx_coe = 1;
		plat->force_sf_dma_mode = 1;

	}

	if (of_find_property(np, "snps,pbl", NULL)) {
		of_property_read_u32(np, "snps,pbl", &plat->pbl);
	}


	return 0;
}
#else
static int gmac_probe_config_dt(struct platform_device *pdev,
				  struct gmac_plat_data *plat,
				  const char **mac)
{
	return -ENOSYS;
}
#endif /* CONFIG_OF */

static int gmac_sys_request(struct platform_device *pdev, struct gmac_priv *priv)
{
	struct resource *clk_reg;

	clk_reg = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->gmac_clk_reg = devm_ioremap_resource(&pdev->dev, clk_reg);
	if (IS_ERR(priv->gmac_clk_reg))
		return PTR_ERR(priv->gmac_clk_reg);

	priv->gmac_ahb_clk = devm_clk_get(&pdev->dev, "stmmaceth" );
	if (unlikely(!priv->gmac_ahb_clk || IS_ERR(priv->gmac_ahb_clk))) {
		printk(KERN_ERR "ERROR: Get clock is failed!\n");
		return PTR_ERR(priv->gmac_ahb_clk);
	}


	return 0;
}

static int gmac_pltfr_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq = 0;
	struct resource *io_gmac;
	struct device *dev = &pdev->dev;
	void __iomem *addr = NULL;
	struct gmac_priv *priv = NULL;
	struct gmac_plat_data *plat_dat = NULL;
	const char *mac = NULL;

	io_gmac = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(dev, io_gmac);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	plat_dat = dev_get_platdata(&pdev->dev);
	if (pdev->dev.of_node) {
		if (!plat_dat)
			plat_dat = devm_kzalloc(&pdev->dev,
					sizeof(struct gmac_plat_data),
					GFP_KERNEL);
		if (!plat_dat) {
			pr_err("%s: ERROR: no memory", __func__);
			return  -ENOMEM;
		}

		ret = gmac_probe_config_dt(pdev, plat_dat, &mac);
		if (ret) {
			pr_err("%s: main dt probe failed", __func__);
			return ret;
		}

		dev->platform_data = plat_dat;
	}

	/* Get the MAC information */
	irq = platform_get_irq_byname(pdev, "macirq");
	if (irq == -ENXIO) {
		printk(KERN_ERR "%s: ERROR: MAC IRQ configuration "
		       "information not found\n", __func__);
		return -ENXIO;
	}

	priv = gmac_dvr_probe(&(pdev->dev), addr, irq);
	if (!priv) {
		printk("[gmac]: %s: main driver probe failed", __func__);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, priv->ndev);

	if(gmac_sys_request(pdev, priv)) {
		gmac_pltfr_remove(pdev);
		return -ENODEV;
	}

	printk("[gmac]: sun6i_gmac platform driver registration completed");

	return 0;
}

static int gmac_pltfr_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	int ret = gmac_dvr_remove(ndev);

	platform_set_drvdata(pdev, NULL);

	return ret;
}

#ifdef CONFIG_PM
static int gmac_pltfr_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return gmac_suspend(ndev);
}

static int gmac_pltfr_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return gmac_resume(ndev);
}

int gmac_pltfr_freeze(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return gmac_freeze(ndev);
}

int gmac_pltfr_restore(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	return gmac_restore(ndev);
}

static const struct dev_pm_ops gmac_pltfr_pm_ops = {
	.suspend = gmac_pltfr_suspend,
	.resume = gmac_pltfr_resume,
	.freeze = gmac_pltfr_freeze,
	.thaw = gmac_pltfr_restore,
	.restore = gmac_pltfr_restore,
};
#else
static const struct dev_pm_ops gmac_pltfr_pm_ops;
#endif /* CONFIG_PM */

static const struct of_device_id gmac_dt_ids[] = {
	{ .compatible = "allwinner,gmac"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gmac_dt_ids);

struct platform_driver gmac_driver = {
	.probe	= gmac_pltfr_probe,
	.remove = gmac_pltfr_remove,
	.driver = {
		   .name = GMAC_RESOURCE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &gmac_pltfr_pm_ops,
		   .of_match_table = of_match_ptr(gmac_dt_ids),
		   },
};

#if 1
static struct resource gmac_resources[] = {
	[0] = {
		.name	= "gmacio",
		.start	= GMAC_BASE,
		.end	= GMAC_BASE + 0x1054,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "clkbus",
		.start	= CCMU_BASE,
		.end	= CCMU_BASE + GMAC_CLK_REG,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.name	= "gpio",
		.start	= GPIO_BASE,
		.end	= GPIO_BASE + 0x0c,
		.flags	= IORESOURCE_MEM,
	},
	[3] = {
		.name	= "gmac_clk_reg",
		.start	= CCMU_BASE,
		.end	= CCMU_BASE + GMAC_CLK_REG,
		.flags	= IORESOURCE_MEM,
	},
	[4] = {
		.name	= "gmacirq",
		.start	= 85 + 32,
		.end	= 85 + 32,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct gmac_mdio_bus_data gmac_mdio_data = {
	.bus_id = 0,
	.phy_reset  = NULL,
	.phy_mask = 0,
	.irqs = NULL,
	.probed_phy_irq = 0,
};

static struct gmac_plat_data gmac_platdata ={
	.bus_id = 0,
	.phy_addr = -1,
	.phy_interface = PHY_INTERFACE_MODE_RGMII,
	.clk_csr = 2,

	.tx_coe = 1,
	.bugged_jumbo = 0,
	.force_sf_dma_mode = 1,
	.pbl = 2,
	.mdio_bus_data = &gmac_mdio_data,
};

static void gmac_device_release(struct device *dev)
{
}

struct platform_device gmac_device = {
	.name = GMAC_RESOURCE_NAME,
	.id = -1,
	.resource = gmac_resources,
	.num_resources = ARRAY_SIZE(gmac_resources),
	.dev = {
		.release = gmac_device_release,
		.platform_data = &gmac_platdata,
	},
};
#endif

