/*
 * Allwinner A31 Reset Controller driver
 *
 * Copyright 2013 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/types.h>

struct sun6i_reset_data {
	void __iomem				*membase;
	struct reset_controller_dev		rcdev;
};

static int sun6i_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct sun6i_reset_data *data = container_of(rcdev,
						     struct sun6i_reset_data,
						     rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	u32 reg = readl(data->membase + (bank * 4));

	writel(reg & ~BIT(offset), data->membase + (bank * 4));

	return 0;
}

static int sun6i_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct sun6i_reset_data *data = container_of(rcdev,
						     struct sun6i_reset_data,
						     rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	u32 reg = readl(data->membase + (bank * 4));

	writel(reg | BIT(offset), data->membase + (bank * 4));

	return 0;
}

static struct reset_control_ops sun6i_reset_ops = {
	.assert		= sun6i_reset_assert,
	.deassert	= sun6i_reset_deassert,
};

int __init sun6i_reset_init(struct device_node *np)
{
	struct sun6i_reset_data *data;
	struct resource res;
	resource_size_t size;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		goto err_alloc;

	size = resource_size(&res);
	if (!request_mem_region(res.start, size, np->name)) {
		ret = -EBUSY;
		goto err_alloc;
	}

	data->membase = ioremap(res.start, size);
	if (!data->membase) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = size * 32;
	data->rcdev.ops = &sun6i_reset_ops;
	data->rcdev.of_node = np;
	reset_controller_register(&data->rcdev);

	return 0;

err_alloc:
	kfree(data);
	return ret;
}
