// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *
 * Samsung DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>

#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "exynos_drm_dp.h"

struct dp_device *dp_drvdata;
EXPORT_SYMBOL(dp_drvdata);

static inline struct dp_device *encoder_to_dp(struct drm_encoder *e)
{
	return container_of(e, struct dp_device, encoder);
}

static enum hotplug_state dp_get_hpd_state(struct dp_device *dp)
{
	enum hotplug_state hpd_current_state;

	mutex_lock(&dp->hpd_state_lock);
	hpd_current_state = dp->hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);

	return hpd_current_state;
}

static void dp_set_hpd_state(struct dp_device *dp, enum hotplug_state hpd_current_state)
{
	mutex_lock(&dp->hpd_state_lock);
	dp->hpd_current_state = hpd_current_state;
	mutex_unlock(&dp->hpd_state_lock);
}

static void dp_init_info(struct dp_device *dp)
{
	dp->state = DP_STATE_INIT;
	dp->hpd_current_state = EXYNOS_HPD_UNPLUG;
	dp->output_type = EXYNOS_DISPLAY_TYPE_DP0_SST1;
}

static void dp_enable(struct drm_encoder *encoder)
{
}

static void dp_disable(struct drm_encoder *encoder)
{
}

/* Delayed Works */
static void dp_work_hpd_plug(struct work_struct *work)
{
	struct dp_device *dp = get_dp_drvdata();

	if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG) {
		mutex_lock(&dp->hpd_lock);

		pm_runtime_get_sync(dp->dev);
		dp_debug(dp, "pm_rtm_get_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));
		pm_stay_awake(dp->dev);

		/* PHY power on */
		usleep_range(10000, 10030);
		dp_hw_init(&dp->hw_config); /* for AUX ch read/write. */
		usleep_range(10000, 11000);

		mutex_unlock(&dp->hpd_lock);
	}
}

static void dp_work_hpd_unplug(struct work_struct *work)
{
	struct dp_device *dp = get_dp_drvdata();

	if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
		mutex_lock(&dp->hpd_lock);

		/* PHY power off */
		dp_hw_deinit();

		pm_runtime_put_sync(dp->dev);
		dp_debug(dp, "pm_rtm_put_sync usage_cnt(%d)\n",
			 atomic_read(&dp->dev->power.usage_count));
		pm_relax(dp->dev);

		mutex_unlock(&dp->hpd_lock);
	}
}

/* ExtCon Handshaking Functions */
static void dp_hpd_changed(struct dp_device *dp, enum hotplug_state state)
{
	if (dp_get_hpd_state(dp) == state) {
		dp_debug(dp, "DP HPD is same state (%x): Skip\n", state);
		return;
	}

	if (state == EXYNOS_HPD_PLUG) {
		dp_info(dp, "DP HPD changed to EXYNOS_HPD_PLUG\n");
		dp_set_hpd_state(dp, state);
		queue_delayed_work(dp->dp_wq, &dp->hpd_plug_work, 0);
	} else if (state == EXYNOS_HPD_UNPLUG) {
		dp_info(dp, "DP HPD changed to EXYNOS_HPD_UNPLUG\n");
		dp_set_hpd_state(dp, state);
		queue_delayed_work(dp->dp_wq, &dp->hpd_unplug_work, 0);
	} else
		dp_err(dp, "DP HPD changed to abnormal state(%d)\n", state);
}

static int usb_typec_dp_notification(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct dp_device *dp = container_of(nb, struct dp_device, dp_typec_nb);
	union extcon_property_value property = { 0 };
	int ret;

	if (action == 1) {
		ret = extcon_get_property(dp->edev, EXTCON_DISP_DP, EXTCON_PROP_DISP_HPD,
					  &property);
		if (!ret) {
			if (property.intval == 1) {
				dp_info(dp, "%s: USB Type-C is HPD PLUG status\n", __func__);

				if (dp_get_hpd_state(dp) == EXYNOS_HPD_UNPLUG) {
					memset(&dp->hw_config, 0, sizeof(struct dp_hw_config));

					ret = extcon_get_property(dp->edev, EXTCON_DISP_DP,
								  EXTCON_PROP_DISP_ORIENTATION,
								  &property);
					if (!ret) {
						dp_info(dp, "%s: disp_orientation = %d\n", __func__,
							property.intval);
						dp->hw_config.orient_type =
							(enum plug_orientation)(property.intval);
					}

					ret = extcon_get_property(dp->edev, EXTCON_DISP_DP,
								  EXTCON_PROP_DISP_PIN_CONFIG,
								  &property);
					if (!ret) {
						dp_info(dp, "%s: disp_pin_config = %d\n", __func__,
							property.intval);
						dp->hw_config.pin_type =
							(enum pin_assignment)(property.intval);

						if (dp->hw_config.pin_type == PIN_TYPE_A ||
						    dp->hw_config.pin_type == PIN_TYPE_C ||
						    dp->hw_config.pin_type == PIN_TYPE_E)
							dp->hw_config.num_lanes = 4;
						else
							dp->hw_config.num_lanes = 2;
					}

					dp_hpd_changed(dp, EXYNOS_HPD_PLUG);
				} else
					dp_warn(dp, "%s: HPD is already set\n", __func__);
			} else {
				dp_info(dp, "%s: USB Type-C is HPD UNPLUG status\n", __func__);

				if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG)
					dp_hpd_changed(dp, EXYNOS_HPD_UNPLUG);
			}
		} else
			dp_err(dp, "%s: extcon_get_property() is failed\n", __func__);
	} else {
		dp_info(dp, "%s: USB Type-C is not in display ALT mode\n", __func__);

		if (dp_get_hpd_state(dp) == EXYNOS_HPD_PLUG)
			dp_hpd_changed(dp, EXYNOS_HPD_UNPLUG);
	}

	return NOTIFY_OK;
}

static void dp_register_extcon_notifier(void)
{
	struct dp_device *dp = get_dp_drvdata();
	int ret;

	if (!dp->notifier_registered) {
		dp->dp_typec_nb.notifier_call = usb_typec_dp_notification;
		ret = extcon_register_notifier(dp->edev, EXTCON_DISP_DP, &dp->dp_typec_nb);
		if (ret < 0)
			dp_err(dp, "failed to register PDIC Notifier\n");

		dp->notifier_registered = 1;
		dp_debug(dp, "PDIC Notifier has been registered\n");
	}
}

/* DP DRM Connector Helper Functions */
static enum drm_mode_status dp_mode_valid(struct drm_encoder *encoder,
					  const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void dp_atomic_mode_set(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
}

static int dp_atomic_check(struct drm_encoder *encoder,
			   struct drm_crtc_state *crtc_state,
			   struct drm_connector_state *state)
{
	return 0;
}

static const struct drm_encoder_helper_funcs dp_encoder_helper_funcs = {
	.mode_valid = dp_mode_valid,
	.atomic_mode_set = dp_atomic_mode_set,
	.enable = dp_enable,
	.disable = dp_disable,
	.atomic_check = dp_atomic_check,
};

/* DP DRM Encoder Functions */
static const struct drm_encoder_funcs dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

/* DP DRM Connector Functions */
static const struct drm_connector_funcs dp_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* DP DRM Connector Helper Functions */
static int dp_detect(struct drm_connector *connector, struct drm_modeset_acquire_ctx *ctx,
		     bool force)
{
	return connector_status_unknown;
}

static int dp_get_modes(struct drm_connector *connector)
{
	return 0;
}

static const struct drm_connector_helper_funcs dp_connector_helper_funcs = {
	.detect_ctx = dp_detect,
	.get_modes = dp_get_modes,
};

/* DP DRM Component Functions */
static int dp_create_connector(struct drm_encoder *encoder)
{
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_connector *connector = &dp->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector, &dp_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dp_err(dp, "failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &dp_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static int dp_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);
	struct drm_device *drm_dev = data;
	int ret = 0;

	drm_encoder_init(drm_dev, encoder, &dp_encoder_funcs, DRM_MODE_ENCODER_LVDS, NULL);
	drm_encoder_helper_add(encoder, &dp_encoder_helper_funcs);

	encoder->possible_crtcs = exynos_drm_get_possible_crtcs(encoder, dp->output_type);
	if (!encoder->possible_crtcs) {
		dp_err(dp, "failed to get possible crtc, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return -ENOTSUPP;
	}

	ret = dp_create_connector(encoder);
	if (ret) {
		dp_err(dp, "failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	dp_info(dp, "DP Driver has been binded\n");

	dp_register_extcon_notifier();

	return ret;
}

static void dp_unbind(struct device *dev, struct device *master, void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct dp_device *dp = encoder_to_dp(encoder);

	dp_debug(dp, "%s +\n", __func__);
	dp_debug(dp, "%s -\n", __func__);
}

static const struct component_ops dp_component_ops = {
	.bind = dp_bind,
	.unbind = dp_unbind,
};

/* DP DRM Driver */
static int dp_parse_dt(struct dp_device *dp, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dp_err(dp, "no device tree information\n");
		return -EINVAL;
	}

	if (!of_property_read_bool(dev->of_node, "extcon")) {
		dp_err(dp, "no extcon in device tree\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t dp_irq_handler(int irq, void *dev_data)
{
	return IRQ_HANDLED;
}

static int dp_init_resources(struct dp_device *dp, struct platform_device *pdev)
{
	struct resource *res;
	u64 addr_phy = 0x110F0000;
	u64 size_phy = 0x2800;
	int ret = 0;

	/* DP Link SFR */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dp_err(dp, "failed to get mem resource\n");
		return -ENOENT;
	}

	dp->res.link_regs = devm_ioremap_resource(dp->dev, res);
	if (IS_ERR(dp->res.link_regs)) {
		dp_err(dp, "failed to remap DP LINK SFR region\n");
		return -EINVAL;
	}
	dp_regs_desc_init(dp->res.link_regs, res->start, "LINK", REGS_LINK, SST1);

	/* USBDP Combo PHY SFR */
	// ToDo: USBDP PHY is shared with USB Driver.
	// In order to avoid double resource mapping, here uses hard-code.
	// Need to revisit
	dp->res.phy_regs = ioremap((phys_addr_t)addr_phy, size_phy);
	if (IS_ERR(dp->res.phy_regs)) {
		dp_err(dp, "failed to remap USBDP Combo PHY SFR region\n");
		return -EINVAL;
	}
	dp_regs_desc_init(dp->res.phy_regs, (phys_addr_t)addr_phy, "PHY", REGS_PHY, SST1);

	/* DP Interrupt */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dp_err(dp, "failed to get irq resource\n");
		return -ENOENT;
	}

	dp->res.irq = res->start;
	ret = devm_request_irq(dp->dev, res->start, dp_irq_handler, 0, pdev->name, dp);
	if (ret) {
		dp_err(dp, "failed to install DP irq\n");
		return -EINVAL;
	}
	disable_irq(dp->res.irq);

	return 0;
}

static int dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dp_device *dp = NULL;
	int ret = 0;

	dp = devm_kzalloc(dev, sizeof(struct dp_device), GFP_KERNEL);
	if (!dp) {
		dev_err(dev, "failed to allocate dp device.\n");
		return -ENOMEM;
	}
	dp->dev = dev;

	ret = dp_parse_dt(dp, dev);
	if (ret) {
		dp_err(dp, "failed to parse dp device tree.\n");
		return ret;
	}

	ret = dp_init_resources(dp, pdev);
	if (ret) {
		dp_err(dp, "failed to init dp resources.\n");
		return ret;
	}

	platform_set_drvdata(pdev, dp);

	/* Driver Initialization */
	dp_drvdata = dp;
	dp_init_info(dp);

	dma_set_mask(dev, DMA_BIT_MASK(32));

	mutex_init(&dp->hpd_lock);
	mutex_init(&dp->hpd_state_lock);

	/* Create WorkQueue & Delayed Works*/
	dp->dp_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!dp->dp_wq) {
		dp_err(dp, "create wq failed.\n");
		ret = -ENOMEM;
		goto err;
	}

	INIT_DELAYED_WORK(&dp->hpd_plug_work, dp_work_hpd_plug);
	INIT_DELAYED_WORK(&dp->hpd_unplug_work, dp_work_hpd_unplug);

	/* Register callback to ExtCon */
	dp->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR_OR_NULL(dp->edev)) {
		dp_err(dp, "error while retrieving extcon dev\n");
		ret = -ENOMEM;
		goto err;
	}

	pm_runtime_enable(dev);

	dp_info(dp, "DP Driver has been probed.\n");
	return component_add(dp->dev, &dp_component_ops);

err:
	return ret;
}

static int dp_remove(struct platform_device *pdev)
{
	struct dp_device *dp = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	mutex_destroy(&dp->hpd_lock);
	mutex_destroy(&dp->hpd_state_lock);

	destroy_workqueue(dp->dp_wq);

	dp_info(dp, "DP Driver has been removed\n");
	return 0;
}

static const struct of_device_id dp_of_match[] = {
	{ .compatible = "samsung,exynos-dp" },
	{},
};
MODULE_DEVICE_TABLE(of, dp_of_match);

struct platform_driver dp_driver __refdata = {
	.probe			= dp_probe,
	.remove			= dp_remove,
	.driver = {
		.name		= "exynos-drmdp",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(dp_of_match),
	}
};

MODULE_AUTHOR("YongWook Shin <yongwook.shin@samsung.com>");
MODULE_DESCRIPTION("Samusung DisplayPort driver");
MODULE_LICENSE("GPL");
