/*
 * Copyright 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Alex <alex.c.li@intel.com>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_esd_handler.h"


struct esd_private i915_esd = { -1, DISPLAY_DSI_STATE_OK };

static void i915_esd_work_func(struct work_struct *work)
{
	struct esd_private *i915_esd_priv =
		container_of(work, struct esd_private, esd_work);
         mutex_lock(&i915_esd_priv->reset_mutex);
	i915_esd_priv->dsi_esd_state = DISPLAY_DSI_STATE_HANG;
     if(i915_esd_priv->reset_flasg==ESD_NONE)
             i915_esd_priv->reset_flasg = ESD_IRQ_RESET;
       DRM_INFO("ESD interrupt triggered\n");
        intel_connector_reset(i915_esd_priv->esd_connector);
       i915_esd_priv->flip_fail_count = 0;
       mutex_unlock(&i915_esd_priv->reset_mutex);

}

static void i915_esd_delay_work_func(struct work_struct *work)
{
	struct esd_private *i915_esd_priv =
		container_of(work, struct esd_private,
		esd_delay_work.work);

	/*reenable ESD detect IRQ*/
	DRM_DEBUG_DRIVER("ESD interrupt enabled\n");
	enable_irq(i915_esd_priv->esd_gpio_irq_mapped);
	return;
}

static irqreturn_t i915_dsi_hang_irq_handler(int irq, void *arg)
{
	struct esd_private *i915_esd_priv = arg;
		DRM_DEBUG_DRIVER("ESD interrupt received \n");

	if (irq == i915_esd_priv->esd_gpio_irq_mapped) {
		DRM_DEBUG_DRIVER("ESD interrupt received (irq: %d)\n", irq);
		/*disable ESD detect interrupt until reset done to avoid
		interrupt storm and GPIO line jitter when Power
		on/off DSI HW and panel*/
		disable_irq_nosync(irq);
		/*Add task to handle connector reset process in another thread
		and exit IRQ context ASAP*/
		schedule_work(&i915_esd_priv->esd_work);
		return IRQ_HANDLED;
	}

	DRM_ERROR("ESD: Unexpect interrupt received (irq: %d)\n", irq);

	return IRQ_NONE;
}

/**
* intel_dsi_esd_init - request GPIO and GPIO IRQ for display hang detect
* (ESD). The GPIO num and IRQ trigger type can be changed according
* different platform and configuration
* @connector: display port (drm connector) which may hang and detected
*by GPIO IRQ
*/
void intel_dsi_esd_init(struct drm_connector *connector)
{
	u32 te_config;
	struct drm_i915_private *dev_priv = connector->dev->dev_private;

#ifndef FEATURE_ESD_RESET
	DRM_INFO("Don't support ESD.");
	return;
#endif

	if (i915_esd.esd_gpio_irq_mapped > 0) {
		DRM_DEBUG_DRIVER("ESD GPIO IRQ already inited befores\n");
		return;
	}
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SUS,CHV_GPIO_TE_CFG0,0x60008200); //input 20k pullup
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SUS,CHV_GPIO_TE_CFG1,0x4c00402); //raising egde
	te_config =vlv_gpio_read(dev_priv,IOSF_PORT_GPIO_SUS,CHV_GPIO_TE_CFG0); 
	DRM_DEBUG_DRIVER("te config:%x",te_config);
	i915_esd.esd_connector = connector;

	i915_esd.dsi_esd_state = DISPLAY_DSI_STATE_OK;
	i915_esd.reset_flasg = ESD_NONE;
	mutex_init(&i915_esd.reset_mutex);

	INIT_WORK(&i915_esd.esd_work, i915_esd_work_func);
	INIT_DELAYED_WORK(&i915_esd.esd_delay_work,
		i915_esd_delay_work_func);

	/*According intel schematic, the ESD hang detect pin connected
	to ISH_GPIO_9 which mapped as GPIO 334, please change the GPIO
	to coresponding num if platform changed*/
	if (gpio_request(ESD_IRQ_GPIO_NUM, ESD_GPIO_INT_NAME)) {
		DRM_ERROR("ESD detect GPIO request failure: pin=%d;\n",
			ESD_IRQ_GPIO_NUM);
		return;
	}

	i915_esd.esd_gpio_irq_mapped = gpio_to_irq(ESD_IRQ_GPIO_NUM);
	DRM_DEBUG_DRIVER("esd irq:%d\n",i915_esd.esd_gpio_irq_mapped);
	if (i915_esd.esd_gpio_irq_mapped < 0) {
		DRM_ERROR("ESD GPIO to IRQ mapping failure %s\n",
			ESD_GPIO_INT_NAME);
		/*free the GPIO*/
		gpio_free(ESD_IRQ_GPIO_NUM);
		return;
	}

	/*register ESD hang detect interrupt at GPIO High Level*/
	if (request_threaded_irq(i915_esd.esd_gpio_irq_mapped, NULL,
			(irq_handler_t)i915_dsi_hang_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, ESD_GPIO_INT_NAME,
			&i915_esd)) {
		DRM_ERROR("ESD Irq Request failure\n");
		i915_esd.esd_gpio_irq_mapped = -1;
		/*free the IRQ*/
		free_irq(i915_esd.esd_gpio_irq_mapped, NULL);
		/*free the GPIO*/
		gpio_free(ESD_IRQ_GPIO_NUM);
		return;
	}
	DRM_DEBUG_DRIVER("ESD IRQ[%d] Install Success. Mapped GPIO is %d\n",
		i915_esd.esd_gpio_irq_mapped, ESD_IRQ_GPIO_NUM);
	return;
}

/**
* intel_dsi_esd_uninit - release GPIO and GPIO IRQ for display hang
* detect (ESD)
* @connector: display port (drm connector) which release hang detect IRQ
*/
void intel_dsi_esd_destroy(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	DRM_DEBUG_KMS("\n");

#ifndef FEATURE_ESD_RESET
	return;
#endif

	if (-1 != i915_esd.esd_gpio_irq_mapped) {
		free_irq(i915_esd.esd_gpio_irq_mapped, dev);
		i915_esd.esd_gpio_irq_mapped = -1;
		gpio_free(ESD_IRQ_GPIO_NUM);
		i915_esd.dsi_esd_state = DISPLAY_DSI_STATE_OK;
                i915_esd.reset_flasg = ESD_NONE;
		DRM_DEBUG_DRIVER("ESD uninited\n");
	}
	return;
}

/**
* intel_dsi_esd_post_dpms_handler - function to handle some necessary
* process after normal DPMS process done. For example, when ESD IRQ
* triggered, IRQ should disabled to avoid repeat interrupt triggered.
* The IRQ should reenable after display reset done (after DPMS_ON cmd).
* @connector: display port (drm connector) which will do dpms off/on
* @mode:  dpms on/off mode
*/
void intel_dsi_esd_post_dpms_handler(struct drm_connector *connector,
	int mode)
{
    struct drm_device *dev = connector->dev;
    struct drm_i915_private *dev_priv = dev->dev_private;
    int power_count=0;

#ifndef FEATURE_ESD_RESET
	return;
#endif

	/*init esd detect first if it not inited before*/
	if (i915_esd.esd_gpio_irq_mapped < 0)
		intel_dsi_esd_init(connector);

	if ((DISPLAY_DSI_STATE_HANG == i915_esd.dsi_esd_state) &&
	    (DRM_MODE_DPMS_OFF == mode)) {
		/*ESD detected, and get DPMS off from HWC*/
		i915_esd.dsi_esd_state = DISPLAY_DSI_STATE_DONE;
              //power off pipe and power on again
            while(intel_display_power_enabled(dev_priv, POWER_DOMAIN_PIPE_A)){
                intel_display_power_put(dev_priv, POWER_DOMAIN_PIPE_A);
                power_count++;
               DRM_DEBUG_KMS("ESD put pipe power\n");
                msleep(20);
            }
            while(power_count){
                intel_display_power_get(dev_priv, POWER_DOMAIN_PIPE_A);
                power_count--;
            }

		DRM_DEBUG_DRIVER("ESD handle after DPMS Off done\n");
	} else if ((DISPLAY_DSI_STATE_OK != i915_esd.dsi_esd_state) &&
	    (DRM_MODE_DPMS_ON == mode)) {
		/*ESD detected, and get DPMS On from HWC after reset*/
		i915_esd.dsi_esd_state = DISPLAY_DSI_STATE_OK;
                i915_esd.reset_flasg = ESD_NONE;
		/*Wait for while to enable external system is stable to
		avoid IRQ storm, add delayed work to enable IRQ*/
		schedule_delayed_work(&i915_esd.esd_delay_work,
			msecs_to_jiffies(5000));
		DRM_DEBUG_DRIVER("ESD handle after DPMS On done\n");
	}
	return;
}
