/*
 * zl380tw.c  --  zl380tw ALSA Soc Audio driver
 *
 * Copyright 2014 Microsemi Inc.
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/errno.h>

#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#if VPROC_CODEC_MIXER_ENABLE
#include "typedefs.h"
#include "ssl.h"
#include "chip.h"
#include "hbi.h"
#endif
/* driver private data */
struct zl380xx {
    int sysclk;
    struct platform_device *dev;
#if VPROC_CODEC_MIXER_ENABLE
    hbi_handle_t handle;
#endif
} *zl380xx_priv;

/*--------------------------------------------------------------------
 *    ALSA  SOC CODEC driver
 *--------------------------------------------------------------------*/
#if VPROC_CODEC_MIXER_ENABLE
static int dev_addr=0;
static int bus_num=0;

module_param(dev_addr, uint, S_IRUGO);
MODULE_PARM_DESC(dev_addr, "device address (example 0x45 for i2c, chip select 0 or 1 for spi");

module_param(bus_num, uint, S_IRUGO);
MODULE_PARM_DESC(bus_num, "device bus number (example 0 / 1");

static int zl380xx_control_write(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol);

static int zl380xx_control_read(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol);
#endif                                
/*Formatting for the Audio*/
#define zl380xx_DAI_RATES            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000)
#define zl380xx_DAI_FORMATS          (SNDRV_PCM_FMTBIT_S16_LE)
#define zl380xx_DAI_CHANNEL_MIN      1
#define zl380xx_DAI_CHANNEL_MAX      2

static int zl380xx_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_codec *codec = rtd->codec;
    struct zl380xx *zl380xx = snd_soc_codec_get_drvdata(codec);
    int sample_rate = params_rate(params), bits_per_frame = 0;

	if (sample_rate) {
		bits_per_frame = zl380xx->sysclk / sample_rate;   
        dev_info(codec->dev, "TDM clk = %d, bits_per_frame = %d, sample_rate = %d", zl380xx->sysclk, bits_per_frame, sample_rate);
    }
    return 0;
}

static int zl380xx_set_dai_fmt(struct snd_soc_dai *codec_dai,
                              unsigned int fmt)
{
    return 0;
}

static int zl380xx_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
                                    unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    struct zl380xx *zl380xx = snd_soc_codec_get_drvdata(codec);
    zl380xx->sysclk = freq;
    return 0;
}

static const struct snd_soc_dai_ops zl380xx_dai_ops = {
        .set_fmt        = zl380xx_set_dai_fmt,
        .set_sysclk     = zl380xx_set_dai_sysclk,
        .hw_params   	= zl380xx_hw_params,

};

static struct snd_soc_dai_driver zl380xx_dai = {
    .name = "zl380tw-hifi",
    .playback = {
        .stream_name = "Playback",
        .channels_min = zl380xx_DAI_CHANNEL_MIN,
        .channels_max = zl380xx_DAI_CHANNEL_MAX,
        .rates = zl380xx_DAI_RATES,
        .formats = zl380xx_DAI_FORMATS,
    },
    .capture = {
        .stream_name = "Capture",
        .channels_min = zl380xx_DAI_CHANNEL_MIN,
        .channels_max = zl380xx_DAI_CHANNEL_MAX,
        .rates = zl380xx_DAI_RATES,
        .formats = zl380xx_DAI_FORMATS,
    },
    .ops = &zl380xx_dai_ops,
};
EXPORT_SYMBOL(zl380xx_dai);
#if VPROC_CODEC_MIXER_ENABLE
static const struct snd_kcontrol_new zl380xx_snd_controls[] = {
    SOC_SINGLE_EXT("DAC1 GAIN INA", ZL380xx_CP_DAC1_GAIN_REG, 0, 0x6, 0,
                    zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("DAC2 GAIN INA", ZL380xx_CP_DAC2_GAIN_REG, 0, 0x6, 0,
                    zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("DAC1 GAIN INB", ZL380xx_CP_DAC1_GAIN_REG, 8, 0x6, 0,
            zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("DAC2 GAIN INB", ZL380xx_CP_DAC2_GAIN_REG, 8, 0x6, 0,
            zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("MUTE SPEAKER ROUT", ZL380xx_AEC_CTRL0_REG, 7, 1, 0,
                    zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("MUTE MIC SOUT", ZL380xx_AEC_CTRL0_REG, 8, 1, 0,
                    zl380xx_control_read, zl380xx_control_write),
    SOC_SINGLE_EXT("AEC MIC GAIN", ZL380xx_DIG_MIC_GAIN_REG, 0, 0x7, 0,
                    zl380xx_control_read, zl380xx_control_write),
	SOC_SINGLE_EXT("AEC ROUT GAIN", ZL380xx_ROUT_GAIN_CTRL_REG, 0, 0x78, 0,
		            zl380xx_control_read, zl380xx_control_write),
	SOC_SINGLE_EXT("AEC ROUT GAIN EXT", ZL380xx_ROUT_GAIN_CTRL_REG, 7, 0x7, 0,
		        	zl380xx_control_read, zl380xx_control_write),
	SOC_SINGLE_EXT("AEC SOUT GAIN", ZL380xx_SOUT_GAIN_CTRL_REG, 8, 0xf, 0,
		        	zl380xx_control_read, zl380xx_control_write),

};

static int zl380xx_control_read(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

    unsigned int reg = mc->reg;
    unsigned int shift = mc->shift;
    unsigned int mask = mc->max;
    unsigned int invert = mc->invert;
    unsigned char buf[2];
    hbi_status_t status =HBI_STATUS_SUCCESS;
    unsigned int val=0;

    status = HBI_read(zl380xx_priv->handle, reg, buf,2);
    val=buf[0];
    val=(val << 8)|buf[1];
    
    ucontrol->value.integer.value[0] = ((val >> shift) & mask);

    if (invert)
        ucontrol->value.integer.value[0] = mask - ucontrol->value.integer.value[0];

    return 0;
}


static int zl380xx_control_write(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{

    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

    reg_addr_t reg = mc->reg;
    unsigned int shift = mc->shift;
    unsigned int mask = mc->max;
    unsigned int invert = mc->invert;
    unsigned int val = (ucontrol->value.integer.value[0] & mask);
    unsigned int valt = 0;
    user_buffer_t buf[2];
    hbi_status_t status;

    if (invert)
        val = mask - val;

    status = HBI_read(zl380xx_priv->handle, reg, buf,2);
    if (status != HBI_STATUS_SUCCESS){
        return -EIO;
    }

    valt=buf[0];
    valt=(valt << 8)|buf[1];

    if (((valt >> shift) & mask) == val) {
        return 0;
    }


    valt &= ~(mask << shift);
    valt |= val << shift;

    buf[0]=valt>>8;
    buf[1]=valt&0xFF;

    status = HBI_write(zl380xx_priv->handle, reg,buf,2);

    if (status != HBI_STATUS_SUCCESS){
        return -EIO;
    }
    return 0;
}


int zl380xx_add_controls(struct snd_soc_codec *codec)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0))
    return snd_soc_add_controls(codec, zl380xx_snd_controls,
                                ARRAY_SIZE(zl380xx_snd_controls));
#else
    return snd_soc_add_codec_controls(codec, zl380xx_snd_controls,
                                        ARRAY_SIZE(zl380xx_snd_controls));
#endif
}

static int zl380xx_codec_probe(struct snd_soc_codec *codec)
{
    printk(KERN_INFO"Probing zl380tw SoC CODEC driver\n");
    hbi_status_t status;
    hbi_dev_cfg_t cfg;
    
    if(zl380xx_add_controls(codec) < 0)
    {
        return -1;
    }

    status=HBI_init(NULL);
    if(status != HBI_STATUS_SUCCESS)
    {
        printk(KERN_ERR"Error in HBI_init()\n");
        return -1;
    }

    cfg.dev_addr=dev_addr;
    cfg.bus_num=bus_num;
    cfg.pDevName=NULL;

    status=HBI_open(&(zl380xx_priv->handle),&cfg);
    if(status != HBI_STATUS_SUCCESS)
    {
        printk(KERN_ERR"Error in HBI_open()\n");
        HBI_term();
        return -1;
    }

    return 0;
}

static int zl380xx_codec_remove(struct snd_soc_codec *codec)
{
    hbi_status_t status;
    
    status=HBI_close(zl380xx_priv->handle);
    status=HBI_term();
    
    return 0;
}
#endif
static struct snd_soc_codec_driver soc_codec_dev_zl380xx = {
#if VPROC_CODEC_MIXER_ENABLE 
    .probe =    zl380xx_codec_probe,
    .remove =   zl380xx_codec_remove,
#endif
};
EXPORT_SYMBOL(soc_codec_dev_zl380xx);

/*--------------------------------------------------------------------
 *    ALSA  SOC CODEC driver  - END
 *--------------------------------------------------------------------*/

/*IMPORTANT note: Change this controller string maching accordingly per your *.dts or *dtsi compatible definition file*/
static struct of_device_id zl380xx_of_match[] = {
    { .compatible = "ambarella,zl380snd0"},
    {},
};
MODULE_DEVICE_TABLE(of, zl380xx_of_match);

static int zl380xx_probe(struct platform_device *pdev)
{

    int err = 0;
    
    /* Allocate driver data */
    zl380xx_priv = devm_kzalloc(&pdev->dev, sizeof(*zl380xx_priv), GFP_KERNEL);
    if (zl380xx_priv == NULL)
        return -ENOMEM;

    printk(KERN_INFO"probing zl380tw device\n");
    dev_set_drvdata(&pdev->dev, zl380xx_priv);
    zl380xx_priv->dev = pdev;
    
    err = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_zl380xx, &zl380xx_dai, 1);
    if(err < 0) {
        kfree(zl380xx_priv);
        zl380xx_priv=NULL;
        printk(KERN_ERR"zl380tw I2c device not created!!!\n");
        return err;
    }

    printk(KERN_INFO"zl380xx codec device created...\n");
    return err;
}

static int zl380xx_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);
    return 0;
}

static struct platform_driver zl380xx_codec_driver = {
    .probe      = zl380xx_probe,
    .remove     = zl380xx_remove,
    .driver = {
    .name   = "zl380snd0", 
    .owner  = THIS_MODULE,
    .of_match_table = zl380xx_of_match,
    },
};

module_platform_driver(zl380xx_codec_driver);

MODULE_AUTHOR("Jean Bony <jean.bony@microsemi.com>");
MODULE_DESCRIPTION(" Microsemi Timberwolf /alsa codec driver");
MODULE_LICENSE("GPL");
