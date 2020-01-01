/**
 * @file   aai_module.c
 * @brief  AAI kernel module layer
 *
 * @author gregoire.etienne@parrot.com
 * @date   2008-10-02
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#include <asm/fiq.h>

#include "aai.h"
#include "aai_hw.h"

/*
 * Set the AAI software interrupt to MOST
 * irq, since it's not used elsewhere.
 */
#define AAI_SOFT_INTERRUPT  3

static struct fiq_handler fh =
{
    .name = "aai_fiq"
};

/**
* @brief Initialize AAI module
*
* @param pdev  platform device
*
* @return error code
**/
static int aai_module_probe(struct platform_device *pdev)
{
    struct snd_card *card = NULL;
    struct card_data_t *aai_data = NULL;
    struct resource *res;
    int ret;
    extern unsigned char aai_fiq_handler_start, aai_fiq_handler_end;
    struct pt_regs regs;

    /*
     * Map registers
     */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL)
    {
        aai_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
        ret = -ENOENT;
        goto no_iores;
    }

    /*
     * Initialize driver structures
     */
    card = aai_alsa_probe(pdev);
    if (!card)
    {
        ret = -ENXIO;
        goto no_iores;
    }

    aai_data = card->private_data;

    platform_set_drvdata(pdev, card);
    aai_dbg(&pdev->dev, "Card @ %p, driver data struct @ %p\n", card, card->private_data);

    /*
     * Initialize IO
     */
    aai_data->ioarea = request_mem_region(res->start,
                                          res->end - res->start + 1,
                                          pdev->name);
    if (aai_data->ioarea == NULL)
    {
        aai_err(&pdev->dev, "Cannot reserve IO region\n");
        ret = -ENXIO;
        goto no_iores;
    }

    aai_data->iobase = ioremap(res->start, res->end - res->start + 1);
    if (aai_data->iobase == NULL)
    {
        aai_err(&pdev->dev, "Cannot map IO\n");
        ret = -ENXIO;
        goto no_iomap;
    }
    else
    {
        aai_dbg(&pdev->dev, "IO region [0x%08x-0x%08x] mapped to %p\n",
                            res->start, res->end, aai_data->iobase);
    }

    /*
     * AAI Hardware Init
     */
    ret = aai_hw_init(aai_data);
    if (ret != 0)
    {
        aai_err(&pdev->dev, "Hardware init failed \n");
        ret = -EINVAL;
        goto no_irq;
    }

    /*
     * Install FIQ handler
     */
    if (claim_fiq(&fh) < 0)
    {
        aai_err(&pdev->dev, "Couldn't claim fiq handler\n");
        ret = -EBUSY;
        goto no_irq;
    }

    set_fiq_handler(&aai_fiq_handler_start,
            &aai_fiq_handler_end - &aai_fiq_handler_start);

    /*
     * Set registers useful in FIQ process
     */
    regs.ARM_r8 = (unsigned long)aai_data->chans;
    regs.ARM_r9 = (unsigned long)aai_data->pcms_cnt;
    regs.ARM_r10 = sizeof(aai_device_t);
    regs.ARM_fp = (unsigned long)aai_data->iobase;
    regs.ARM_sp = (unsigned long)(&aai_fiq_handler_end - 4 - &aai_fiq_handler_start + 0xffff001c);
    set_fiq_regs(&regs);

    /*
     * Retrieve interruption number
     */
    aai_data->fiq = platform_get_irq(pdev, 0);
    if (aai_data->fiq < 0)
    {
        aai_err(&pdev->dev, "No IRQ number \n");
        ret = -ENOENT;
        goto no_fiq;
    }

    /*
     * Set FIQ in VIC register
     */
    writel(1 << aai_data->fiq, PARROT6_VA_VIC + VIC_INT_SELECT);

    /*
     * Enable FIQ
     */
    enable_fiq(aai_data->fiq);

    /*
     * Install IRQ handler
     */
    aai_data->irq = AAI_SOFT_INTERRUPT;  /* MOST IRQ, unused elsewhere */
    ret = request_irq(aai_data->irq, aai_data->irq_handler, IRQF_DISABLED, pdev->name, aai_data);
    if (!ret)
    {
        aai_dbg(&pdev->dev, "IRQ %d attached\n", aai_data->irq);
    }
    else
    {
        aai_err(&pdev->dev, "failed attaching IRQ %d\n", aai_data->irq);
    }

    aai_dbg(&pdev->dev, "module initialized\n");
    return ret;

no_fiq:
    release_fiq(&fh);
no_irq:
    iounmap(aai_data->iobase);
no_iomap:
    release_resource(aai_data->ioarea);
    kfree(aai_data->ioarea);
no_iores:
    platform_set_drvdata(pdev, NULL);
    if (aai_data)
        snd_card_free(aai_data->card);
    return ret;
}

/**
* @brief Remove AAI module
*
* @param pdev  platform device
*
* @return error code
**/
static int aai_module_remove(struct platform_device *pdev)
{
    struct snd_card *card = platform_get_drvdata(pdev);
    struct card_data_t *aai = card->private_data;

    /*
     * reset hardware layer
     */
    aai_hw_remove(aai);

    /*
     * Release FIQ
     */
    disable_fiq(aai->irq);
    release_fiq(&fh);

    /*free_irq(aai->irq, aai);*/

    /*
     * beware, ths call will free drv_data which is included
     * into card structure.
     */
    aai_alsa_remove(card);

    iounmap(aai->iobase);
    release_resource(aai->ioarea);
    kfree(aai->ioarea);

    platform_set_drvdata(pdev, NULL);
    aai_dbg(&pdev->dev, "module removed \n");

    return 0;
}

static int aai_module_suspend(struct platform_device *pdev, pm_message_t msg)
{
    aai_err(&pdev->dev, "%s\n", __FUNCTION__);
    return 0;
}

static int aai_module_resume(struct platform_device *pdev)
{
    aai_err(&pdev->dev, "%s\n", __FUNCTION__);
    return 0;
}

static struct platform_driver aai_module_driver =
{
    .probe   = aai_module_probe,
    .remove  = aai_module_remove,
    .suspend = aai_module_suspend,
    .resume  = aai_module_resume,
    .driver  =
    {
        .name  = "aai",
        .owner = THIS_MODULE,
    },
};

static int __init aai_module_init(void)
{
    return platform_driver_register(&aai_module_driver);
}

static void __exit aai_module_exit(void)
{
    platform_driver_unregister(&aai_module_driver);
}

module_init(aai_module_init);
module_exit(aai_module_exit);

MODULE_DESCRIPTION("Parrot6 Advanced Audio Interface driver");
MODULE_AUTHOR("Gregoire ETIENNE, <gregoire.etienne@parrot.com>");
MODULE_LICENSE("GPL");

