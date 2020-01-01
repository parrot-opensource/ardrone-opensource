/**
 * @file   aai_alsa.c
 * @brief  AAI alsa layer
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

#include "aai.h"
#include "aai_hw.h"

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;  /* ID for this card */

/**
 * Initialize a PCM instance
 *
 * @param dev device's pointer
 * @param aai driver data's pointer
 * @param ipcm PCM index
 *
 * @return error code
 */
static int aai_alsa_pcm_new(struct device      *dev,
                            struct card_data_t *aai,
                            int                ipcm)
{
    int err;
    aai_device_t * chan = NULL;

    /*
     * Link PCM stream to AAI channel
     */
    chan = &(aai->chans[ipcm]);
    chan->pdrvdata = aai;

    /*
     * Install device
     */
    err = snd_pcm_new(aai->card, &(chan->name[0]),
                      (char)ipcm,                           /* device index */
                      ((chan->direction&AAI_TX) == AAI_TX), /* nb of substreams for playback */
                      ((chan->direction&AAI_RX) == AAI_RX), /* nb of substreams for capture  */
                      &(aai->pcms[ipcm]));                  /* pointer to the new pcm instance */

    if (err < 0)
    {
        aai_err(dev,"%s init failed\n", chan->name);
        return err;
    }

    aai->pcms[ipcm]->private_data = chan;
    aai->pcms[ipcm]->info_flags = 0;
    strcpy(aai->pcms[ipcm]->name, chan->name);

    /*
     * Set Operators
     */
    if (chan->direction&AAI_TX)
    {
        snd_pcm_set_ops(aai->pcms[ipcm], SNDRV_PCM_STREAM_PLAYBACK, chan->ops);
    }
    else if (chan->direction&AAI_RX)
    {
        snd_pcm_set_ops(aai->pcms[ipcm], SNDRV_PCM_STREAM_CAPTURE, chan->ops);
    }
    else
    {
        aai_err(dev,"%s init failed\n", chan->name);
        return -EINVAL;
    }

    snd_pcm_lib_preallocate_pages_for_all(aai->pcms[ipcm],
                                          SNDRV_DMA_TYPE_CONTINUOUS,
                                          snd_dma_continuous_data(GFP_KERNEL),
                                          0,
                                          64*1024); /* XXX where is it freed ? */

    return 0;
}

/**
 * @brief Create an ALSA card instance
 *
 * @param pdev  platform device
 *
 * @return AAI sound card instance
 */
struct snd_card * __devinit aai_alsa_probe(struct platform_device *pdev)
{
    struct snd_card *card;
    struct card_data_t *aai;
    int32_t idx;
    int err;
    int dev = pdev->id;

    /*
     * Initialize Card
     */
    card = snd_card_new(index[dev], id[dev], THIS_MODULE, sizeof(struct card_data_t));
    if (card == NULL)
    {
        aai_err(&pdev->dev, "card creation failed\n");
        return NULL;
    }

    aai = card->private_data;
    aai->card = card;
    aai->dev = &pdev->dev;

    /*
     * Init card for specific target, p6 or p6i
     */
    if (parrot_chip_is_p6())
    {
        err = aai_init_card_p6(&pdev->dev, aai, dev);
    }
    else
    {
        err = aai_init_card_p6i(&pdev->dev, aai, dev);
    }
    if (err < 0)
    {
        aai_err(&pdev->dev,"card init failed: err=%d\n", err);
        goto __nodev;
    }

    /*
     * alloc pcms table
     */
    aai->pcms = kmalloc(aai->pcms_cnt * sizeof(struct snd_pcm *), GFP_KERNEL);

    /*
     * Initialize PCM streams
     */
    for (idx = 0; idx < aai->pcms_cnt; idx++)
    {
        if ((err = aai_alsa_pcm_new(&pdev->dev, aai, idx)) < 0)
        {
            aai_err(&pdev->dev, "%s pcm %d init failed\n", __FUNCTION__, idx);
            goto __nodev;
        }
    }
    aai_dbg(&pdev->dev, "pcm streams initialization ok - %d streams\n", idx);

    /*
     * Register sound card for Alsa
     */
    snd_card_set_dev(card, &pdev->dev);
    err = snd_card_register(card);
    if (err != 0)
    {
        aai_err(&pdev->dev,"card registration failed: err=%d\n", err);
        goto __nodev;
    }

    aai_info(&pdev->dev,"%s card initialized\n", card->longname);
    return card;

__nodev:
    aai_err(&pdev->dev,"AAI card initialization failed \n");
    snd_card_free(card);
    return NULL;
}

/**
 * @brief Remove sound card
 *
 * @param card  card to remove
 *
 * @return error code
 */
int __devexit aai_alsa_remove(struct snd_card *card)
{
    struct card_data_t *aai = card->private_data;

    if (aai->pcms)
        kfree(aai->pcms);

    if (card)
        snd_card_free(card);

    return 0;
}

