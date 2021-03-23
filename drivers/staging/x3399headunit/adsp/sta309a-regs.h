/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for STA309A audio processor on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#ifndef _STA309A_REGS_H_
#define _STA309A_REGS_H_

/* Configuration register A */
#define STA309A_REG_CONFA                   0x00
#   define STA309A_CONFA_MCS_0              (1 << 0)        /* Master clock select */
#   define STA309A_CONFA_MCS_1              (1 << 1)
#   define STA309A_CONFA_MCS_2              (1 << 2)
#   define STA309A_CONFA_IR_0               (1 << 3)        /* Interpolation ratio bits */
#   define STA309A_CONFA_IR_1               (1 << 4)
#   define STA309A_CONFA_DSP_BYPASS         (1 << 5)        /* DSP bypass of biquad and bass/treble functions */
#   define STA309A_CONFA_COS_PLL_DIV1       (0 << 6)        /* CKOUT frequency: PLL output/1 */
#   define STA309A_CONFA_COS_PLL_DIV4       (1 << 6)        /* PLL output/4 */
#   define STA309A_CONFA_COS_PLL_DIV8       (2 << 6)        /* PLL output/8 */
#   define STA309A_CONFA_COS_PLL_DIV16      (3 << 6)        /* PLL output/16 */

/* Configuration register B: serial input formats */
#define STA309A_REG_CONFB                   0x01
#   define STA309A_CONFB_SAI_MASK           (0x0f)
#   define STA309A_CONFB_SAI_0              (1 << 0)        /* Serial audio input interface format */
#   define STA309A_CONFB_SAI_1              (1 << 1)
#   define STA309A_CONFB_SAI_2              (1 << 2)
#   define STA309A_CONFB_SAI_3              (1 << 3)
#   define STA309A_CONFB_SAIFB_MSB          (0 < 4)         /* Determines MSB or LSB first for all SAI formats: MSB first */
#   define STA309A_CONFB_SAIFB_LSB          (1 < 4)         /* LSB first */

/* Configuration register C: serial output formats */
#define STA309A_REG_CONFC                   0x02
#   define STA309A_CONFC_SAO_MASK           (0x0f)
#   define STA309A_CONFC_SAO_0              (1 << 0)        /* Serial audio output interface format */
#   define STA309A_CONFC_SAO_1              (1 << 1)
#   define STA309A_CONFC_SAO_2              (1 << 2)
#   define STA309A_CONFC_SAO_3              (1 << 3)
#   define STA309A_CONFC_SAOFB_MSB          (0 < 4)         /* Determines MSB or LSB first for all SAI formats: MSB first */
#   define STA309A_CONFC_SAOFB_LSB          (1 < 4)         /* LSB first */

/* Configuration register D: DDX */
#define STA309A_REG_CONFD                   0x03
#   define STA309A_CONFD_OM_0               (1 << 0)        /* DDX power output mode */
#   define STA309A_CONFD_OM_1               (1 << 1)
#   define STA309A_CONFD_CSZ_0              (1 << 2)        /* Contra size */
#   define STA309A_CONFD_CSZ_1              (1 << 3)
#   define STA309A_CONFD_CSZ_2              (1 << 4)
#   define STA309A_CONFD_CSZ_3              (1 << 5)
#   define STA309A_CONFD_CSZ_4              (1 << 6)
#   define STA309A_CONFD_MPC                (1 << 7)        /* Enable STA50x correction for THD reduction near maximum power output. */

/* Configuration register E: DDX */
#define STA309A_REG_CONFE                   0x04
#   define STA309A_CONFE_C1BO               (1 << 0)        /* Channels 1 binary output */
#   define STA309A_CONFE_C2BO               (1 << 1)        /* Channels 2 binary output */
#   define STA309A_CONFE_C3BO               (1 << 2)        /* Channels 3 binary output */
#   define STA309A_CONFE_C4BO               (1 << 3)        /* Channels 4 binary output */
#   define STA309A_CONFE_C5BO               (1 << 4)        /* Channels 5 binary output */
#   define STA309A_CONFE_C6BO               (1 << 5)        /* Channels 6 binary output */
#   define STA309A_CONFE_C7BO               (1 << 6)        /* Channels 7 binary output */
#   define STA309A_CONFE_C8BO               (1 << 7)        /* Channels 8 binary output */

/* Configuration register F  */
#define STA309A_REG_CONFF                   0x05
#   define STA309A_CONFF_HPB                (1 << 0)        /* High-pass filter bypass bit */
#   define STA309A_CONFF_DRC                (1 << 1)        /* Dynamic range compression/anti-clipping */
#   define STA309A_CONFF_DE_EMPHASIS        (1 << 2)        /* Enable de-emphasis */
#   define STA309A_CONFF_PSL                (1 << 3)        /* Postscale link */
#   define STA309A_CONFF_BQL                (1 << 4)        /* Biquad link */
#   define STA309A_CONFF_PWMS_0             (1 << 5)        /* PWM speed selection */
#   define STA309A_CONFF_PWMS_1             (1 << 6)
#   define STA309A_CONFF_PWMS_2             (1 << 7)
#   define STA309A_CONFF_PWMS_NORMAL        (0 << 5)        /* Normal Speed (384kHz) All Channels */
#   define STA309A_CONFF_PWMS_HALF          (1 << 5)        /* Half Speed (192kHz) All Channels */
#   define STA309A_CONFF_PWMS_DOUBLE        (2 << 5)        /* Double Speed (768kHz) All Channels */
#   define STA309A_CONFF_PWMS_MIXED         (3 << 5)        /* Normal Speed Channels 1-6, Double Speed Channels 7-8 */
#   define STA309A_CONFF_PWMS_ODD           (4 << 5)        /* Odd Speed (341.3kHz) All Channels */

/* Configuration register G */
#define STA309A_REG_CONFG                   0x06
#   define STA309A_CONFG_PWM_DIS            (1 << 0)        /* no PWM output */
#   define STA309A_CONFG_I2S_DIS            (1 << 1)        /* no I2S output */
#   define STA309A_CONFG_CLKO_DIS           (1 << 2)        /* no clock output */
#   define STA309A_CONFG_AM_ENA             (1 << 3)        /* AM reduction mode DDX operation */
#   define STA309A_CONFG_AM2_ENA            (1 << 4)        /* AM2 reduction mode DDX operation */
#   define STA309A_CONFG_HP_ENA             (1 << 5)        /* Switch channels 7 and 8 to headphone operation */
#   define STA309A_CONFG_DCCV               (1 << 6)        /* uses DCC coefficient */
#   define STA309A_CONFG_MPCV               (1 << 7)        /* use MPCC bits for MPC coefficient */

/* Configuration register H */
#define STA309A_REG_CONFH                   0x07
#   define STA309A_CONFH_NSBW               (1 << 0)        /* Noise-shaper bandwidth selection: 3rd order NS */
#   define STA309A_CONFH_ZC_ENA             (1 << 1)        /* Zero-crossing volume enable */
#   define STA309A_CONFH_SV_ENA             (1 << 2)        /* Soft volume enable */
#   define STA309A_CONFH_ZD_ENA             (1 << 3)        /* Zero-detect mute enable */
#   define STA309A_CONFH_ID_ENA             (1 << 4)        /* Invalid input detect mute enable */
#   define STA309A_CONFH_BCL_ENA            (1 << 5)        /* Binary output mode clock loss detection enable */
#   define STA309A_CONFH_LDT_ENA            (1 << 6)        /* LRCLK double trigger protection enable */
#   define STA309A_CONFH_ECL_ENA            (1 << 7)        /* Auto EAPD on clock loss */

/* Configuration register I */
#define STA309A_REG_CONFI                   0x08
#   define STA309A_CONFI_PSCE               (1 << 0)        /* Power supply ripple correction enable */
#   define STA309A_CONFI_EAPD               (1 << 7)        /* External amplifier power down: Normal Operation */

/* Master mute register */
#define STA309A_REG_MASTER_MUTE             0x09
#   define STA309A_CONFMM_ENA               (1 << 0)        /* Master mute enable */

/* Master volume register */
#define STA309A_REG_MASTER_VOL              0x0A
#   define STA309A_MVOL_MASK                (0xfe)          /* 0 dB <-> -79.5 dB */
#   define STA309A_MVOL_MUTE                (0xff)          /* Hardware channel mute */

/* Channel X volume */
#define STA309A_REG_CH_1_VOL                0x0B
#define STA309A_REG_CH_2_VOL                0x0C
#define STA309A_REG_CH_3_VOL                0x0D
#define STA309A_REG_CH_4_VOL                0x0E
#define STA309A_REG_CH_5_VOL                0x0F
#define STA309A_REG_CH_6_VOL                0x10
#define STA309A_REG_CH_7_VOL                0x11
#define STA309A_REG_CH_8_VOL                0x12
#   define STA309A_CHXVOL_MASK              (0xfe)          /* +48 dB <-> -79.5 dB */
#   define STA309A_CHXVOL_MUTE              (0xff)          /* Hardware channel mute */

/* Channel X Volume trim, mute, bypass */
#define STA309A_REG_CH_1_VTMB               0x13
#define STA309A_REG_CH_2_VTMB               0x14
#define STA309A_REG_CH_3_VTMB               0x15
#define STA309A_REG_CH_4_VTMB               0x16
#define STA309A_REG_CH_5_VTMB               0x17
#define STA309A_REG_CH_6_VTMB               0x18
#define STA309A_REG_CH_7_VTMB               0x19
#define STA309A_REG_CH_8_VTMB               0x1A
#   define STA309A_CHVTMB_VOL_TRIM_MASK     (0x1f)          /* +10 dB <-> -10 dB */
#   define STA309A_CHVTMB_MVOL_BYPASS       (1 << 6)        /* Master volume bypass */
#   define STA309A_CHVTMB_MUTE              (1 << 7)        /* Soft mute is performed on that channel */


/* Channel input mapping channels */
#define STA309A_REG_CH1_CH2_IM              0x1B
#define STA309A_REG_CH3_CH4_IM              0x1C
#define STA309A_REG_CH5_CH6_IM              0x1D
#define STA309A_REG_CH7_CH8_IM              0x1E
#   define STA309A_IM_CH1_TO(x)             ((x) & 7)        /* Map channel 1 to IN_CH_x channel */
#   define STA309A_IM_CH2_TO(x)             (((x) & 7) << 4) /* Map channel 2 to IN_CH_x channel */
#   define STA309A_IM_CH3_TO(x)             ((x) & 7)        /* Map channel 3 to IN_CH_x channel */
#   define STA309A_IM_CH4_TO(x)             (((x) & 7) << 4) /* Map channel 4 to IN_CH_x channel */
#   define STA309A_IM_CH5_TO(x)             ((x) & 7)        /* Map channel 5 to IN_CH_x channel */
#   define STA309A_IM_CH6_TO(x)             (((x) & 7) << 4) /* Map channel 6 to IN_CH_x channel */
#   define STA309A_IM_CH7_TO(x)             ((x) & 7)        /* Map channel 7 to IN_CH_x channel */
#   define STA309A_IM_CH8_TO(x)             (((x) & 7) << 4) /* Map channel 8 to IN_CH_x channel */
#   define STA309A_IN_CH_1                  (0x0)            /* Serial input from: channel 1 */
#   define STA309A_IN_CH_2                  (0x1)            /* Serial input from: channel 2 */
#   define STA309A_IN_CH_3                  (0x2)            /* Serial input from: channel 3 */
#   define STA309A_IN_CH_4                  (0x3)            /* Serial input from: channel 4 */
#   define STA309A_IN_CH_5                  (0x4)            /* Serial input from: channel 5 */
#   define STA309A_IN_CH_6                  (0x5)            /* Serial input from: channel 6 */
#   define STA309A_IN_CH_7                  (0x6)            /* Serial input from: channel 7 */
#   define STA309A_IN_CH_8                  (0x7)            /* Serial input from: channel 8 */


/* AUTO1 - Automode(tm) EQ, volume, GC */
#define STA309A_REG_AUTO1                   0x1F
#   define STA309A_AUTO2_AMEQ_0             (1 << 0)        /* Biquad 2-6 mode */
#   define STA309A_AUTO2_AMEQ_1             (1 << 1)
#   define STA309A_AUTO2_AMEQ_USER          (0x0)           /* Biquad 2-6 mode is user programmable */
#   define STA309A_AUTO2_AMEQ_PRESET_EQ     (0x1)           /* Biquad 2-6 mode is preset EQ - PEQ bits */
#   define STA309A_AUTO2_AMEQ_GRAPHICS_EQ   (0x2)           /* Biquad 2-6 mode is graphic EQ - xGEQ bits */
#   define STA309A_AUTO2_AMEQ_AUTO_VOL      (0x3)           /* Biquad 2-6 mode is auto volume controlled loudness curve */
#   define STA309A_AUTO2_AMV_0              (1 << 2)        /* Automode volume mode (MVOL) */
#   define STA309A_AUTO2_AMV_1              (1 << 3)
#   define STA309A_AUTO2_AMGC_0             (1 << 4)        /* Automode gain compression/limiters mode */
#   define STA309A_AUTO2_AMGC_1             (1 << 5)
#   define STA309A_AUTO2_AMGC_2             (1 << 6)
#   define STA309A_AUTO2_AMDM               (1 << 7)        /* Automode 5.1 downmix */


/* AUTO2 - Automode(tm) bass management2 */
#define STA309A_REG_AUTO2                   0x20
#   define STA309A_AUTO2_AMBMM_ENA          (1 << 0)        /* Automode bass management mix enabled */
#   define STA309A_AUTO2_AMBMX_ENA          (1 << 1)        /* Automode bass management crossover enabled*/
#   define STA309A_AUTO2_FSS_SMALL          (0 << 2)        /* front speaker size: small */
#   define STA309A_AUTO2_FSS_LARGE          (1 << 2)        /* front speaker size: large */
#   define STA309A_AUTO2_CSS_SMALL          (0 << 3)        /* center speaker size: small */
#   define STA309A_AUTO2_CSS_LARGE          (1 << 3)        /* center speaker size: large */
#   define STA309A_AUTO2_CSS_OFF            (2 << 3)        /* center speaker size not applicable */
#   define STA309A_AUTO2_RSS_SMALL          (0 << 5)        /* rear speaker size: small */
#   define STA309A_AUTO2_RSS_LARGE          (1 << 5)        /* rear speaker size: large */
#   define STA309A_AUTO2_RSS_OFF            (2 << 5)        /* rear speaker size not applicable */
#   define STA309A_AUTO2_SUB_ENA            (1 << 7)        /* subwoofer enable */
#   define STA309A_AUTO2_SUB_OFF            (0 << 7)        /* subwoofer disable */

/* AUTO3 - Automode(tm) AM/prescale/bass management scale */
#define STA309A_REG_AUTO3                   0x21

/* PREEQ - Preset EQ settings */
#define STA309A_REG_PREEQ                   0x22
#   define STA309A_PREEQ_PEQ_0              (1 << 0)        /* PEQ Mode / setting */
#   define STA309A_PREEQ_PEQ_1              (1 << 1)
#   define STA309A_PREEQ_PEQ_2              (1 << 2)
#   define STA309A_PREEQ_PEQ_3              (1 << 3)
#   define STA309A_PREEQ_PEQ_4              (1 << 4)
#   define STA309A_PREEQ_PEQ_FLAT           (0)
#   define STA309A_PREEQ_PEQ_ROCK           (1)
#   define STA309A_PREEQ_PEQ_SOFT_ROCK      (2)
#   define STA309A_PREEQ_PEQ_JAZZ           (3)
#   define STA309A_PREEQ_PEQ_CLASSICAL      (4)
#   define STA309A_PREEQ_PEQ_DANCE          (5)
#   define STA309A_PREEQ_PEQ_POP            (6)
#   define STA309A_PREEQ_PEQ_SOFT           (7)
#   define STA309A_PREEQ_PEQ_HARD           (8)
#   define STA309A_PREEQ_PEQ_PARTY          (9)
#   define STA309A_PREEQ_PEQ_VOCAL          (10)
#   define STA309A_PREEQ_PEQ_HIPHOP         (11)
#   define STA309A_PREEQ_PEQ_DIALOG         (12)
#   define STA309A_PREEQ_PEQ_BASS_BOOST1    (13)
#   define STA309A_PREEQ_PEQ_BASS_BOOST2    (14)
#   define STA309A_PREEQ_PEQ_BASS_BOOST3    (15)
#   define STA309A_PREEQ_PEQ_LOUDNESS1      (16)
/* TODO: To add loudness levels here */
#   define STA309A_PREEQ_PEQ_LOUDNESS16     (31)
#   define STA309A_PREEQ_XO_0               (1 << 5)        /* Bass management crossover frequency */
#   define STA309A_PREEQ_XO_1               (1 << 6)
#   define STA309A_PREEQ_XO_2               (1 << 7)
#   define STA309A_PREEQ_XO_70HZ            (0 << 5)        /* 70 Hz */
#   define STA309A_PREEQ_XO_80HZ            (1 << 5)        /* 80 Hz */
#   define STA309A_PREEQ_XO_90HZ            (2 << 5)        /* 90 Hz */
#   define STA309A_PREEQ_XO_100HZ           (3 << 5)        /* 100 Hz */
#   define STA309A_PREEQ_XO_110HZ           (4 << 5)        /* 110 Hz */
#   define STA309A_PREEQ_XO_120HZ           (5 << 5)        /* 120 Hz */
#   define STA309A_PREEQ_XO_140HZ           (6 << 5)        /* 140 Hz */
#   define STA309A_PREEQ_XO_160HZ           (7 << 5)        /* 160 Hz */

/* AGEQ - graphic EQ 80-Hz band */
#define STA309A_REG_AGEQ                    0x23
/* BGEQ - graphic EQ 300-Hz band */
#define STA309A_REG_BGEQ                    0x24
/* CGEQ - graphic EQ 1-kHz band */
#define STA309A_REG_CGEQ                    0x25
/* DGEQ - graphic EQ 3-kHz band */
#define STA309A_REG_DGEQ                    0x26
/* EGEQ - graphic EQ 8-kHz band */
#define STA309A_REG_EGEQ                    0x27
#   define STA309A_xGEQ_MASK                (0x1f)          /* Boost/cut: +16 <-> -15 */

/* Processing loop registers */
/*------------------------------------*/
#define STA309A_REG_BQLP                    0x28
#define STA309A_REG_MXLP                    0x29

/* EQ bypass */
#define STA309A_REG_EQBP                    0x2A
#   define STA309A_EQBP_CH_1                (1 << 0)        /* Bypass EQ on channel 1 */
#   define STA309A_EQBP_CH_2                (1 << 1)        /* channel 2 */
#   define STA309A_EQBP_CH_3                (1 << 2)        /* channel 3 */
#   define STA309A_EQBP_CH_4                (1 << 3)        /* channel 4 */
#   define STA309A_EQBP_CH_5                (1 << 4)        /* channel 5 */
#   define STA309A_EQBP_CH_6                (1 << 5)        /* channel 6 */
#   define STA309A_EQBP_CH_7                (1 << 6)        /* channel 7 */
#   define STA309A_EQBP_CH_8                (1 << 7)        /* channel 8 */

/* Tone control bypass */
#define STA309A_REG_TONEBP                  0x2B
#   define STA309A_TONEBP_CH_1              (1 << 0)        /* Tone control (bass/treble) on channel 1 */
#   define STA309A_TONEBP_CH_2              (1 << 1)        /* channel 2 */
#   define STA309A_TONEBP_CH_3              (1 << 2)        /* channel 3 */
#   define STA309A_TONEBP_CH_4              (1 << 3)        /* channel 4 */
#   define STA309A_TONEBP_CH_5              (1 << 4)        /* channel 5 */
#   define STA309A_TONEBP_CH_6              (1 << 5)        /* channel 6 */
#   define STA309A_TONEBP_CH_7              (1 << 6)        /* channel 7 */
#   define STA309A_TONEBP_CH_8              (1 << 7)        /* channel 8 */

/* Tone control bass/treble */
#define STA309A_REG_TONE                    0x2C
#    define STA309A_TONE_BTC_VAL(x)         ((x) & 0xf)     /* Bass: +12dB <-> -12 dB */
#    define STA309A_TONE_TTC_VAL(x)         ((x) << 4)      /* Treble: +12dB <-> -12 dB */

/* Dynamics control registers */
/*------------------------------------*/
#define STA309A_REG_C1234LS                 0x2D
#define STA309A_REG_C5678LS                 0x2E
#define STA309A_REG_L1AR                    0x2F
#define STA309A_REG_L1ATRT                  0x30
#define STA309A_REG_L2AR                    0x31
#define STA309A_REG_L2ATRT                  0x32

/* PWM output timing registers */
/*------------------------------------*/
#define STA309A_REG_C12OT                   0x33
#define STA309A_REG_C34OT                   0x34
#define STA309A_REG_C56OT                   0x35
#define STA309A_REG_C78OT                   0x36

/* I2S output channel mapping registers */
#define STA309A_REG_CH1_CH2_OM              0x37
#define STA309A_REG_CH3_CH4_OM              0x38
#define STA309A_REG_CH5_CH6_OM              0x39
#define STA309A_REG_CH7_CH8_OM              0x3A
#   define STA309A_OM_CH1_TO(x)             (((x) & 7) << 4) /* Map channel 1 to IN_CH_x channel */
#   define STA309A_OM_CH2_TO(x)             ((x) & 7)        /* Map channel 2 to IN_CH_x channel */
#   define STA309A_OM_CH3_TO(x)             (((x) & 7) << 4) /* Map channel 3 to IN_CH_x channel */
#   define STA309A_OM_CH4_TO(x)             ((x) & 7)        /* Map channel 4 to IN_CH_x channel */
#   define STA309A_OM_CH5_TO(x)             (((x) & 7) << 4) /* Map channel 5 to IN_CH_x channel */
#   define STA309A_OM_CH6_TO(x)             ((x) & 7)        /* Map channel 6 to IN_CH_x channel */
#   define STA309A_OM_CH7_TO(x)             (((x) & 7) << 4) /* Map channel 7 to IN_CH_x channel */
#   define STA309A_OM_CH8_TO(x)             ((x) & 7)        /* Map channel 8 to IN_CH_x channel */
#   define STA309A_OUT_CH_1                 (0x0)            /* Serial output from: channel 1 */
#   define STA309A_OUT_CH_2                 (0x1)            /* Serial output from: channel 2 */
#   define STA309A_OUT_CH_3                 (0x2)            /* Serial output from: channel 3 */
#   define STA309A_OUT_CH_4                 (0x3)            /* Serial output from: channel 4 */
#   define STA309A_OUT_CH_5                 (0x4)            /* Serial output from: channel 5 */
#   define STA309A_OUT_CH_6                 (0x5)            /* Serial output from: channel 6 */
#   define STA309A_OUT_CH_7                 (0x6)            /* Serial output from: channel 7 */
#   define STA309A_OUT_CH_8                 (0x7)            /* Serial output from: channel 8 */

/* ------------------------------------------------------------------ */
#define STA309A_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
                     SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
                     SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
                     SNDRV_PCM_RATE_192000)

#define STA309A_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                         SNDRV_PCM_FMTBIT_S24_LE)

#endif /* _STA309A_REGS_H_ */
