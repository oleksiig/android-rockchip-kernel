/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for Si468x AM/FM/DAB Radio Receiver on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#ifndef __SI468X_H__
#define __SI468X_H__

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>


#define DRIVER_NAME "si468x-radio"
#define DRIVER_CARD "SI468x AM/FM/DAB Receiver"

struct si468x_band_desc {
    uint32_t rangelow_khz;
    uint32_t rangehigh_khz;

#define     RADIO_MODULATION_FM   0
#define     RADIO_MODULATION_AM   1
#define     RADIO_MODULATION_DAB  2
    uint8_t modulation;

#define     RADIO_DEEMPHASIS_D50  0
#define     RADIO_DEEMPHASIS_D75  1
    uint8_t deemphasis;
};

struct si468x_rsq_status {
    uint32_t freq_khz;          /* Currently tuned frequency */
    uint8_t freq_offset_ppm;    /* Signed frequency offset in PPM –128 to 127 */
    int8_t rssi_dbuv;           /* Received signal strength indicator in dBµV –128 to 127 */
    int8_t snr_db;              /* RF SNR indicator in dB –128 to 127 */
    uint16_t ant_cap;           /* Antenna tuning cap value */
    uint8_t multipath_ind;      /* Multipath indicator 0–255 */
};

struct si468x_rds_status {
    uint16_t program_id;        /* Current channel's Program Identification */
    uint8_t pty_code;           /* Current channel's PTY code */
    uint8_t tp;                 /* Traffic programm flag */
    char ps_name[9];            /* Programme service name */
    char radiotext[129];

    /* RDS private flags */
    uint16_t group_0a_flags;
    uint32_t group_2a_flags;
};

struct si468x_acf_status {
    uint8_t pilot;              /* Stereo pilot indicator */
    uint8_t stereo_level;       /* Stereo separation in percent */
    uint8_t blend_state;        /* Blended between max stereo and min stereo separation */
};

struct si468x_tune_status
{
#define TUNE_STATUS_FLAG_STC    (1 << 0)
#define TUNE_STATUS_FLAG_RDS    (1 << 1)
#define TUNE_STATUS_FLAG_RSQ    (1 << 2)
#define TUNE_STATUS_FLAG_ACF    (1 << 3)

    uint8_t flags;
    /* Current statio tune info if TUNE_STATUS_FLAG_STC is set */
    struct si468x_rsq_status rsq;
    /* RDS info, if TUNE_STATUS_FLAG_RDS flag is set */
    struct si468x_rds_status rds;
    /* ACF info, if TUNE_STATUS_FLAG_ACF flag is set */
    struct si468x_acf_status acf;
};

/*
 * si468x_device - private data
 */
struct si468x_device {
	struct v4l2_device v4l2_dev;
	struct video_device videodev;

    uint8_t mode;
    uint8_t irq_flags;

    struct mutex i2c_lock;
    struct mutex irq_flags_lock;

    wait_queue_head_t status_queue;

	/* si468x ops */
    int (*si468x_xfer)(struct si468x_device *radio,
        void *write_ptr, size_t write_sz, void *read_ptr, size_t read_sz);

	int (*fops_open)(struct file *file);
	int (*fops_release)(struct file *file);

    /* */
    struct si468x_tune_status status;

#if IS_ENABLED(CONFIG_X3399HEADUNIT_SI468X_I2C)
	struct i2c_client *client;
	struct gpio_desc *gpio_reset;
#endif
};

/* ------------------------------------------------------------------ */
#define SI468X_MODE_UNKNOWN		                0
#define SI468X_MODE_BOOT	                    1
#define SI468X_MODE_AM		                    2
#define SI468X_MODE_FM		                    3
#define SI468X_MODE_DAB		                    4

#define SI468X_FLASH_OFFSET_PATCH               0x00002000
#define SI468X_FLASH_OFFSET_FM                  0x00006000
#define SI468X_FLASH_OFFSET_DAB                 0x00086000
#define SI468X_FLASH_OFFSET_AM		            0x00106000

#define SI468X_RD_REPLY                         0x00
#define SI468X_POWER_UP                         0x01
#define SI468X_HOST_LOAD                        0x04
#define SI468X_FLASH_LOAD                       0x05
#define SI468X_LOAD_INIT                        0x06
#define SI468X_BOOT                             0x07

#define SI468X_GET_PART_INFO                    0x08
#define SI468X_GET_SYS_STATE                    0x09
#define SI468X_SET_PROPERTY                     0x13
#define SI468X_GET_PROPERTY                     0x14

#define SI468X_FM_TUNE_FREQ                     0x30
#define SI468X_FM_SEEK_START                    0x31
#define SI468X_FM_RSQ_STATUS                    0x32
#define SI468X_FM_ACF_STATUS                    0x33
#define SI468X_FM_RDS_STATUS                    0x34
#define SI468X_FM_RDS_BLOCKCOUNT                0x35

#define SI468X_DAB_TUNE_FREQ                    0xB0
#define SI468X_DAB_DIGRAD_STATUS                0xB2
#define SI468X_DAB_GET_SERVICE_LINKING_INFO     0xB7
#define SI468X_DAB_SET_FREQ_LIST                0xB8
#define SI468X_DAB_GET_DIGITAL_SERVICE_LIST     0x80
#define SI468X_DAB_START_DIGITAL_SERVICE        0x81
#define SI468X_DAB_GET_ENSEMBLE_INFO            0xB4
#define SI468X_DAB_GET_AUDIO_INFO               0xBD
#define SI468X_DAB_GET_SUBCHAN_INFO             0xBE

#define SI468X_AM_TUNE_FREQ                     0x40
#define SI468X_AM_SEEK_START                    0x41
#define SI468X_AM_RSQ_STATUS                    0x42

#define SI468X_INT_CTL_ENABLE                   0x0000
#define SI468X_INT_CTL_REPEAT                   0x0001

#define SI468X_AM_AVC_MIN_GAIN                  0x0500  /* The minimum gain value for the AVC. Range: -4096-3061 - -24dB to +18dB */
#define SI468X_AM_AVC_MAX_GAIN                  0x0501  /* The maximum gain value for the AVC. Range: 0-32767 - 0 to 193dB */

#define SI468X_PIN_CONFIG_ENABLE                0x0800

#define SI468X_DIGITAL_IO_OUTPUT_SELECT         0x0200
#define SI468X_DIGITAL_IO_OUTPUT_SAMPLE_RATE	0x0201
#define SI468X_DIGITAL_IO_OUTPUT_FORMAT         0x0202

#define SI468X_FM_TUNE_FE_CFG                   0x1712
#define SI468X_FM_SEEK_BAND_BOTTOM              0x3100
#define SI468X_FM_SEEK_BAND_TOP                 0x3101
#define SI468X_FM_VALID_MAX_TUNE_ERROR          0x3200
#define SI468X_FM_VALID_RSSI_TIME               0x3201
#define SI468X_FM_VALID_RSSI_THRESHOLD          0x3202
#define SI468X_FM_VALID_SNR_TIME                0x3203
#define SI468X_FM_VALID_SNR_THRESHOLD           0x3204
#define SI468X_FM_SOFTMUTE_SNR_LIMITS           0x3500
#define SI468X_FM_SOFTMUTE_SNR_ATTENUATION      0x3501
#define SI468X_FM_RSQ_INTERRUPT_SOURCE          0x3300
#define SI468X_FM_RSQ_SNR_HIGH_THRESHOLD        0x3301
#define SI468X_FM_RSQ_SNR_LOW_THRESHOLD         0x3302
#define SI468X_FM_AUDIO_DE_EMPHASIS             0x3900
#define SI468X_FM_ACF_INTERRUPT_SOURCE          0x3400
#define SI468X_FM_RDS_INTERRUPT_SOURCE          0x3C00
#define SI468X_FM_RDS_CONFIG                    0x3C02

#define SI468X_AM_CHBW_OVERRIDE_BW              0x2204
#define SI468X_AM_SEEK_BAND_BOTTOM              0x4100
#define SI468X_AM_SEEK_BAND_TOP                 0x4101
#define SI468X_AM_SEEK_FREQUENCY_SPACING        0x4102
#define SI468X_AM_VALID_RSSI_THRESHOLD          0x4202
#define SI468X_AM_VALID_SNR_THRESHOLD           0x4204


#define SI468X_DAB_TUNE_FE_CFG                  0x1712
#define SI468X_DAB_TUNE_FE_VARM                 0x1710
#define SI468X_DAB_TUNE_FE_VARB                 0x1711
#define SI468X_DAB_CTRL_DAB_MUTE_ENABLE         0xB400
#define SI468X_DAB_CTRL_DAB_MUTE_SIGNAL_LEVEL_THRESHOLD 0xB501
#define SI468X_DAB_CTRL_DAB_MUTE_SIGLOW_THRESHOLD 0xB505

#define SI468X_DIGITAL_SERVICE_INT_SOURCE       0x8100

/* 0x00=75us -> defaults to USA (default), 0x01=50us -> defaults to Europe, 0x02=disabled */
#define SI468X_AUDIO_DE_EMPHASIS_US             0x00
#define SI468X_AUDIO_DE_EMPHASIS_EU             0x01
#define SI468X_AUDIO_DE_EMPHASIS_OFF            0x02

/* */
int si468x_start_fm(struct si468x_device *radio,
                        struct si468x_band_desc *desc);
int si468x_start_am(struct si468x_device *radio,
                        struct si468x_band_desc *desc);
int si468x_start_dab(struct si468x_device *radio,
                        struct si468x_band_desc *desc);

int si468x_tune(struct si468x_device *radio,
    uint32_t freqKhz);

int si468x_seek_start(struct si468x_device *radio,
    uint8_t up, uint8_t wrap);

int si468x_read_rsq_status(struct si468x_device *radio);
int si468x_fm_read_rds_status(struct si468x_device *radio);
int si468x_fm_read_acf_status(struct si468x_device *radio);

#endif /* __SI468X_H__ */
