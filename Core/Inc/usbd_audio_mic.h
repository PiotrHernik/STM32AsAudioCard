/**
  ******************************************************************************
  * @file    usbd_audio_mic.h
  * @brief   USB Audio Class 1.0 microphone (IN) implementation.
  *          Mono, 16-bit PCM, fixed sample rate (USBD_AUDIO_MIC_FREQ).
  *          Designed as a drop-in replacement for the ST USBD_AUDIO class
  *          when the device must act as a USB microphone.
  ******************************************************************************
  */

#ifndef __USBD_AUDIO_MIC_H
#define __USBD_AUDIO_MIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/* ===== Stream configuration ===== */
#ifndef USBD_AUDIO_MIC_FREQ
#define USBD_AUDIO_MIC_FREQ                 16000U
#endif
#define USBD_AUDIO_MIC_CHANNELS             1U
#define USBD_AUDIO_MIC_BITS                 16U

/* Bytes per 1ms USB frame (one isochronous packet). */
#define AUDIO_MIC_PACKET_SIZE \
    ((USBD_AUDIO_MIC_FREQ / 1000U) * USBD_AUDIO_MIC_CHANNELS * (USBD_AUDIO_MIC_BITS / 8U))

/* Samples per packet (int16_t units). */
#define AUDIO_MIC_PACKET_SAMPLES            (AUDIO_MIC_PACKET_SIZE / 2U)

/* ===== USB topology constants ===== */
#define AUDIO_MIC_IN_EP                     0x81U
#define AUDIO_MIC_AC_INTERFACE              0x00U
#define AUDIO_MIC_AS_INTERFACE              0x01U

/* Audio Control unit IDs */
#define AUDIO_MIC_INPUT_TERMINAL_ID         0x01U
#define AUDIO_MIC_FEATURE_UNIT_ID           0x02U
#define AUDIO_MIC_OUTPUT_TERMINAL_ID        0x03U

/* Total length of the configuration descriptor. */
#define USBD_AUDIO_MIC_CONFIG_DESC_SIZ      0x6DU

/* ===== Ring buffer sizing =====
 * Holds AUDIO_MIC_RING_PACKETS worth of 1ms packets to absorb jitter between
 * the I2S producer and the USB consumer. 8 ms is plenty for 16 kHz mono.
 */
#define AUDIO_MIC_RING_PACKETS              8U
#define AUDIO_MIC_RING_SAMPLES              (AUDIO_MIC_PACKET_SAMPLES * AUDIO_MIC_RING_PACKETS)

typedef struct {
    uint8_t  alt_setting;                       /* Current alt of the AS interface */
    uint8_t  mute;                              /* Mute state (0/1) */

    /* Single-producer (I2S IRQ) / single-consumer (USB IRQ) ring buffer.
     * Indices are in samples (int16_t units). volatile because they are
     * exchanged between two IRQ contexts. */
    int16_t  ring[AUDIO_MIC_RING_SAMPLES];
    volatile uint16_t wr_idx;
    volatile uint16_t rd_idx;

    /* Outgoing packet buffer, must persist across DataIn callbacks because
     * USBD_LL_Transmit only sets up the transfer and returns. */
    int16_t  tx_pkt[AUDIO_MIC_PACKET_SAMPLES];

    /* EP0 control transfer scratch area for SET_CUR. */
    uint8_t  ctl_cmd;
    uint8_t  ctl_unit;
    uint8_t  ctl_len;
    uint8_t  ctl_data[8];
} USBD_AUDIO_MIC_HandleTypeDef;

extern USBD_ClassTypeDef USBD_AUDIO_MIC;
#define USBD_AUDIO_MIC_CLASS &USBD_AUDIO_MIC

/* Producer API: push `count` int16 mono samples into the ring buffer.
 * Safe to call from an interrupt at lower priority than the USB IRQ. */
void USBD_AUDIO_MIC_PushSamples(const int16_t *samples, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_AUDIO_MIC_H */
