/**
  ******************************************************************************
  * @file    usbd_audio_mic.h
  * @brief   USB Audio Class 1.0 microphone array (IN) implementation.
  *          4-channel interleaved, 24-bit PCM packed in 3 bytes (S24_3LE),
  *          fixed sample rate. Sources 4 ICS43434 microphones sharing one
  *          BCLK/WS pair (sample-aligned in hardware): two mics per I2S
  *          data line, distinguished by the L/R SEL pin.
  *
  *          Per-frame channel order on the USB wire (interleaved):
  *              [mic1, mic2, mic3, mic4, mic1, mic2, mic3, mic4, ...]
  *          where
  *              mic1 = I2S1 SD, SEL=GND  (master, L slot)
  *              mic2 = I2S1 SD, SEL=VDD  (master, R slot)
  *              mic3 = I2S3 SD, SEL=GND  (slave,  L slot)
  *              mic4 = I2S3 SD, SEL=VDD  (slave,  R slot)
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
#define USBD_AUDIO_MIC_CHANNELS             4U
#define USBD_AUDIO_MIC_BITS                 24U
#define USBD_AUDIO_MIC_BYTES_PER_SAMPLE     3U  /* S24_3LE: 24-bit packed in 3 bytes */

/* Bytes per 1ms USB frame (one isochronous packet).
 *   16 kHz * 4 channels * 3 bytes = 192 bytes/ms — well within FS isoc limits
 *   (max 1023 B per isoc microframe). */
#define AUDIO_MIC_PACKET_SIZE \
    ((USBD_AUDIO_MIC_FREQ / 1000U) * USBD_AUDIO_MIC_CHANNELS * USBD_AUDIO_MIC_BYTES_PER_SAMPLE)

/* "Samples" here = individual channel values (NOT frames). One 1 ms packet
 * carries 16 frames * 4 channels = 64 channel-samples. */
#define AUDIO_MIC_PACKET_SAMPLES            (AUDIO_MIC_PACKET_SIZE / USBD_AUDIO_MIC_BYTES_PER_SAMPLE)

/* ===== USB topology constants ===== */
#define AUDIO_MIC_IN_EP                     0x81U
#define AUDIO_MIC_AC_INTERFACE              0x00U
#define AUDIO_MIC_AS_INTERFACE              0x01U

/* Audio Control unit IDs */
#define AUDIO_MIC_INPUT_TERMINAL_ID         0x01U
#define AUDIO_MIC_FEATURE_UNIT_ID           0x02U
#define AUDIO_MIC_OUTPUT_TERMINAL_ID        0x03U

/* Total length of the configuration descriptor.
 * Compared to the mono variant the Feature Unit gained 3 extra per-channel
 * bmaControls bytes (bLength: 9 -> 12), so the whole config grew by 3 bytes:
 *   0x6D (109) -> 0x70 (112). */
#define USBD_AUDIO_MIC_CONFIG_DESC_SIZ      0x70U

/* ===== Ring buffer sizing =====
 * Holds AUDIO_MIC_RING_PACKETS worth of 1ms packets to absorb jitter between
 * the I2S producer and the USB consumer. 8 ms at 4 ch / 16 kHz = 512 slots
 * (2 KiB), still trivial on the F411's 128 KiB SRAM.
 */
#define AUDIO_MIC_RING_PACKETS              8U
#define AUDIO_MIC_RING_SAMPLES              (AUDIO_MIC_PACKET_SAMPLES * AUDIO_MIC_RING_PACKETS)

typedef struct {
    uint8_t  alt_setting;                       /* Current alt of the AS interface */
    uint8_t  mute;                              /* Mute state (0/1) */

    /* Single-producer (I2S IRQ) / single-consumer (USB IRQ) ring buffer.
     * Each slot stores one full I2S word with the 24-bit sample in bits
     * [31:8] (sign-extended via the int32 representation) and bits [7:0]
     * zero — i.e. exactly the raw layout coming out of the I2S DMA. We
     * pack to 3-byte little-endian only when copying into tx_pkt, so the
     * full precision is preserved end-to-end through firmware. Indices
     * are in samples; volatile because they cross IRQ contexts. */
    int32_t  ring[AUDIO_MIC_RING_SAMPLES];
    volatile uint16_t wr_idx;
    volatile uint16_t rd_idx;

    /* Outgoing packet buffer, must persist across DataIn callbacks because
     * USBD_LL_Transmit only sets up the transfer and returns. Holds the
     * already-packed S24_3LE bytes ready to go on the wire. */
    uint8_t  tx_pkt[AUDIO_MIC_PACKET_SIZE];

    /* tx_busy = 1 when a packet has been handed to USBD_LL_Transmit and we
     * have not yet seen the matching DataIn / IsoINIncomplete completion.
     * SOF uses this as the "must I bootstrap?" flag — primary re-arm is still
     * DataIn (after success) and IsoINIncomplete (after host miss). */
    volatile uint8_t tx_busy;

    /* EP0 control transfer scratch area for SET_CUR.
     * ctl_target  = 1 -> interface (feature unit)
     *             = 2 -> endpoint (sampling frequency)
     * ctl_unit    = HIBYTE(wIndex)  (unit ID for IF target, undefined for EP)
     * ctl_cs      = HIBYTE(wValue)  (control selector)
     */
    uint8_t  ctl_cmd;
    uint8_t  ctl_target;
    uint8_t  ctl_unit;
    uint8_t  ctl_cs;
    uint8_t  ctl_len;
    uint8_t  ctl_data[8];
} USBD_AUDIO_MIC_HandleTypeDef;

extern USBD_ClassTypeDef USBD_AUDIO_MIC;
#define USBD_AUDIO_MIC_CLASS &USBD_AUDIO_MIC

/* Producer API: push `count` channel-samples into the ring buffer in the
 * exact order they should appear on the USB wire. For the 4-mic array that
 * means interleaved frames of 4 ints: [m1, m2, m3, m4, m1, m2, m3, m4, ...].
 * Each int32 is a raw 32-bit I2S slot value: 24-bit signed sample in bits
 * [31:8] (sign-extended), bits [7:0] = 0. This matches what the I2S DMA
 * delivers, so no conversion is needed on the producer side. Safe to call
 * from an interrupt at lower priority than the USB IRQ. */
void USBD_AUDIO_MIC_PushSamples(const int32_t *samples, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_AUDIO_MIC_H */
