/**
  ******************************************************************************
  * @file    usbd_audio_mic.c
  * @brief   USB Audio Class 1.0 microphone (IN) implementation.
  *
  * Topology:
  *   [Microphone (0x0201) ID=1]
  *           |
  *           v
  *   [Feature Unit ID=2, mute]
  *           |
  *           v
  *   [USB Streaming (0x0101) ID=3]  -->  ISO IN endpoint 0x81
  *
  * Data flow:
  *   I2S DMA half/full callback -> USBD_AUDIO_MIC_PushSamples() -> ring buffer
  *   USB IRQ DataIn callback     -> AUDIO_MIC_SendNextPacket()  <- ring buffer
  ******************************************************************************
  */

#include "usbd_audio_mic.h"
#include "usbd_ctlreq.h"
#include "usbd_core.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ============================================================================
 *                       Forward declarations
 * ==========================================================================*/
static uint8_t  USBD_AUDIO_MIC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_MIC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_MIC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_AUDIO_MIC_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_MIC_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t  USBD_AUDIO_MIC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_AUDIO_MIC_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t  USBD_AUDIO_MIC_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_MIC_REQ_GetCurrent(USBD_HandleTypeDef *pdev,
                                     USBD_SetupReqTypedef *req,
                                     uint8_t recipient);
static void AUDIO_MIC_REQ_SetCurrent(USBD_HandleTypeDef *pdev,
                                     USBD_SetupReqTypedef *req,
                                     uint8_t recipient);
static void AUDIO_MIC_SendNextPacket(USBD_HandleTypeDef *pdev);

/* ============================================================================
 *                          Class descriptor table
 * ==========================================================================*/
USBD_ClassTypeDef USBD_AUDIO_MIC = {
    USBD_AUDIO_MIC_Init,
    USBD_AUDIO_MIC_DeInit,
    USBD_AUDIO_MIC_Setup,
    NULL,                              /* EP0_TxSent  */
    USBD_AUDIO_MIC_EP0_RxReady,
    USBD_AUDIO_MIC_DataIn,
    NULL,                              /* DataOut     */
    NULL,                              /* SOF         */
    USBD_AUDIO_MIC_IsoINIncomplete,
    NULL,                              /* IsoOUTIncomplete */
#ifdef USE_USBD_COMPOSITE
    NULL, NULL, NULL, NULL,
#else
    USBD_AUDIO_MIC_GetCfgDesc,         /* HS */
    USBD_AUDIO_MIC_GetCfgDesc,         /* FS */
    USBD_AUDIO_MIC_GetCfgDesc,         /* OtherSpeed */
    USBD_AUDIO_MIC_GetDeviceQualifierDesc,
#endif
};

/* ============================================================================
 *                       Configuration descriptor
 * ==========================================================================*/
#define AUDIO_FREQ_B0  ((uint8_t)((USBD_AUDIO_MIC_FREQ)       & 0xFFU))
#define AUDIO_FREQ_B1  ((uint8_t)((USBD_AUDIO_MIC_FREQ >>  8) & 0xFFU))
#define AUDIO_FREQ_B2  ((uint8_t)((USBD_AUDIO_MIC_FREQ >> 16) & 0xFFU))

#define AUDIO_PKT_LO   ((uint8_t)((AUDIO_MIC_PACKET_SIZE)      & 0xFFU))
#define AUDIO_PKT_HI   ((uint8_t)((AUDIO_MIC_PACKET_SIZE >> 8) & 0xFFU))

/* Length of the AC interface "collection" (header + IT + FU + OT) */
#define AUDIO_MIC_AC_TOTAL_LEN     (9U + 12U + 9U + 9U)   /* = 39 */

__ALIGN_BEGIN static uint8_t USBD_AUDIO_MIC_CfgDesc[USBD_AUDIO_MIC_CONFIG_DESC_SIZ] __ALIGN_END = {
    /* ===== Configuration descriptor (9) ===== */
    0x09,
    USB_DESC_TYPE_CONFIGURATION,
    LOBYTE(USBD_AUDIO_MIC_CONFIG_DESC_SIZ),
    HIBYTE(USBD_AUDIO_MIC_CONFIG_DESC_SIZ),
    0x02,                              /* bNumInterfaces */
    0x01,                              /* bConfigurationValue */
    0x00,                              /* iConfiguration */
    0xC0,                              /* bmAttributes: self-powered */
    0x32,                              /* bMaxPower = 100 mA */

    /* ===== Standard AC interface (9) ===== */
    0x09,
    USB_DESC_TYPE_INTERFACE,
    AUDIO_MIC_AC_INTERFACE,
    0x00,                              /* bAlternateSetting */
    0x00,                              /* bNumEndpoints (AC has no streaming EP) */
    0x01,                              /* bInterfaceClass = AUDIO */
    0x01,                              /* bInterfaceSubClass = AUDIOCONTROL */
    0x00,                              /* bInterfaceProtocol */
    0x00,                              /* iInterface */

    /* ===== Class-specific AC Header (9) ===== */
    0x09,
    0x24,                              /* CS_INTERFACE */
    0x01,                              /* HEADER */
    0x00, 0x01,                        /* bcdADC 1.00 */
    LOBYTE(AUDIO_MIC_AC_TOTAL_LEN),
    HIBYTE(AUDIO_MIC_AC_TOTAL_LEN),
    0x01,                              /* bInCollection */
    AUDIO_MIC_AS_INTERFACE,            /* baInterfaceNr(1) */

    /* ===== Input Terminal: Microphone (12) ===== */
    0x0C,
    0x24,                              /* CS_INTERFACE */
    0x02,                              /* INPUT_TERMINAL */
    AUDIO_MIC_INPUT_TERMINAL_ID,       /* bTerminalID = 1 */
    0x01, 0x02,                        /* wTerminalType = 0x0201 Microphone */
    0x00,                              /* bAssocTerminal */
    USBD_AUDIO_MIC_CHANNELS,           /* bNrChannels */
    0x00, 0x00,                        /* wChannelConfig = 0 (Mono) */
    0x00,                              /* iChannelNames */
    0x00,                              /* iTerminal */

    /* ===== Feature Unit (9) ===== */
    /* bLength = 7 + (ch+1)*bControlSize = 7 + 2*1 = 9 */
    0x09,
    0x24,                              /* CS_INTERFACE */
    0x06,                              /* FEATURE_UNIT */
    AUDIO_MIC_FEATURE_UNIT_ID,         /* bUnitID = 2 */
    AUDIO_MIC_INPUT_TERMINAL_ID,       /* bSourceID = 1 */
    0x01,                              /* bControlSize = 1 byte per channel */
    0x01,                              /* bmaControls(0) master: Mute */
    0x00,                              /* bmaControls(1) ch1:    none */
    0x00,                              /* iFeature */

    /* ===== Output Terminal: USB Streaming (9) ===== */
    0x09,
    0x24,                              /* CS_INTERFACE */
    0x03,                              /* OUTPUT_TERMINAL */
    AUDIO_MIC_OUTPUT_TERMINAL_ID,      /* bTerminalID = 3 */
    0x01, 0x01,                        /* wTerminalType = 0x0101 USB Streaming */
    0x00,                              /* bAssocTerminal */
    AUDIO_MIC_FEATURE_UNIT_ID,         /* bSourceID = 2 */
    0x00,                              /* iTerminal */

    /* ===== Standard AS interface alt 0 (zero bandwidth) (9) ===== */
    0x09,
    USB_DESC_TYPE_INTERFACE,
    AUDIO_MIC_AS_INTERFACE,
    0x00,                              /* bAlternateSetting = 0 */
    0x00,                              /* bNumEndpoints */
    0x01,                              /* AUDIO */
    0x02,                              /* AUDIOSTREAMING */
    0x00,
    0x00,

    /* ===== Standard AS interface alt 1 (operational) (9) ===== */
    0x09,
    USB_DESC_TYPE_INTERFACE,
    AUDIO_MIC_AS_INTERFACE,
    0x01,                              /* bAlternateSetting = 1 */
    0x01,                              /* bNumEndpoints */
    0x01,                              /* AUDIO */
    0x02,                              /* AUDIOSTREAMING */
    0x00,
    0x00,

    /* ===== Class-specific AS General (7) ===== */
    0x07,
    0x24,                              /* CS_INTERFACE */
    0x01,                              /* AS_GENERAL */
    AUDIO_MIC_OUTPUT_TERMINAL_ID,      /* bTerminalLink = 3 */
    0x01,                              /* bDelay */
    0x01, 0x00,                        /* wFormatTag = PCM */

    /* ===== Type I Format (11) ===== */
    0x0B,
    0x24,                              /* CS_INTERFACE */
    0x02,                              /* FORMAT_TYPE */
    0x01,                              /* FORMAT_TYPE_I */
    USBD_AUDIO_MIC_CHANNELS,           /* bNrChannels */
    (USBD_AUDIO_MIC_BITS / 8U),        /* bSubFrameSize */
    USBD_AUDIO_MIC_BITS,               /* bBitResolution */
    0x01,                              /* bSamFreqType = 1 (one discrete freq) */
    AUDIO_FREQ_B0, AUDIO_FREQ_B1, AUDIO_FREQ_B2,

    /* ===== Standard ISO IN endpoint (9) ===== */
    0x09,
    USB_DESC_TYPE_ENDPOINT,
    AUDIO_MIC_IN_EP,
    0x05,                              /* bmAttributes: Isoc + Async source */
    AUDIO_PKT_LO, AUDIO_PKT_HI,        /* wMaxPacketSize */
    0x01,                              /* bInterval = 1 ms */
    0x00,                              /* bRefresh */
    0x00,                              /* bSynchAddress */

    /* ===== Class-specific AS Isoc Audio Data EP (7) ===== */
    0x07,
    0x25,                              /* CS_ENDPOINT */
    0x01,                              /* EP_GENERAL */
    0x01,                              /* bmAttributes: Sampling Frequency Control supported */
    0x00,                              /* bLockDelayUnits */
    0x00, 0x00,                        /* wLockDelay */
};

/* USB Device Qualifier descriptor (FS-only device must still expose it). */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_MIC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02,
    0x00, 0x00, 0x00,
    0x40,
    0x01,
    0x00,
};

/* ============================================================================
 *                          Single instance state
 * ==========================================================================*/
__ALIGN_BEGIN static USBD_AUDIO_MIC_HandleTypeDef haudio_mic __ALIGN_END;

/* ============================================================================
 *                              Class methods
 * ==========================================================================*/
static uint8_t USBD_AUDIO_MIC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    USBD_AUDIO_MIC_HandleTypeDef *haudio = &haudio_mic;
    memset(haudio, 0, sizeof(*haudio));

    pdev->pClassDataCmsit[pdev->classId] = haudio;
    pdev->pClassData = haudio;

    /* Open the audio ISO IN endpoint. Alt 0 has no endpoints active, but the
     * STM32 OTG stack tolerates pre-opening; we will (re)flush on alt change. */
    pdev->ep_in[AUDIO_MIC_IN_EP & 0x0FU].bInterval = 0x01U;
    (void)USBD_LL_OpenEP(pdev, AUDIO_MIC_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_MIC_PACKET_SIZE);
    pdev->ep_in[AUDIO_MIC_IN_EP & 0x0FU].is_used = 1U;

    return (uint8_t)USBD_OK;
}

static uint8_t USBD_AUDIO_MIC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    (void)USBD_LL_CloseEP(pdev, AUDIO_MIC_IN_EP);
    pdev->ep_in[AUDIO_MIC_IN_EP & 0x0FU].is_used = 0U;
    pdev->ep_in[AUDIO_MIC_IN_EP & 0x0FU].bInterval = 0U;

    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;

    return (uint8_t)USBD_OK;
}

static uint8_t USBD_AUDIO_MIC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    USBD_StatusTypeDef ret = USBD_OK;
    uint16_t status = 0U;

    if (haudio == NULL) {
        return (uint8_t)USBD_FAIL;
    }

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    /* ---------- Class-specific (audio) requests ---------- */
    case USB_REQ_TYPE_CLASS: {
        /* Recipient bits 0..4 of bmRequestType: 1 = interface, 2 = endpoint. */
        uint8_t recipient = req->bmRequest & 0x1FU;
        switch (req->bRequest) {
        case 0x81U:  /* GET_CUR */
        case 0x82U:  /* GET_MIN */
        case 0x83U:  /* GET_MAX */
        case 0x84U:  /* GET_RES */
            AUDIO_MIC_REQ_GetCurrent(pdev, req, recipient);
            break;
        case 0x01U:  /* SET_CUR */
            AUDIO_MIC_REQ_SetCurrent(pdev, req, recipient);
            break;
        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;
    }

    /* ---------- Standard requests targeted at our interface ---------- */
    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                (void)USBD_CtlSendData(pdev, (uint8_t *)&status, 2U);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                uint8_t intf = LOBYTE(req->wIndex);
                haudio->ctl_data[0] =
                    (intf == AUDIO_MIC_AS_INTERFACE) ? haudio->alt_setting : 0U;
                (void)USBD_CtlSendData(pdev, haudio->ctl_data, 1U);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_SET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                uint8_t intf    = LOBYTE(req->wIndex);
                uint8_t new_alt = LOBYTE(req->wValue);

                if (intf == AUDIO_MIC_AS_INTERFACE && new_alt <= 1U) {
                    haudio->alt_setting = new_alt;
                    if (new_alt == 1U) {
                        /* Always (re)prime, regardless of previous alt — the
                         * host may re-issue SET_INTERFACE(alt=1) as part of
                         * error recovery and the EP may be in an unknown
                         * state. */
                        haudio->wr_idx = 0U;
                        haudio->rd_idx = 0U;
                        memset(haudio->tx_pkt, 0, sizeof(haudio->tx_pkt));
                        (void)USBD_LL_FlushEP(pdev, AUDIO_MIC_IN_EP);
                        (void)USBD_LL_Transmit(pdev, AUDIO_MIC_IN_EP,
                                               (uint8_t *)haudio->tx_pkt,
                                               AUDIO_MIC_PACKET_SIZE);
                    } else {
                        (void)USBD_LL_FlushEP(pdev, AUDIO_MIC_IN_EP);
                    }
                } else if (intf == AUDIO_MIC_AC_INTERFACE && new_alt == 0U) {
                    /* AC interface only has alt 0 — ack silently. */
                } else {
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                }
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_CLEAR_FEATURE:
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }

    return (uint8_t)ret;
}

static uint8_t *USBD_AUDIO_MIC_GetCfgDesc(uint16_t *length)
{
    *length = (uint16_t)sizeof(USBD_AUDIO_MIC_CfgDesc);
    return USBD_AUDIO_MIC_CfgDesc;
}

static uint8_t *USBD_AUDIO_MIC_GetDeviceQualifierDesc(uint16_t *length)
{
    *length = (uint16_t)sizeof(USBD_AUDIO_MIC_DeviceQualifierDesc);
    return USBD_AUDIO_MIC_DeviceQualifierDesc;
}

static uint8_t USBD_AUDIO_MIC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if ((epnum & 0x7FU) == (AUDIO_MIC_IN_EP & 0x7FU)) {
        AUDIO_MIC_SendNextPacket(pdev);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_AUDIO_MIC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (haudio == NULL) {
        return (uint8_t)USBD_FAIL;
    }

    if (haudio->ctl_cmd == 0x01U /* SET_CUR */) {
        if (haudio->ctl_target == 0x01U
            && haudio->ctl_unit == AUDIO_MIC_FEATURE_UNIT_ID
            && haudio->ctl_cs == 0x01U /* MUTE */) {
            haudio->mute = haudio->ctl_data[0];
        } else if (haudio->ctl_target == 0x02U
                   && haudio->ctl_cs == 0x01U /* SAMPLING_FREQ */) {
            /* Single-rate device: accept the rate iff it matches, ignore otherwise.
             * (We already filtered wLength==3 in Setup, so reading 3 bytes is safe.) */
            uint32_t rate = (uint32_t)haudio->ctl_data[0]
                          | ((uint32_t)haudio->ctl_data[1] << 8)
                          | ((uint32_t)haudio->ctl_data[2] << 16);
            (void)rate; /* nothing to actually change — we are fixed-rate */
        }
        haudio->ctl_cmd    = 0U;
        haudio->ctl_target = 0U;
        haudio->ctl_unit   = 0U;
        haudio->ctl_cs     = 0U;
        haudio->ctl_len    = 0U;
    }
    return (uint8_t)USBD_OK;
}

static uint8_t USBD_AUDIO_MIC_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (haudio == NULL || haudio->alt_setting != 1U) {
        return (uint8_t)USBD_OK;
    }
    if ((epnum & 0x7FU) != (AUDIO_MIC_IN_EP & 0x7FU)) {
        return (uint8_t)USBD_OK;
    }

    /* Host missed our IN window — flush the stale packet and re-arm. */
    (void)USBD_LL_FlushEP(pdev, AUDIO_MIC_IN_EP);
    AUDIO_MIC_SendNextPacket(pdev);
    return (uint8_t)USBD_OK;
}

/* ============================================================================
 *                           Control transfer helpers
 * ==========================================================================*/
static void AUDIO_MIC_REQ_GetCurrent(USBD_HandleTypeDef *pdev,
                                     USBD_SetupReqTypedef *req,
                                     uint8_t recipient)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (haudio == NULL) {
        return;
    }

    uint8_t cs   = HIBYTE(req->wValue);
    uint8_t unit = HIBYTE(req->wIndex);

    memset(haudio->ctl_data, 0, sizeof(haudio->ctl_data));
    uint16_t respond_len = 0U;
    uint8_t  stall       = 1U;

    if (recipient == 0x01U /* interface (feature unit) */) {
        if (unit == AUDIO_MIC_FEATURE_UNIT_ID && cs == 0x01U /* MUTE */) {
            /* Only GET_CUR makes sense for MUTE; GET_MIN/MAX/RES → stall. */
            if (req->bRequest == 0x81U) {
                haudio->ctl_data[0] = haudio->mute;
                respond_len = 1U;
                stall = 0U;
            }
        }
    } else if (recipient == 0x02U /* endpoint */) {
        if (cs == 0x01U /* SAMPLING_FREQ_CONTROL */) {
            /* Single supported rate — report it for GET_CUR/MIN/MAX, 0 for RES. */
            uint32_t freq = (req->bRequest == 0x84U) ? 0U
                                                     : (uint32_t)USBD_AUDIO_MIC_FREQ;
            haudio->ctl_data[0] = (uint8_t)(freq & 0xFFU);
            haudio->ctl_data[1] = (uint8_t)((freq >> 8) & 0xFFU);
            haudio->ctl_data[2] = (uint8_t)((freq >> 16) & 0xFFU);
            respond_len = 3U;
            stall = 0U;
        }
    }

    if (stall) {
        USBD_CtlError(pdev, req);
        return;
    }

    uint16_t len = (req->wLength < respond_len) ? req->wLength : respond_len;
    (void)USBD_CtlSendData(pdev, haudio->ctl_data, len);
}

static void AUDIO_MIC_REQ_SetCurrent(USBD_HandleTypeDef *pdev,
                                     USBD_SetupReqTypedef *req,
                                     uint8_t recipient)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (haudio == NULL || req->wLength == 0U) {
        USBD_CtlError(pdev, req);
        return;
    }

    uint8_t cs   = HIBYTE(req->wValue);
    uint8_t unit = HIBYTE(req->wIndex);
    uint8_t accept = 0U;

    if (recipient == 0x01U /* interface (feature unit) */) {
        if (unit == AUDIO_MIC_FEATURE_UNIT_ID && cs == 0x01U /* MUTE */
            && req->wLength == 1U) {
            accept = 1U;
        }
    } else if (recipient == 0x02U /* endpoint */) {
        if (cs == 0x01U /* SAMPLING_FREQ_CONTROL */ && req->wLength == 3U) {
            accept = 1U;
        }
    }

    if (!accept) {
        USBD_CtlError(pdev, req);
        return;
    }

    haudio->ctl_cmd    = 0x01U; /* SET_CUR */
    haudio->ctl_target = recipient;
    haudio->ctl_unit   = unit;
    haudio->ctl_cs     = cs;
    haudio->ctl_len    = (req->wLength < sizeof(haudio->ctl_data))
                       ? (uint8_t)req->wLength
                       : (uint8_t)sizeof(haudio->ctl_data);

    (void)USBD_CtlPrepareRx(pdev, haudio->ctl_data, haudio->ctl_len);
}

/* ============================================================================
 *                       Packet pump (consumer side)
 * ==========================================================================*/
static void AUDIO_MIC_SendNextPacket(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (haudio == NULL || haudio->alt_setting != 1U) {
        return;
    }

    const uint16_t ring_n = AUDIO_MIC_RING_SAMPLES;
    const uint16_t pkt_n  = AUDIO_MIC_PACKET_SAMPLES;

    uint16_t wr = haudio->wr_idx;
    uint16_t rd = haudio->rd_idx;
    uint16_t available = (wr >= rd) ? (uint16_t)(wr - rd)
                                    : (uint16_t)(ring_n - rd + wr);

    if (available >= pkt_n) {
        if (haudio->mute) {
            memset(haudio->tx_pkt, 0, sizeof(haudio->tx_pkt));
            rd = (uint16_t)((rd + pkt_n) % ring_n);
        } else {
            for (uint16_t i = 0U; i < pkt_n; ++i) {
                haudio->tx_pkt[i] = haudio->ring[rd];
                rd = (uint16_t)((rd + 1U) % ring_n);
            }
        }
        haudio->rd_idx = rd;
    } else {
        /* Underrun: send silence to keep the stream alive (host expects a
         * packet every 1 ms or it will retry / drop). */
        memset(haudio->tx_pkt, 0, sizeof(haudio->tx_pkt));
    }

    (void)USBD_LL_Transmit(pdev, AUDIO_MIC_IN_EP,
                           (uint8_t *)haudio->tx_pkt,
                           AUDIO_MIC_PACKET_SIZE);
}

/* ============================================================================
 *                  Public producer API (called from I2S IRQ)
 * ==========================================================================*/
void USBD_AUDIO_MIC_PushSamples(const int16_t *samples, uint16_t count)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }
    USBD_AUDIO_MIC_HandleTypeDef *haudio =
        (USBD_AUDIO_MIC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (haudio == NULL) {
        return;
    }

    const uint16_t ring_n = AUDIO_MIC_RING_SAMPLES;
    uint16_t wr = haudio->wr_idx;

    for (uint16_t i = 0U; i < count; ++i) {
        haudio->ring[wr] = samples[i];
        wr = (uint16_t)((wr + 1U) % ring_n);
    }
    haudio->wr_idx = wr;
}
