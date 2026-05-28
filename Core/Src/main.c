/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_audio_mic.h"   // USBD_AUDIO_MIC_PushSamples()
#include "usbd_core.h"        // USBD_STATE_CONFIGURED
#include <string.h>           // memset
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */
/* I2S DMA buffers — one per peripheral, both shaped identically so the
 * same slot index in each buffer corresponds to the same physical moment
 * in time. That's true because SPI1 (master) drives BCLK/WS while SPI3
 * (slave) listens to the very same signals via two short jumper wires
 * (PA4->PA15 for WS, PA5->PB3 for BCLK). Both peripherals therefore
 * latch on identical clock edges and their DMAs advance in lockstep.
 *
 * I2S sends stereo frames (L+R), each a 24-bit sample left-justified in
 * a 32-bit channel slot. With DMA configured as PSIZE=HALFWORD +
 * MSIZE=WORD (see HAL_I2S_MspInit), the DMA packs every two 16-bit reads
 * from SPI_DR into one 32-bit memory write. So each uint32 holds exactly
 * one full 32-bit audio slot (one channel), in this layout:
 *     bits  0..15  = upper 16 bits of the 24-bit sample (bits 23..8)
 *     bits 16..31  = lower 8 bits (left-justified, padded with zeros)
 *
 * I2S_BUF_SIZE = uint32 channel slots per buffer = 2 mic samples per buffer
 *                slot × 2 (stereo: SEL=GND in L, SEL=VDD in R).
 * At 16 kHz, 64 slots = 32 stereo frames = 2 ms total, so each
 * half-complete callback covers exactly 1 ms (16 frames = 16 samples per
 * mic), matching one USB 1 ms isochronous frame.
 *
 * The two buffers together carry 4 channels (mic1+mic2 in i2s1, mic3+mic4
 * in i2s3); we interleave them into the USB ring inside the master's
 * DMA callback.
 */
#define I2S_BUF_SIZE 64
uint32_t i2s1_rx_buf[I2S_BUF_SIZE];
uint32_t i2s3_rx_buf[I2S_BUF_SIZE];
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S1_Init();
  MX_I2S3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // DMA startuje dopiero po enumeracji USB (w pętli głównej)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint8_t dma_started = 0;
    if (!dma_started && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
      /* For I2S_DATAFORMAT_24B the HAL interprets `Size` as the number of
       * audio samples (24-bit in 32-bit slots), and internally doubles it
       * to NDTR (half-word transfers). With PSIZE=HALFWORD + MSIZE=WORD,
       * the DMA packs every two 16-bit reads from SPI_DR into one 32-bit
       * memory write — so each uint32 in i2sX_rx_buf holds exactly one
       * 32-bit audio channel slot.
       *
       * Sizing: I2S_BUF_SIZE (= 64) audio samples = 32 stereo frames
       *   -> HAL NDTR = 128 half-word reads
       *   -> 64 word writes to memory = exactly i2sX_rx_buf (256 bytes)
       *   -> half-complete fires every 16 stereo frames = 1 ms @ 16 kHz
       *      (matches one USB isochronous frame).
       *
       * Start order is critical for sample-accurate sync between the two
       * peripherals: arm the SLAVE first so it's already listening on
       * PA15/PB3 when the master starts driving them. The slave then locks
       * onto the very first WS edge the master generates and both DMAs
       * advance in lockstep from sample 0 onward. */
      HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)i2s3_rx_buf, I2S_BUF_SIZE);
      HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)i2s1_rx_buf, I2S_BUF_SIZE);
      dma_started = 1;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function — MASTER RX for mic1 + mic2.
  *        Generates BCLK on PA5 and WS on PA4; those signals are wired
  *        (jumpered) to PA15/PB3 so I2S3 (slave) sees the same clocks.
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function — SLAVE RX for mic3 + mic4.
  *        Receives BCLK on PB3 and WS on PA15 from external jumper wires
  *        coming from the master (PA5 and PA4 respectively). All Init
  *        fields except Mode must match the master so the two FIFOs
  *        interpret the same bitstream identically.
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable.
   * Master (SPI1_RX) lives on DMA2 Stream 0 Ch 3, slave (SPI3_RX) on
   * DMA1 Stream 0 Ch 0 — different controllers so they don't share the
   * AHB master port and can't starve each other. */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  /* Priorytet niższy niż USB OTG (0,0) aby uniknąć wyścigu.
   * Master callback (SPI1) is where we combine both buffers and push to
   * the USB ring, so it absolutely must not preempt the USB IRQ. */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  /* DMA1_Stream0_IRQn — SPI3 (slave) RX.
   * Same preemption level as the master DMA; the callback is a no-op
   * (just used so HAL keeps the HT/TC flags clean) so subpriority is
   * irrelevant. */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Convert one I2S channel slot (uint32 as packed by DMA) back into the
 * raw 32-bit I2S word so we can stream the full 24-bit precision to the
 * host. No gain, no truncation — we want every bit of dynamic range
 * preserved for downstream DSP (beamforming, drone-noise suppression,
 * far-field target extraction). The host can boost / filter / clip
 * later with full information; once we throw bits away in firmware
 * they are gone forever.
 *
 * DMA layout (PSIZE=HALFWORD, MSIZE=WORD) packs two 16-bit reads from
 * SPI_DR into one 32-bit memory word:
 *   - low 16 bits  = first half-word received  = slot bits [31:16]
 *                                              = sample bits [23:8]
 *   - high 16 bits = second half-word received = slot bits [15:0]
 *                                              = sample bits [7:0] << 8
 * Reassembling them gives the original 32-bit I2S slot with the 24-bit
 * signed sample in bits [31:8] and bits [7:0] = 0. The USB class then
 * packs this into 3-byte little-endian S24_3LE for transmission. */
static inline int32_t i2s_slot_to_int32(uint32_t frame)
{
    uint16_t hi = (uint16_t)(frame & 0xFFFFU);
    uint16_t lo = (uint16_t)((frame >> 16) & 0xFFFFU);
    return (int32_t)(((uint32_t)hi << 16) | (uint32_t)lo);
}

/* Combine one half of BOTH I2S DMA buffers into interleaved 4-channel
 * samples and push them to the USB microphone class ring buffer.
 *
 * Per ICS43434:
 *   SEL=GND -> mic transmits during the LEFT half of WS  (= even slot)
 *   SEL=VDD -> mic transmits during the RIGHT half of WS (= odd slot)
 *
 * Our wiring is:
 *   mic1: I2S1 SD, SEL=GND -> i2s1_rx_buf[even]
 *   mic2: I2S1 SD, SEL=VDD -> i2s1_rx_buf[odd]
 *   mic3: I2S3 SD, SEL=GND -> i2s3_rx_buf[even]
 *   mic4: I2S3 SD, SEL=VDD -> i2s3_rx_buf[odd]
 *
 * Because BCLK/WS are physically shared between SPI1 and SPI3, slot index
 * i in i2s1_rx_buf and slot index i in i2s3_rx_buf reference the exact
 * same physical I2S frame -> no time offset between channels.
 *
 * We emit one interleaved 4-ch frame per stereo slot pair:
 *   [mic1, mic2, mic3, mic4, mic1, mic2, mic3, mic4, ...]
 *
 * slot_count is the number of channel slots covered by this callback
 * (== I2S_BUF_SIZE / 2 == 32 for 1 ms at 16 kHz with the current buffer). */
static void mic_push_half(uint16_t slot_offset, uint16_t slot_count)
{
    const uint32_t *m = &i2s1_rx_buf[slot_offset];
    const uint32_t *s = &i2s3_rx_buf[slot_offset];

    /* Worst case: 16 frames * 4 channels = 64 samples per 1 ms callback. */
    int32_t out[(I2S_BUF_SIZE / 2) * 2];
    uint16_t n = 0U;
    for (uint16_t i = 0U; i < slot_count; i += 2U) {
        out[n++] = i2s_slot_to_int32(m[i]);       /* mic1: master L */
        out[n++] = i2s_slot_to_int32(m[i + 1U]);  /* mic2: master R */
        out[n++] = i2s_slot_to_int32(s[i]);       /* mic3: slave  L */
        out[n++] = i2s_slot_to_int32(s[i + 1U]);  /* mic4: slave  R */
    }
    USBD_AUDIO_MIC_PushSamples(out, n);
}

/* Only the MASTER (SPI1) callbacks drive the USB pump. The slave (SPI3)
 * IRQ still fires — that's what lets HAL clear its HT/TC flags — but the
 * payload is consumed from i2s3_rx_buf inside the master callback, where
 * we already know the lock-step DMA index. */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI1) {
        mic_push_half(0U, I2S_BUF_SIZE / 2U);
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI1) {
        mic_push_half(I2S_BUF_SIZE / 2U, I2S_BUF_SIZE / 2U);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
