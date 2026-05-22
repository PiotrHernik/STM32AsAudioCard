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
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
/* I2S DMA buffer.
 * I2S sends stereo frames (L+R), each a 24-bit sample left-justified in a
 * 32-bit channel slot. With DMA configured as PSIZE=HALFWORD + MSIZE=WORD
 * (see HAL_I2S_MspInit), the DMA packs every two 16-bit reads from SPI_DR
 * into one 32-bit memory write. So each uint32 in i2s_rx_buf holds exactly
 * one full 32-bit audio slot (one channel), in this layout:
 *     bits  0..15  = upper 16 bits of the 24-bit sample (bits 23..8)
 *     bits 16..31  = lower 8 bits (left-justified, padded with zeros)
 *
 * I2S_BUF_SIZE = number of uint32 channel slots in the buffer
 *              = number of mono audio samples per buffer (after dropping R)
 *              × 2 (for stereo)
 * At 16 kHz, 64 slots = 32 stereo frames = 32 mono samples = 2 ms total,
 * so each half-complete callback covers exactly 1 ms (16 mono samples),
 * matching one USB 1 ms isochronous frame.
 */
#define I2S_BUF_SIZE 64
uint32_t i2s_rx_buf[I2S_BUF_SIZE];
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
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
       * memory write — so each uint32 in i2s_rx_buf holds exactly one
       * 32-bit audio channel slot (low 16 bits = upper 16 bits of the
       * 24-bit sample, high 16 bits = the remaining 8 bits left-justified).
       *
       * Sizing: I2S_BUF_SIZE (= 64) audio samples = 32 stereo frames
       *   -> HAL NDTR = 128 half-word reads
       *   -> 64 word writes to memory = exactly i2s_rx_buf (256 bytes)
       *   -> half-complete fires every 16 stereo frames = 1 ms @ 16 kHz
       *      (matches one USB isochronous frame). */
      HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)i2s_rx_buf, I2S_BUF_SIZE);
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
  * @brief I2S1 Initialization Function
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  /* Priorytet niższy niż USB OTG (0,0) aby uniknąć wyścigu */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

/* Convert one half of the I2S DMA buffer into mono int32 samples and push
 * them to the USB microphone class ring buffer. The ICS43434 is a mono
 * mic; with L/R = GND it puts data on the left channel slot, so we take
 * every other uint32 (even indices = L slots, odd = R slot which is mute). */
static void mic_push_half(const uint32_t *frames, uint16_t slot_count)
{
    int32_t mono[I2S_BUF_SIZE / 4]; /* worst case = 16 samples per 1 ms */
    uint16_t n = 0U;
    for (uint16_t i = 0U; i < slot_count; i += 2U) {
        mono[n++] = i2s_slot_to_int32(frames[i]);
    }
    USBD_AUDIO_MIC_PushSamples(mono, n);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
    mic_push_half(&i2s_rx_buf[0], I2S_BUF_SIZE / 2);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;
    mic_push_half(&i2s_rx_buf[I2S_BUF_SIZE / 2], I2S_BUF_SIZE / 2);
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
