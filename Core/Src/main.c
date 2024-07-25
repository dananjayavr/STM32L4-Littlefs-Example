/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LITTLEFS_SECTOR_START 0x08040000
#define STM32L476_SECTOR_SIZE 0x800 //(2048 bytes)

#define FLASH_START_ADDRESS 0x08000000
#define FLASH_END_ADDRESS   0x08100000
#define PAGE_SIZE           2048

// Constants defined by the TRM, section 3.3.1
#define FLASH_BLOCK_SIZE 2048  // FLash block size in bytes
#define FLASH_MAX_BLOCKS 512   // Total number of available blocks

// Tunable constants
#define LFS_BLOCKS 8     // Number of blocks to use
#define LFS_BUF_SIZE 64  // Buffer size used for reads, writes, and cache
#define LFS_MAX_FDS 6    // Maximum number of opened files / dirs

#define PARAMETER_NOT_USED(p) (void) ((p))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int getFlashPageNumber(uint32_t address);
void hexdump(const void *buffer, size_t length);

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

int lfs_driver_read(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, void *buf, lfs_size_t len);
int lfs_driver_prog(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, const void *buf, lfs_size_t len);
int lfs_driver_erase(const struct lfs_config *cfg, lfs_block_t block);
int lfs_driver_sync(const struct lfs_config *cfg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin == PB_USER_Pin)
    printf("Button Pressed.\r\n");

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

/**
 * @brief Dumps the contents of a buffer in hexadecimal format.
 *
 * @param buffer Pointer to the buffer to be dumped.
 * @param length Length of the buffer in bytes.
 */
void hexdump(const void *buffer, size_t length) {
    const unsigned char *buf = (const unsigned char *)buffer;
    
    for (size_t i = 0; i < length; i += 16) {
        // Print the offset
        printf("%08zx  ", i);

        // Print hex values for the next 16 bytes
        for (size_t j = 0; j < 16; j++) {
            if (i + j < length) {
                printf("%02x ", buf[i + j]);
            } else {
                printf("   "); // For alignment of the ASCII section
            }
        }

        // Print ASCII characters for the next 16 bytes
        printf(" |");
        for (size_t j = 0; j < 16; j++) {
            if (i + j < length) {
                unsigned char c = buf[i + j];
                // Check if the byte is a printable character
                if (c >= 32 && c <= 126) {
                    printf("%c", c);
                } else {
                    printf("."); // Non-printable characters are replaced with '.'
                }
            }
        }
        printf("|\n");
    }
}

/**
 * @brief Calculate the flash page number for a given address.
 *
 * @param address The address to calculate the page number for.
 * @return The page number corresponding to the address, or -1 if the address is out of range.
 */
int getFlashPageNumber(uint32_t address) {
    // Check if the address is within the valid flash range
    if (address < FLASH_START_ADDRESS || address >= FLASH_END_ADDRESS) {
        return -1; // Invalid address
    }

    // Calculate the page number
    int pageNumber = (address - FLASH_START_ADDRESS) / PAGE_SIZE;
    
    return pageNumber;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = lfs_driver_read,
    .prog  = lfs_driver_prog,
    .erase = lfs_driver_erase,
    .sync  = lfs_driver_sync,

    // block device configuration
    .read_size = LFS_BUF_SIZE,
    .prog_size = LFS_BUF_SIZE,
    .block_size = FLASH_BLOCK_SIZE,
    .block_count = LFS_BLOCKS,
    .cache_size = LFS_BUF_SIZE,
    .lookahead_size = LFS_BUF_SIZE/2,
    .block_cycles = 200,
};
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Hello, World!\r\n");

  // mount the filesystem
  int err = lfs_mount(&lfs, &cfg);

  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err) {
      lfs_format(&lfs, &cfg);
      lfs_mount(&lfs, &cfg);
  }

  // read current count
  uint32_t boot_count = 0;
  lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

  // update boot count
  boot_count += 1;
  lfs_file_rewind(&lfs, &file);
  lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

  // remember the storage is not updated until the file is closed successfully
  lfs_file_close(&lfs, &file);

  // release any resources we were using
  lfs_unmount(&lfs);

  // print the boot count
  printf("boot_count: %ld\r\n", boot_count);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin);
    HAL_Delay(500);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_USER_Pin */
  GPIO_InitStruct.Pin = PB_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_GREEN_Pin */
  GPIO_InitStruct.Pin = LD_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int lfs_driver_read(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, void *buf, lfs_size_t len) {
	PARAMETER_NOT_USED(cfg);

	if(!buf || !len) {
		printf("%s Invalid parameter!\r\n",__func__);
		return LFS_ERR_INVAL;
	}

	lfs_block_t address = LITTLEFS_SECTOR_START + (block * STM32L476_SECTOR_SIZE + off);
	//printf("+%s(Addr 0x%06lX, Len 0x%04lX)\r\n",__func__,address,len);
	//hexdump((void *)address,len);
	memcpy(buf, (void *)address, len);

	return LFS_ERR_OK;
}

int lfs_driver_prog(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, const void *buf, lfs_size_t len) {
	PARAMETER_NOT_USED(cfg);
	lfs_block_t address = LITTLEFS_SECTOR_START + (block * STM32L476_SECTOR_SIZE + off);
	HAL_StatusTypeDef hal_rc = HAL_OK;
	uint32_t block_count = len / 8;

	//printf("+%s(Addr 0x%06lX, Len 0x%04lX)\r\n",__func__,address,len);
	//hexdump((void *)address,len);
  /* Program the user Flash area word by word
  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
  HAL_FLASH_Unlock();

  uint64_t data_source;

  for(uint32_t i=0;i<block_count;i++)
  {
    memcpy(&data_source,buf,8); // load the 64-bit source from the buffer
    hal_rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_source);
    if (hal_rc == HAL_OK)
    {
      address += 8;
      buf = (uint8_t *)buf + 8;
    }
    else
    {
      /* Error occurred while writing data in Flash memory.
        User can add here some code to deal with this error */
      printf("Program Error, 0x%X\r\n",hal_rc);

    } // else
  } // for
  //printf("-%s\n",__func__);
  
  HAL_FLASH_Lock();
  return hal_rc == HAL_OK?LFS_ERR_OK:LFS_ERR_IO; // If HAL_OK, return LFS_ERR_OK, else return LFS_ERR_IO
}

int lfs_driver_erase(const struct lfs_config *cfg, lfs_block_t block) {
	PARAMETER_NOT_USED(cfg);
	lfs_block_t address = LITTLEFS_SECTOR_START + (block * STM32L476_SECTOR_SIZE);
	//printf("+%s(Addr 0x%06lX)\r\n",__func__,address);

	HAL_StatusTypeDef hal_rc;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError = 0;

  HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = getFlashPageNumber(address);
	EraseInitStruct.NbPages     = 1;
	hal_rc = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	if (hal_rc != HAL_OK)
	{
		printf("%s ERROR 0x%X\r\n",__func__,hal_rc);
	}
	else
		printf("%s SUCCESS\r\n",__func__);

  HAL_FLASH_Lock();
  return hal_rc == HAL_OK?LFS_ERR_OK:LFS_ERR_IO; // If HAL_OK, return LFS_ERR_OK, else return LFS_ERR_IO
}

int lfs_driver_sync(const struct lfs_config *cfg) {
  PARAMETER_NOT_USED(cfg);
  // write function performs no caching.  No need for sync.
  //printf("+%s()\r\n",__func__);
  return LFS_ERR_OK;
  //return LFS_ERR_IO;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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

#ifdef  USE_FULL_ASSERT
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
