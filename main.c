/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (modified for LCD + PWM buzzer + buttons)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include <stdint.h>
#include "i2c-lcd.h"   // Thu vi?n b?n dã có (i2c-lcd.c / i2c-lcd.h)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t volume = 0; // 0..100%
uint32_t last_change_tick = 0;
const uint32_t debounce_ms = 150;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void update_pwm_from_volume(void);
void display_volume_on_lcd(void);
void uint_to_str(uint32_t val, char *buf); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void update_pwm_from_volume(void)
{
    uint32_t arr = htim2.Init.Period;
    uint32_t compare = ( (uint32_t)volume * (arr + 1) ) / 100u;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}


void uint_to_str(uint32_t val, char *buf)
{
    char tmp[5];
    int i = 0;
    if (val == 0) {
        buf[0] = '0'; buf[1] = 0; return;
    }
    while (val > 0 && i < 4) {
        tmp[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
   
    int j;
    for (j = 0; j < i; ++j) {
        buf[j] = tmp[i - 1 - j];
    }
    buf[i] = 0;
}

void display_volume_on_lcd(void)
{
    char numbuf[5];
    uint_to_str(volume, numbuf);

    lcd_clear_display();
    HAL_Delay(2);

    lcd_goto_XY(1, 0); 
    lcd_send_string("Volume Control");

    lcd_goto_XY(2, 0); // Dòng 2, c?t 0
    lcd_send_string("Vol: ");
    lcd_send_string(numbuf);
    lcd_send_string(" %");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  lcd_init();
  HAL_Delay(20);

  // B?t d?u PWM
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
      Error_Handler();
  }

  // Cài duty ban d?u
  update_pwm_from_volume();
  display_volume_on_lcd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t t = HAL_GetTick();

    // Nút tang: PA1 (active LOW)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        if (t - last_change_tick > debounce_ms)
        {
            if (volume <= 95) volume += 5;
            else volume = 100;
            update_pwm_from_volume();
            display_volume_on_lcd();
            last_change_tick = t;
        }
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        if (t - last_change_tick > debounce_ms)
        {
            if (volume >= 5) volume -= 5;
            else volume = 0;
            update_pwm_from_volume();
            display_volume_on_lcd();
            last_change_tick = t;
        }
    }

    HAL_Delay(50);
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  *        (gi? nguyên c?u hình HSI nhu file b?n cung c?p)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                     
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;     
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;         
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                 
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }


  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;            
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}



static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}


static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;   
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;   
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* user can add reporting here */
}
#endif
