---
aliases:
  - STM32
  - 非阻塞式
  - 按键驱动
cssclasses:
  - STM32笔记
Time: 2026-01-01
---

# 引言
	在 STM 32 开发中按键是我们使用比较多的一个外设，它是一个硬件结构并不复杂的外设，但是在软件上可谓五花八门，最经典的也是最完善的莫过Linux的按键驱动，动辄几千行代码量，但是STM32开发中并不需要像Linux那样复杂的按键驱动，在此以我个人记录视角，记录一篇个人使用体验比较好的非阻塞按键驱动，涵盖单击、双击、长按、连击事件.

# 代码

## 头文件
```c fold title:port_key.h 
#ifndef __PORT_KEY_H
#define __PORT_KEY_H

#include "main.h" 

/* 参数配置 */
#define KeyNum              1        /* 按键数量 */

#define KEY_IDLE_TIMEOUT_MS 2000U    /* 无操作 2s 后进入休眠 */ 

#define KEY_DEBOUNCE_MS     20U      /* 消抖时间 */ 

#define KEY_LONG_PRESS_MS   1000U    /* 长按判定时间 */ 

#define KEY_DOUBLE_CLICK_MS 150U     /* 双击间隔 */

#define KEY_REPEAT_RATE_MS  200U     /* 长按触发后，每隔多少毫秒触发一次连击事件 */

typedef enum {
    NoneClick = 0,
    
    SingleClick,
    
    DoubleClick,
    
    LongPress,
    
    ContinuousPress
    
} KeyEvent;

typedef enum {
    KeyIdle = 0,
    
    KeyBounce,
    
    KeyPressed,
    
    KeyReleaseBounce
    
} KeyState;

typedef struct {
    
    GPIO_TypeDef* keyPort;
    
    uint16_t keyPin;
    
    GPIO_PinState activeLevel;
    
    IRQn_Type irqType;
    
} KeyHardwareTypeDef;

typedef struct {
    
    /* 状态机变量 */
    KeyState state;
    
    uint8_t ClickCount;
    
    uint32_t bounceTimerCounter; 
    
    uint32_t pressTimeCounter;
    
    uint32_t lastReleaseTime;
    
    bool longPressTriggered;
    
    uint32_t nextRepeatTime; 

    /* 关联硬件 */ 
    KeyHardwareTypeDef hardWare;
    
} KeyHandleTypeDef;

typedef struct {
    
    KeyHandleTypeDef keys[KeyNum];
    
} KeyGroupTypeDef;

extern KeyGroupTypeDef keyGroup;

void Port_KeyInit(void);
void Key_EXTI_Wakeup_Callback(void);
void Key_Timer_Scan_Callback(void);
KeyEvent GetKeyFunEventHandle(KeyHandleTypeDef* key, uint8_t enableRepeat);

#endif

```

## 源码
```c fold title:port_key.c
#include "./PortKey/port_key.h"
#include "tim.h" 

KeyGroupTypeDef keyGroup;

static uint32_t g_KeyIdleTimeCounter = 0; // 全局空闲计时器


static const KeyHardwareTypeDef HardwareConfig[KeyNum] = 
{
    {GPIOA, GPIO_PIN_15, GPIO_PIN_RESET, EXTI15_10_IRQn}, 
};


void Port_KeyInit(void)
{
    for(uint8_t i = 0; i < KeyNum; i++)
    {
        keyGroup.keys[i].hardWare = HardwareConfig[i];
        keyGroup.keys[i].state = KeyIdle;
        keyGroup.keys[i].ClickCount = 0;
        keyGroup.keys[i].longPressTriggered = false;
    }
    
    HAL_TIM_Base_Stop_IT(&htim7);
    
    for(uint8_t i = 0; i < KeyNum; i++) 
    {
        HAL_NVIC_EnableIRQ(keyGroup.keys[i].hardWare.irqType);
    }
}

static GPIO_PinState ReadGPIOLevel(KeyHandleTypeDef* key) 
{   
    return HAL_GPIO_ReadPin(key->hardWare.keyPort, key->hardWare.keyPin);
}


static void SingleClickKeyHandle(KeyHandleTypeDef* key, uint32_t currentTime)
{
    GPIO_PinState currentLevel = ReadGPIOLevel(key);
    
    bool isPressed = (currentLevel == key->hardWare.activeLevel);

    switch(key->state)
    {
        case KeyIdle:
            if(isPressed) 
            {
                key->state = KeyBounce;
                
                key->bounceTimerCounter = currentTime;
            } 
            else 
            {
                key->longPressTriggered = false;
            }
            break;
        
        case KeyBounce:
            if(!isPressed) 
            {
                key->state = KeyIdle;
            } 
            else if(currentTime - key->bounceTimerCounter >= KEY_DEBOUNCE_MS) 
            {
                key->state = KeyPressed;
                
                key->pressTimeCounter = 0; // 重置长按计时
            }
            break;
        
        case KeyPressed:
            if(!isPressed) 
            {   
                key->state = KeyReleaseBounce; 
                
                key->bounceTimerCounter = currentTime;
            } 
            else 
            {
                // 累加长按时间 (每次进入这里大约经过 10ms)
                key->pressTimeCounter += 10; 
            }
            break;

        case KeyReleaseBounce:
            if(isPressed) 
            {
                key->state = KeyPressed; 
            } 
            else if(currentTime - key->bounceTimerCounter >= KEY_DEBOUNCE_MS) 
            {
                key->state = KeyIdle;
                
                key->lastReleaseTime = currentTime;
                
                if(!key->longPressTriggered) 
                {
                    key->ClickCount++;
                }
            }
            break;
    }
}


static bool IsAllKeysIdle(void)
{
    for(uint8_t i = 0; i < KeyNum; i++) 
    {
        
        if(keyGroup.keys[i].state != KeyIdle || 
           ReadGPIOLevel(&keyGroup.keys[i]) == keyGroup.keys[i].hardWare.activeLevel) 
        {
            return false;
        }
    }
    return true;
}


void Key_EXTI_Wakeup_Callback(void)
{

    if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) == RESET) 
    {
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    }
    
    if(!(htim7.Instance->CR1 & TIM_CR1_CEN)) 
    {
        HAL_TIM_Base_Start_IT(&htim7);
    }

    for(uint8_t i = 0; i < KeyNum; i++) 
    {
        HAL_NVIC_DisableIRQ(keyGroup.keys[i].hardWare.irqType);
    }
    
    g_KeyIdleTimeCounter = 0;
}


void Key_Timer_Scan_Callback(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    for(uint8_t i = 0; i < KeyNum; i++) 
    {
        SingleClickKeyHandle(&keyGroup.keys[i], currentTime);
    }

    if(IsAllKeysIdle() == false) 
    {
        g_KeyIdleTimeCounter = 0; 
    } 
    else 
    {
        g_KeyIdleTimeCounter += 10;
        

        if(g_KeyIdleTimeCounter >= KEY_IDLE_TIMEOUT_MS) 
        {

            HAL_TIM_Base_Stop_IT(&htim7);
            
            __HAL_TIM_CLEAR_FLAG(&htim7,TIM_FLAG_UPDATE);

            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
            

            for(uint8_t i = 0; i < KeyNum; i++) 
            {
                HAL_NVIC_EnableIRQ(keyGroup.keys[i].hardWare.irqType);
            }
            
            g_KeyIdleTimeCounter = 0;
        }
    }
}


KeyEvent GetKeyFunEventHandle(KeyHandleTypeDef* key, uint8_t enableRepeat)
{
    uint32_t currentTime = HAL_GetTick();
    
    KeyEvent event = NoneClick;
    
    // 1. 处理长按 (LongPress) 及 连击初始化
    if (key->state == KeyPressed && !key->longPressTriggered 
        && key->pressTimeCounter >= KEY_LONG_PRESS_MS)
    {
        key->longPressTriggered = true;
        
        // 初始化连击时间点
        key->nextRepeatTime = key->pressTimeCounter + KEY_REPEAT_RATE_MS;
        
        // 根据是否开启连击，决定返回什么事件
        if (enableRepeat)
        {
            // 如果开启连击，第一次达到长按时间时，直接视为第一次连击
            event = ContinuousPress; 
        }
        else
        {
            // 如果关闭连击，则报告标准的长按事件
            event = LongPress;
        }
        
        return event;
    }
    
    // 2. 处理后续的连击 (ContinuousPress)
    // 只有当 enableRepeat 为 1 时，才进入后续检测
    if (enableRepeat && key->state == KeyPressed && key->longPressTriggered)
    {
        // 检查是否到达下一次触发时间
        if (key->pressTimeCounter >= key->nextRepeatTime)
        {
            event = ContinuousPress;
            
            // 更新下一次触发时间
            key->nextRepeatTime = key->pressTimeCounter + KEY_REPEAT_RATE_MS;
            
            return event;
        }
    }
    
    // 3. 处理单击和双击 (松手后的逻辑 - 保持不变)
    if (key->state == KeyIdle && key->ClickCount > 0)
    {
        if (currentTime - key->lastReleaseTime >= KEY_DOUBLE_CLICK_MS)
        {
            if (key->ClickCount == 1)
            {
                event = SingleClick;
            }
            else if (key->ClickCount == 2)
            {
                event = DoubleClick;
            }
            
            key->ClickCount = 0;
        }
    }
    return event;
}
```

### 主程序测试
```c fold title:main.c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bdma.h"
#include "dma.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*includes WheelRobot BSP Head Start */

#include "./PortKey/port_key.h"
#include "./PortLed/port_led.h"
/*includes WheelRobot BSP Head End*/

/*  ----------------------------------------------------------*/

/*includes WheelRobot Application Head Start */

#include "./AppKey/app_key.h"
#include "./AppDebug/app_debug.h"

/*includes WheelRobot Application Head End */

/*  ----------------------------------------------------------*/

/*includes WheelRobot Algorithm Head Start */

#include "./PID/algorithm_pid.h"

/*includes WheelRobot Algorithm Head End */

/*  ----------------------------------------------------------*/

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void MPU_Config(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_BDMA_Init();
  MX_USART1_UART_Init();
  MX_SPI6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Port_KeyInit();
  Log_Init();
  LOG_Debug("System Boot Success! Core Clock: %lu MHz\r\n", SystemCoreClock / 1000000);
  BreathLed_Init(&BreathLed, 10, 200,3);
  RGBLedSendColorData_DMA(&ledPalette[3]);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RGBLed_Breath(&BreathLed);
    for(int i = 0; i < KeyNum; i++)
    {
        KeyEvent evt = GetKeyFunEventHandle(&keyGroup.keys[i],1);
        if(evt != NoneClick)
        {
            // 处理按键逻辑
            if(i == 0) 
            {
                if(evt == SingleClick) { LOG_Info("SingleClick\r\n"); }
                if(evt == DoubleClick)   { LOG_Info("DoubleClick\r\n"); }
                if(evt == LongPress)   { LOG_Info("LongPress\r\n"); }
                if(evt == ContinuousPress) { LOG_Info("Keep Pressing\r\n"); }
            }

        }
    }
      
      
      
      
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  GPIO 外部中断回调
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
    if(GPIO_Pin == GPIO_PIN_15 )
    {
        Key_EXTI_Wakeup_Callback();
    }
}

/**
  * @brief  定时器周期回调
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {   
        Key_Timer_Scan_Callback();
    }   
}



//void MPU_Config(void)
//{
//  MPU_Region_InitTypeDef MPU_InitStruct = {0};

//  /* Disables the MPU */
//  HAL_MPU_Disable();

//  /** Initializes and configures the Region and the memory to be protected
//  */
//  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
//  MPU_InitStruct.BaseAddress = 0x30000000;
//  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
//  MPU_InitStruct.SubRegionDisable = 0x0;
//  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
//  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
//  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
//  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
//  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // 关闭 Cache
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

//  HAL_MPU_ConfigRegion(&MPU_InitStruct);

//  /* Enables the MPU */
//  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
//}




//void MPU_Config(void)
//{
//  MPU_Region_InitTypeDef MPU_InitStruct = {0};

//  HAL_MPU_Disable();

//  // 配置 D2 SRAM (0x30000000) 区域
//  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
//  MPU_InitStruct.BaseAddress = 0x30000000;
//  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
//  MPU_InitStruct.SubRegionDisable = 0x0;
//  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1; // TEX=1, C=1, B=1 -> Write-Back, Write-Allocate
//  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
//  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
//  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;   
//  
//  // === 开启 Cache ===
//  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;   // 开启缓存
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE; // 开启缓冲

//  HAL_MPU_ConfigRegion(&MPU_InitStruct);

//  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
//}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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


```

## 测试效果图
### 连击模式功能测试
![[Pasted image 20260101103719.png]]

### 长按模式功能测试
![[Pasted image 20260101103923.png]]