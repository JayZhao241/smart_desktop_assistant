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
#include "adc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdint.h>
#include <stdio.h>
#include "touch.h"


#define V25 0.76f
#define AVG_SLOPE 0.0025f
#define SPI_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define SPI_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define GRAPH_POINTS 100  // 趋势图的点数
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
static uint32_t g_fac_us=168; //微秒延时的倍乘数
uint32_t pwm_val=0;
uint16_t adc_val=0;
uint16_t temp_adc=0;
float temp_history[GRAPH_POINTS]; // 存储历史温度的数组
uint8_t temp_index = 0;           // 当前记录到的位置
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t W25QXX_ReadID(void);
void W25QXX_Read(uint32_t addr, uint8_t *pBuffer, uint16_t len);
void W25QXX_Write(uint32_t addr, uint8_t *pBuffer, uint16_t len);
void W25QXX_EraseSector(uint32_t addr);
void UI_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t percent, uint16_t color);
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
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        float max_temp_record=0.0f; //历史最高温
  float current_temp=0.0f; //目前温度
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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_FSMC_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  tp_dev.init();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  W25QXX_Read(0x000000, (uint8_t*)&max_temp_record, 4);
  
  // 如果是第一次使用，读取到的可能是非法值 (NaN)
  if (max_temp_record < 0 || max_temp_record > 100) {
      max_temp_record = 0.0f; 
  }
  
  printf("History Max Temp: %.2f\r\n", max_temp_record);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // 启动背光调试

  printf("LCD初始化中……\n");
  lcd_init();
  printf("初始化完成\n");

  

  //读取flash ID
  printf("Reading W25Q128 ID...\r\n");
  uint16_t flash_id = W25QXX_ReadID();
  printf("Flash ID: 0x%X\r\n", flash_id);

  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 6000);
  lcd_clear(WHITE);

  //在屏幕上显示ID
  lcd_show_string(30, 220, 200, 16, 16, "Flash ID:", BLACK);
  if (flash_id == 0xEF17) 
  {
      lcd_show_string(110, 220, 200, 16, 16, "0xEF17 (OK)", BLUE);
  }
  else
  {
      // 如果显示 0x0000 或 0xFFFF 说明 SPI 没通
      lcd_show_xnum(110, 220, flash_id, 4, 16, 0, RED); 
  }
  HAL_Delay(2000);

  lcd_clear(WHITE);
  // lcd_show_string(30, 50, 200, 16, 16, "Explorer F407", RED);
  // lcd_show_string(30, 70, 200, 16, 16, "VSCode + CMake", BLUE);
  HAL_Delay(1000);

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // //设置初始化时间
  // sTime.Hours = 15;
  // sTime.Minutes = 50;
  // sTime.Seconds = 00;
  // sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; 
  // sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  // if (HAL_RTC_SetTime(&hrtc, &sTime,RTC_FORMAT_BIN))
  // {
  //   Error_Handler();
  // }
  // sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  // sDate.Month = 01;
  // sDate.Date = 13;
  // sDate.Year = 26;
  // if (HAL_RTC_SetDate(&hrtc, &sDate,RTC_FORMAT_BIN))
  // {
  //   Error_Handler();
  // }
  // HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x5050);
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x5050)
  {

    //设置初始化时间
    sTime.Hours = 15;
    sTime.Minutes = 36;
    sTime.Seconds = 00;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; 
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime,RTC_FORMAT_BIN))
    {
      Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
    sDate.Month = 01;
    sDate.Date = 13;
    sDate.Year = 26;
    if (HAL_RTC_SetDate(&hrtc, &sDate,RTC_FORMAT_BIN))
    {
      Error_Handler();
    }
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x5050);
  }
  else
  {
    printf("RTC已在运行，跳过时间设置\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    // 重置功能
    //按钮显示
    lcd_draw_rectangle(150, 280, 220, 310, RED);
    lcd_show_string(160, 285, 200, 16, 16, "RESET", RED);
    //触摸交互
    tp_dev.scan(0); // 扫描触摸屏
    if (tp_dev.sta & TP_PRES_DOWN)  // 如果按下
    {
      // 检查触摸点是否在按钮区域内
      if (tp_dev.x[0] > 150 && tp_dev.x[0] < 220 && tp_dev.y[0] > 280 && tp_dev.y[0] < 310)
      {
        // 1. 执行清除逻辑
        max_temp_record = 0.0f;
        W25QXX_EraseSector(0x000000); // 擦除 Flash 记录

        // 2.视觉反馈
        lcd_clear(RED);
        HAL_Delay(100);
        lcd_clear(WHITE);
      }
    }

    

    //背光调节功能
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10)==HAL_OK) {
        adc_val=HAL_ADC_GetValue(&hadc3); //读取传感器数值
    } //指定adc轮询
    HAL_ADC_Stop(&hadc3);
    //接下来实现adc越小，背光越亮
    if (adc_val>4000)  {
      adc_val=4000;
    }
    pwm_val=10000-(adc_val*5)/2;
    if (pwm_val<1000) {
      pwm_val=1000;
    }
    if (pwm_val>8000){
      pwm_val=8000;
    }
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_val);



    //光照强度显示功能
    uint8_t light_percent = 100 - (adc_val * 100 / 4095);
    lcd_show_string(30, 50, 200, 16, 16, "Light Intensity:", BLACK);
    UI_DrawProgressBar(30, 70, 180, 15, light_percent, YELLOW);
    


    //内部温度监控功能
    //接下来读取内部温度
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK) {
        temp_adc=HAL_ADC_GetValue(&hadc1); //读取传感器数值
        float temp_vol=(float)temp_adc*(3.3f/4096.0f); //换算成温度电压值
        float chip_temp=(temp_vol-V25)/AVG_SLOPE+25; //计算温度
        float safety_threshold = 50.0f; //安全温度
        uint16_t temp_color;

        //记录历史最高温
        current_temp=chip_temp;
        if (current_temp>max_temp_record)
        {
          max_temp_record=current_temp;
          W25QXX_Write(0x000000, (uint8_t *)&max_temp_record, 4);
          printf("New Record Saved:%.2f\r\n",max_temp_record);
        }
        //自动报警系统
        if (current_temp>safety_threshold)
        {
          HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET); //蜂鸣器
          HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); //灯亮
          temp_color = RED;
        }
        else if (current_temp>42) {
          temp_color=MAGENTA;
        }
        else {
          HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET); //停止鸣叫
          HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //灯灭
        }
        //在屏幕上显示
        lcd_show_string(30, 240, 200, 16, 16, "Max Record:", BLACK);
        lcd_show_num(120, 240, (int)max_temp_record, 2, 16, MAGENTA);
        lcd_show_string(140, 240, 200, 16, 16, "C", MAGENTA);
        lcd_show_string(30, 90, 200, 16, 16, "Temp:",BLACK);
        lcd_show_num(120, 90, (uint32_t) chip_temp, 2, 16, RED);
        lcd_show_string(140, 90, 200, 16, 16, " C", RED);
    } 
     HAL_ADC_Stop(&hadc1);
    



    //日期显示功能
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    char date_str[30]; // 定义一个足够长的字符数组
    
    // 格式化字符串
    // %02d: 2位数字，不足补0
    // sDate.Year 只有后两位(比如23)，所以前面手动加 "20" 变成 "2023"
    sprintf(date_str, "20%02d-%02d-%02d", sDate.Year, sDate.Month, sDate.Date);
    
    // // 显示在屏幕上 (假设显示在时间上方，y=150的位置)
    // // 字体用 16号小字，颜色用黑色
    lcd_show_string(60, 150, 200, 16, 16, date_str, BLACK);
    
    char time_str[20];
    sprintf(time_str, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
    lcd_show_string(50, 180,200, 24, 24, time_str, BLUE);



    // 屏幕校准功能
    static uint16_t key_up_press_count = 0;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) // 如果按键按下
    {
        key_up_press_count++;
        if (key_up_press_count > 10) { // 按住超过 0.5 秒开始显示
            lcd_show_string(30, 10, 200, 16, 16, "Calibrating...", RED);
            lcd_fill(30, 30, 30 + (key_up_press_count * 2), 35, RED); 
        }
      }
    else {
        if (key_up_press_count > 0) // 如果之前显示过提示，松开时清除它
        {
            lcd_fill(30, 10, 230, 40, WHITE); // 擦除提示区域
        }
        key_up_press_count = 0; // 计数归零
      }
    if (key_up_press_count >= 40) 
    {
        key_up_press_count = 0; // 触发前先归零
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET); // 蜂鸣器响一声提示进入
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
        
        printf("Entering Calibration from Loop...\r\n");
        
        tp_adjust(); // 执行校准（这个函数是阻塞的，直到校准完成）
        
        // 【关键】校准完后，屏幕被 tp_adjust 弄乱了，必须重新绘制你的 UI
        lcd_clear(WHITE);
        lcd_draw_rectangle(150, 280, 220, 310, RED);
        lcd_show_string(160, 285, 200, 16, 16, "RESET", RED);
    }


    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(50 + (adc_val / 40));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//SPI读写函数
uint8_t SPI1_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    // 发送 tx_data，同时接收 rx_data，超时 100ms
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 100);
    return rx_data;
}

//读取芯片ID
uint16_t W25QXX_ReadID(void)
{
    uint16_t temp = 0;
    
    SPI_CS_LOW();             // 1. 拉低片选，选中芯片
    SPI1_ReadWriteByte(0x90); // 2. 发送命令：读ID (0x90)
    SPI1_ReadWriteByte(0x00); // 3. 发送 24位 假地址 (0,0,0)
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    
    // 4. 接收数据
    temp |= SPI1_ReadWriteByte(0xFF) << 8; // 读高8位 (厂家ID)
    temp |= SPI1_ReadWriteByte(0xFF);      // 读低8位 (设备ID)
    
    SPI_CS_HIGH();            // 5. 拉高片选，结束通信
    return temp;
}
// 写使能Flash
void W25QXX_WriteEnable(void)
{
    SPI_CS_LOW();
    SPI1_ReadWriteByte(0x06); // 发送写使能命令
    SPI_CS_HIGH();
}
//等待芯片作用完毕
void W25QXX_WaitBusy(void)
{
    uint8_t status = 0;
    do {
        SPI_CS_LOW();
        SPI1_ReadWriteByte(0x05); // 读状态寄存器1
        status = SPI1_ReadWriteByte(0xFF);
        SPI_CS_HIGH();
    } while (status & 0x01); // BIT0 为 1 表示忙
}
//擦除一个扇区
void W25QXX_EraseSector(uint32_t addr)
{
    W25QXX_WriteEnable();
    W25QXX_WaitBusy();
    SPI_CS_LOW();
    SPI1_ReadWriteByte(0x20); // 扇区擦除命令
    SPI1_ReadWriteByte((uint8_t)(addr >> 16)); // 发送24位地址
    SPI1_ReadWriteByte((uint8_t)(addr >> 8));
    SPI1_ReadWriteByte((uint8_t)addr);
    SPI_CS_HIGH();
    W25QXX_WaitBusy(); // 等待擦除完成
}
//在指定地址写入数据
void W25QXX_Write(uint32_t addr, uint8_t *pBuffer, uint16_t len)
{
    W25QXX_EraseSector(addr); // 先擦除
    W25QXX_WriteEnable();
    SPI_CS_LOW();
    SPI1_ReadWriteByte(0x02); // 页编程命令
    SPI1_ReadWriteByte((uint8_t)(addr >> 16));
    SPI1_ReadWriteByte((uint8_t)(addr >> 8));
    SPI1_ReadWriteByte((uint8_t)addr);
    for(uint16_t i=0; i<len; i++) SPI1_ReadWriteByte(pBuffer[i]);
    SPI_CS_HIGH();
    W25QXX_WaitBusy();
}
//读取数据
void W25QXX_Read(uint32_t addr, uint8_t *pBuffer, uint16_t len)
{
    SPI_CS_LOW();
    SPI1_ReadWriteByte(0x03); // 读取命令
    SPI1_ReadWriteByte((uint8_t)(addr >> 16));
    SPI1_ReadWriteByte((uint8_t)(addr >> 8));
    SPI1_ReadWriteByte((uint8_t)addr);
    for(uint16_t i=0; i<len; i++) pBuffer[i] = SPI1_ReadWriteByte(0xFF);
    SPI_CS_HIGH();
}

void delay_us(uint32_t nus)
 {
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;   // LOAD的值 
    ticks = nus * g_fac_us;            // 需要的节拍数 
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;   // 这里注意 SysTick 是向下递减计数的 
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) break;  // 时间超过/等于要延时的时间,则退出 
        }
    }
}



// 绘制进度条函数
void UI_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t percent, uint16_t color)
{
    if (percent > 100) percent = 100;
    // 1. 画外框
    lcd_draw_rectangle(x, y, x + w, y + h, BLACK);
    // 2. 清除背景（白色）
    lcd_fill(x + 1, y + 1, x + w - 1, y + h - 1, WHITE);
    // 3. 填充进度
    if (percent > 0) {
        uint16_t fill_w = (w - 2) * percent / 100;
        lcd_fill(x + 1, y + 1, x + fill_w, y + h - 1, color);
    }
}



// 绘制温度变化图函数
void UI_DrawTrendGraph(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    // 1. 画坐标轴
    lcd_draw_line(x, y, x + w, y, BLACK);      // X轴
    lcd_draw_line(x, y, x, y - h, BLACK);      // Y轴
    
    // 2. 绘制数据点之间的连线
    for (uint8_t i = 0; i < GRAPH_POINTS - 1; i++)
    {
        // 计算当前点和下一个点的坐标
        // X坐标：起点 + (间距 * 索引)
        uint16_t x1 = x + (i * w / GRAPH_POINTS);
        uint16_t x2 = x + ((i + 1) * w / GRAPH_POINTS);
        
        // Y坐标映射 (高度范围 20~50度)
        float t1 = temp_history[i];
        float t2 = temp_history[i + 1];
        
        // 限幅处理
        if (t1 < 20) t1 = 20; if (t1 > 80) t1 = 80;
        if (t2 < 20) t2 = 20; if (t2 > 80) t2 = 80;

        uint16_t y1 = y - (uint16_t)((t1 - 20) * h / 60);
        uint16_t y2 = y - (uint16_t)((t2 - 20) * h / 60);

        // 如果数据有效（不为0），则画线
        if (t1 > 0 && t2 > 0) {
            lcd_draw_line(x1, y1, x2, y2, BLUE);
        }
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
