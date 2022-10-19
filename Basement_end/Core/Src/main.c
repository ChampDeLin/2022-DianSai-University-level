/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "Motor.control.h"
#include "test.h"
#include "stdbool.h"
#include "keyboard.h"
#include <math.h>
#include "pid.h"
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "decoding.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Start_pwm(void)
{
//  for (int i = 0; i <6; i++){
//    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,i*500);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//  
//    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

//    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

//    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//    
//    HAL_Delay(100);
//  }
  
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

void Stop_pwm(void)
{
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}
extern M1TypeDef M1;
extern M2TypeDef M2;
extern void test_1(void);
char ch;
float distance[8] = {0, -29, -20, -10, 0, 10, 20, 28.1};


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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void beep_on()
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

void beep_off()
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
}

  task_state_t task_state;
  
  char keyboard_result;
  uint8_t i,j;
  int16_t Mode3_distance[2];


/**
 * @brief 
 * 
 * @param aimangle 
 * @param balance_timelong 
 * @param ifclearparam 是否清除参数，默认清除，在做第3问时不清除
 * @return uint8_t 
 */
uint8_t runto_angle(float aimangle, uint32_t balance_timelong, uint8_t ifclearparam)
{
   bool finish;
   uint32_t sysTick = 0;

  uint32_t balance_begin_tick = 0;

  if (ifclearparam == 1) {
    PID_M1_Init();
    PID_M1_SetPoint(aimangle);	//X方向PID跟踪目标值sin
    PID_M1_SetKp(25/2.2 + 2);
    PID_M1_SetKi(0.325);
    PID_M1_SetKd((25/2.2+2)/4);
    // PID_M1_SetKi(0.79);
    // PID_M1_SetKd(3000);

    PID_M2_Init();
    PID_M2_SetPoint(aimangle);	//X方向PID跟踪目标值sin
    PID_M2_SetKp(25/2.2 + 2);  // 42 0.21
    PID_M2_SetKi(0.325);
    PID_M2_SetKd((25/2.2+2)/4);
    // PID_M2_SetKi(0.79);
    // PID_M2_SetKd(3000);
  }
  else {
    PID_M1_SetPoint(aimangle);	//X方向PID跟踪目标值sin
    PID_M2_SetPoint(aimangle);	//X方向PID跟踪目标值sin
  }

  while(1){
      if(HAL_GetTick() - sysTick >= 5){
        sysTick = HAL_GetTick();
              
        M1.CurX_angle = g_mpu6500_2.angle.Angle[0];
        M2.CurX_angle = g_mpu6500_2.angle.Angle[0];

        M1.PWM = PID_M1_PosLocCalc(M1.CurX_angle); 
        M2.PWM = PID_M2_PosLocCalc(M2.CurX_angle); 

        if(M1.PWM > DUTY_CYCLE_MAX)  M1.PWM =  DUTY_CYCLE_MAX;
        if(M1.PWM < -DUTY_CYCLE_MAX) M1.PWM = -DUTY_CYCLE_MAX;

        if(M2.PWM > DUTY_CYCLE_MAX)  M2.PWM =  DUTY_CYCLE_MAX;
        if(M2.PWM < -DUTY_CYCLE_MAX) M2.PWM = -DUTY_CYCLE_MAX;


        pwm_control(M1.PWM, M2.PWM);

        if(fabs(degree_2_X(M1.CurX_angle) - degree_2_X(aimangle)) < 2){
        //if(fabs(M1.CurX_angle - aimangle) < 0.45){
          //平衡
          if (balance_begin_tick == 0) {
            balance_begin_tick = HAL_GetTick();
          }
          else {
            if (HAL_GetTick() - balance_begin_tick >= balance_timelong) {
              
              return 1;
            }
          }

          beep_on();
        }
        else {
          balance_begin_tick = 0;
          beep_off();
        }
      }
  }

  return 0;
}

void task_1()
{
  uint8_t key_index = 0;
  char keyboard_result;
  char key_reserve[10];
  uint8_t i,j;
  float X_angle;

  //屏幕清零
  OLED_Clear();
  OLED_ShowString(10, 10, "Test Item 1", 16);
  OLED_Refresh();

  memset(key_reserve, 0, sizeof(key_reserve));

  while(1){

    //输入数字
    keyboard_result = matrix_keyboard_scan(&g_keyboard);

    HAL_Delay(100);
    
    if(keyboard_result != KEY_OK){

      if (keyboard_result == '0' || keyboard_result == '1' ||
          keyboard_result == '2' || keyboard_result == '3' ||
          keyboard_result == '4' || keyboard_result == '5' ||
          keyboard_result == '6' || keyboard_result == '7' ||
          keyboard_result == '8' || keyboard_result == '9' ||
          keyboard_result == '-') {

        key_reserve[key_index] = keyboard_result;
        key_index++; 

        if (key_index >= 10) {
          //清屏
          OLED_ShowString(10, 26, "                ", 16);
          key_index = 0;
          memset(key_reserve, 0, sizeof(key_reserve));
        }
        else {
          OLED_ShowChar(10+key_index*16, 26, keyboard_result, 16);
          OLED_Refresh();
        }
      }
      else if (keyboard_result == KEY_CANCEL) {
        Stop_pwm();
        task_state = INIT;
        return;
      }
    }
    else {
      //确认了，运行到某个点
    
      X_angle = atof(key_reserve);
            
      X_angle = X_2_degree(X_angle);

      if (runto_angle(X_angle, 3000, 1) == 1) {
        //显示完成
        beep_off();
        Stop_pwm();
        OLED_Clear();
        OLED_ShowString(10, 10, "Test 1 finish", 16);
        OLED_Refresh();
        task_state = INIT; 
        return;
      }
      else {
        //显示未完成
        OLED_Clear();
        OLED_ShowString(10, 10, "Test 1 fail", 16);
        OLED_Refresh();
      }

    }
  }
  
  task_state = INIT; 
}

void task_2()
{
  int i;
  float x_angle[8];

  OLED_Clear();
  OLED_ShowString(10, 10, "Test Item 2", 16);
  OLED_Refresh();

  for(i=0; i<8; i++){
    x_angle[i] = X_2_degree(distance[i]);
    if (i == 0 || i == 5) {
      runto_angle(x_angle[i], 3000, 1);
    }
    else {
      runto_angle(x_angle[i], 3000, 0);
    }
        //显示完成
//    beep_off();
//    Stop_pwm();
    OLED_Clear();
    OLED_ShowString(10, 10, "Test2 go...", 16);
    OLED_Refresh();
  }

  beep_off();
  OLED_Clear();
  OLED_ShowString(10, 10, "Test2 finish", 16);
  OLED_Refresh();
  Stop_pwm();

  HAL_Delay(500);
  task_state = INIT; 
  return;
}

void run_x2y_repeat(float X_angle1, float X_angle2, uint8_t runtimes, uint8_t iffirstclearparm)
{
  uint8_t i, ifclearparam;
  char tmpbuffer[32];
  
  for(i=0; i<runtimes; i++){
    
    sprintf(tmpbuffer, "Repeat %d", i+1);
    OLED_ShowString(10, 26, "                    ", 16);
    OLED_ShowString(10, 26, tmpbuffer, 16);
    OLED_Refresh();

    if (i == 0) {
      if (iffirstclearparm == 1) {
        ifclearparam = 1;
      }
      else {
        ifclearparam = 0;
      }
    }
    else {
      ifclearparam = 0;
    }
    
    if (runto_angle(X_angle1, 100, ifclearparam) == 1) {
      //显示完成
      beep_off();
      //Stop_pwm();          
    }
    else {
      //显示未完成          
    }        

    if (runto_angle(X_angle2, 100, 0) == 1) {
      //显示完成   
      beep_off();       
    }
    else {
      //显示未完成          
    }
  }
}

void task_F_1()
{
  uint8_t key_index1 = 0, key_index2 = 0;
  char keyboard_result, state;
  char key_reserve1[10], key_reserve2[10];
  uint8_t i,j;
  uint8_t ifclearparam;
  float X_angle1 = 0, X_angle2 = 0;

  //屏幕清零
  OLED_Clear();
  OLED_ShowString(10, 10, "Test F-1", 16);
  OLED_Refresh();

  memset(key_reserve1, 0, sizeof(key_reserve1));
  memset(key_reserve2, 0, sizeof(key_reserve2));

  while(1){

    //输入数字
    keyboard_result = matrix_keyboard_scan(&g_keyboard);

    HAL_Delay(100);
    
    if(keyboard_result != KEY_OK && keyboard_result != KEY_MARK){

      if (keyboard_result == '0' || keyboard_result == '1' ||
          keyboard_result == '2' || keyboard_result == '3' ||
          keyboard_result == '4' || keyboard_result == '5' ||
          keyboard_result == '6' || keyboard_result == '7' ||
          keyboard_result == '8' || keyboard_result == '9' ||
          keyboard_result == '-') {
        if(state != KEY_MARK){
          key_reserve1[key_index1] = keyboard_result;
          key_index1++; 

          if (key_index1 >= 10) {
            //清屏
            OLED_ShowString(10, 26, "                ", 16);
            key_index1 = 0;
            memset(key_reserve1, 0, sizeof(key_reserve1));
          }
          else {
            OLED_ShowChar(3+key_index1*16, 26, keyboard_result, 12);
            OLED_Refresh();
          }
        }else{
          key_reserve2[key_index2] = keyboard_result;
          key_index2++; 

          if (key_index2 >= 10) {
            //清屏
            OLED_ShowString(10, 26, "                ", 16);
            key_index2 = 0;
            memset(key_reserve2, 0, sizeof(key_reserve2));
            }
            else{
              OLED_ShowChar(3+(key_index2+key_index1+1)*16, 26, keyboard_result, 12);
              OLED_Refresh();
            }
          }
      }else if (keyboard_result == KEY_CANCEL) {
        
        Stop_pwm();
        task_state = INIT;
        return;
      }
        
    }else if(keyboard_result == KEY_MARK){
      state = KEY_MARK;
      OLED_ShowChar(3+(key_index1+1)*16, 26, keyboard_result, 12);
      OLED_Refresh();
    }else{
      //确认了，运行到某个点
    
      X_angle1 = atof(key_reserve1);
      X_angle2 = atof(key_reserve2);
            
      X_angle1 = X_2_degree(X_angle1);
      X_angle2 = X_2_degree(X_angle2);

      for(i=0; i<4; i++){

        if (i == 0) {
          ifclearparam = 1;
        }
        else {
          ifclearparam = 0;
        }
        
        if (runto_angle(X_angle1, 3000, ifclearparam) == 1) {
          //显示完成
          beep_off();
          //Stop_pwm();
          OLED_Clear();
          OLED_ShowString(10, 10, "Test3 go...", 16);
          OLED_Refresh();
        }
        else {
          //显示未完成
          OLED_Clear();
          OLED_ShowString(10, 10, "Test 3 fail", 16);
          OLED_Refresh();
          task_state = INIT; 
          return;
        }

        if (runto_angle(X_angle2, 3000, 0) == 1) {
          //显示完成
          beep_off();
          //Stop_pwm();
          OLED_Clear();
          OLED_ShowString(10, 10, "Test3 go...", 16);
          OLED_Refresh();
        }
        else {
          //显示未完成
          OLED_Clear();
          OLED_ShowString(10, 10, "Test 3 fail", 16);
          OLED_Refresh();
          task_state = INIT; 
          return;
        }
      }
      beep_off();
      Stop_pwm();
      break;
    }
  }
  
  OLED_Clear();
  OLED_ShowString(10, 10, "Test3 finish", 16);
  OLED_Refresh();

  HAL_Delay(500);
  task_state = INIT; 
  return;
}

pc_t g_pc;

void pc__init()
{
  memset(&g_pc, 0, sizeof(pc_t));
  g_pc.huart = &huart3;
  
}

void pc__open_to_receive(void)
{
    g_pc.rev_state = SEARCH_FRAMEHEAD;
    memset(&g_pc.rev_buffer, 0, sizeof(g_pc.rev_buffer));
    HAL_UART_AbortReceive_IT(g_pc.huart);
    __HAL_UART_CLEAR_IDLEFLAG(g_pc.huart);
    __HAL_UART_ENABLE_IT(g_pc.huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(g_pc.huart, g_pc.rx_dma_buffer_shadow, PC_RX_DMA_MAXSIZE);
}

void pc__stop_receive(void)
{
  HAL_UART_AbortReceive_IT(g_pc.huart);
  __HAL_UART_CLEAR_IDLEFLAG(g_pc.huart);
  __HAL_UART_DISABLE_IT(g_pc.huart, UART_IT_IDLE);
}

void pc__getkey(uint8_t key)
{
  if (g_pc.rev_state == SEARCH_FRAMEHEAD) {
    if (key == '%') {
      g_pc.rev_state = SEARCH_FRAMETAIL;
      g_pc.rev_buffer[0] = '%';
      g_pc.rev_write_index = 1;
    }
  }
  else if (g_pc.rev_state == SEARCH_FRAMETAIL) {
    g_pc.rev_buffer[g_pc.rev_write_index] = key;
    g_pc.rev_write_index++;

    if (key == '&') {
      //接收到了帧尾
      g_pc.rev_state = SEARCH_DATA;
      pc__stop_receive();
    }
    else {
      if (g_pc.rev_write_index >= 100) {
        g_pc.rev_state = SEARCH_FRAMEHEAD;
      }
    }

  }
}

void pc__rev_msg(uint8_t *revbuff, uint8_t revlength)
{
  uint8_t i;

  for (i = 0; i < revlength; i++) {
    pc__getkey(revbuff[i]);
  }
}

void task_F_2()
{
  int i;
  float x_angle[8];
  List* cmdlist;
  CommandConfig cmd;
  char tmpbuffer[32];
  uint32_t cmdrunstep;

  OLED_Clear();
  OLED_ShowString(10, 10, "Test F-2", 16);
  OLED_ShowString(10, 26, "wait cmd...", 16);
  OLED_Refresh();

  pc__open_to_receive();

  while (1) {
    
    //输入数字
    keyboard_result = matrix_keyboard_scan(&g_keyboard);

    if (keyboard_result == KEY_CANCEL) {
        Stop_pwm();
        task_state = INIT;
        return;      
    }

    if (g_pc.rev_state == SEARCH_DATA) {
      //如果接收到了数据，开始运行指令
      cmdlist = regex_string((const char*)g_pc.rev_buffer);

      while (1) {
        //输入数字
        keyboard_result = matrix_keyboard_scan(&g_keyboard);

        if (keyboard_result == KEY_CANCEL) {
            Stop_pwm();
            beep_off();
            task_state = INIT;
            return;
        }
        
        cmd = deal_command(cmdlist);

        if (cmd.flag == -1) {
          cmdlist = regex_string((const char*)g_pc.rev_buffer);
          continue;                    
        }
        else if (cmd.flag == 0) { //a#b        
          //执行a#b
          OLED_ShowString(10, 42, "                 ", 16);
          sprintf(tmpbuffer, "%d#%d", cmd.operate.config_a_b.a, cmd.operate.config_a_b.b);
          OLED_ShowString(10, 42, tmpbuffer, 16);
          OLED_Refresh();
          if (cmdrunstep == 0 ) {
            if (runto_angle(X_2_degree(cmd.operate.config_a_b.a), cmd.operate.config_a_b.b*1000, 1) == 1) {
              
            }
            else {            
            }
          }
          else {
            if (runto_angle(X_2_degree(cmd.operate.config_a_b.a), cmd.operate.config_a_b.b*1000, 0) == 1) {
              
            }
            else {            
            }
          }
          cmdrunstep++;
        }
        else if (cmd.flag == 1) { //c@d#e
          //执行a#b
          OLED_ShowString(10, 42, "                 ", 16);
          sprintf(tmpbuffer, "%d@%d#%d", cmd.operate.config_c_d_e.c, 
            cmd.operate.config_c_d_e.d, cmd.operate.config_c_d_e.e);
          OLED_ShowString(10, 42, tmpbuffer, 16);
          OLED_Refresh();

          if (cmdrunstep == 0) {
            run_x2y_repeat(X_2_degree(cmd.operate.config_c_d_e.c), 
              X_2_degree(cmd.operate.config_c_d_e.d), 
              cmd.operate.config_c_d_e.e, 1);
          }
          else {
            run_x2y_repeat(X_2_degree(cmd.operate.config_c_d_e.c), 
              X_2_degree(cmd.operate.config_c_d_e.d), 
              cmd.operate.config_c_d_e.e, 0);
          }
          
          cmdrunstep++;
        }
      }


      Stop_pwm();
  	  beep_off();
      task_state = INIT;
      return;
    }
    HAL_Delay(50);
  }


}
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
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  beep_off();
  OLED_Init();
  mpu6500__init(&g_mpu6500_2, &huart2);
  mpu6500__open_to_receive(&g_mpu6500_2);
  imu_calibration_to_zero();///< 先进行陀螺仪校准
  beep_on();
  Motor_init();///< 对电机相关参数进行初始化
  keyboard_init();///< 矩阵键盘初始
  pc__init();

  OLED_Clear();
  OLED_ShowString(10, 10, "Select Item", 16);
  OLED_Refresh();
  HAL_Delay(100);
  beep_off();
  
  #if 0
  while (1) {
    keyboard_result = matrix_keyboard_scan(&g_keyboard);
    if (keyboard_result != 0) {
      OLED_ShowChar(10, 10, ' ', 16);
      OLED_ShowChar(10, 10, keyboard_result, 16);
      OLED_Refresh();
    }
	     
  }
  #endif



  // Stop_pwm();	
  Start_pwm();

    // pwm_control(4000, 4000);
    // while(1);
  //runto_angle(X_2_degree(25), 5000);
  //while(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // if(task_state = INIT){

    // }

    keyboard_result = matrix_keyboard_scan(&g_keyboard);
    if(keyboard_result!=0){
      ch = keyboard_result;
    }
    switch(keyboard_result){
      case '1': 
        task_state = MODE_ONE;
        task_1();
        if(task_state == INIT){
          OLED_Clear();
          OLED_ShowString(10, 10, "Select Item", 16);
          OLED_Refresh();
        }
        break;
      case '2': 
        task_state = MODE_TWO;
        task_2();
        if(task_state == INIT){
          OLED_Clear();
          OLED_ShowString(10, 10, "Select Item", 16);
          OLED_Refresh();
        }
        break;
      case '3': 
        task_state = MODE_THREE;
        task_F_1();
        if(task_state == INIT){
          OLED_Clear();
          OLED_ShowString(10, 10, "Select Item", 16);
          OLED_Refresh();
        }
        break;
      case '4': 
        task_state = MODE_FOUR;
        break;
      case '5': 
        task_state = MODE_THREE;
        task_F_2();
        if(task_state == INIT){
          OLED_Clear();
          OLED_ShowString(10, 10, "Select Item", 16);
          OLED_Refresh();
        }
        break;
        task_state = MODE_FIVE;
        break;
    }
    HAL_Delay(150);    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
