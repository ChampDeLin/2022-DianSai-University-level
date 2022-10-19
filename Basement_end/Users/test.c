#include "Motor.control.h"
#include "stdio.h"
#include "math.h"
//#include "6050.h"
//#include "motor.pwm.h"
#include "pid.h"
#include "stdlib.h"
#include "gpio.h"
#include "test.h"
#include "task.h"
#include "keyboard.h"

extern M1TypeDef M1;
extern M2TypeDef M2;
extern TIM_HandleTypeDef htim1;




void Move_swing(float M1_exp_angle, float M2_exp_angle)
{
    bool finish;
    uint32_t sysTick = 0;
//    struct SAcc 	acc_test; //加�?�度
//    struct SGyro 	gyro_test;//角�?�度  
//    struct SAngle 	angle_test;//角度

    while(1){
        if(HAL_GetTick() - sysTick >= 5){
            sysTick = HAL_GetTick();
//            acc_test = g_mpu6500_2.acc;
//            gyro_test = g_mpu6500_2.gyro;
//            angle_test = g_mpu6500_2.angle;
//            
//            M1.CurX_angle = angle_test.Angle[0]; 
//            M2.CurX_angle = angle_test.Angle[1];
        
            PID_M1_SetPoint(M1_exp_angle);	//X方向PID跟踪目标值sin
            PID_M1_SetKp(85);
            PID_M1_SetKi(0.425);
            PID_M1_SetKd(3000);

            PID_M2_SetPoint(M2_exp_angle);	//X方向PID跟踪目标值sin
            PID_M2_SetKp(85);
            PID_M2_SetKi(0.425);
            PID_M2_SetKd(3000);

            M1.PWM = PID_M1_PosLocCalc(M1.CurX_angle); 
            M2.PWM = PID_M2_PosLocCalc(M2.CurX_angle); 

            if(M1.PWM > DUTY_CYCLE_MAX)  M1.PWM =  DUTY_CYCLE_MAX;
            if(M1.PWM < -DUTY_CYCLE_MAX) M1.PWM = -DUTY_CYCLE_MAX;

            if(M2.PWM > DUTY_CYCLE_MAX)  M2.PWM =  DUTY_CYCLE_MAX;
            if(M2.PWM < -DUTY_CYCLE_MAX) M2.PWM = -DUTY_CYCLE_MAX;


            pwm_control(M1.PWM, M2.PWM);

            finish = balance_judge(3000);
        }

            if(finish == true){
                break;
            }
    }
    
}



int16_t coordinate_input(void)
{
    uint8_t indexs = 0;
    char keyboard_result;
    char key_reserve[10];
    uint8_t i,j;
    float X_angle;

    while(1){
      keyboard_result = matrix_keyboard_scan(&g_keyboard);
      HAL_Delay(200);
      if(keyboard_result != '&'){
        key_reserve[indexs] = keyboard_result;
        indexs++; 
      }else{
      //   M1.SetX_angle = 0;
      //   M2.SetX_angle = 0;
        if(key_reserve[0] == '-'){
          for(j=1; j<indexs; j++){
            X_angle = X_angle+key_reserve[j]*pow(10, indexs-j-1);
      //      M2.SetX_angle = M2.SetX_angle+key_reserve[j]*pow(10, indexs-j-1);
          }
        }else{
          for(j=0; j<indexs; j++){
            X_angle = X_angle+key_reserve[j]*pow(10, indexs-j-1);
      //      M2.SetX_angle = M2.SetX_angle+key_reserve[j]*pow(10, indexs-j-1);
          }
        }
        X_angle = X_2_degree(X_angle);
        return X_angle;
      }
  }
}