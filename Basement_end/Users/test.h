#ifndef TASK_H_1
#define TASK_H_1

//< 当前任务状态
typedef enum task_stat
{
    INIT = 0,
	MODE_ONE,
    MODE_TWO,
    MODE_THREE,
    MODE_FOUR,
    MODE_FIVE, 
}task_state_t;

void Move_swing(float M1_exp_angle, float M2_exp_angle);
int16_t coordinate_input(void);
void test_2(void);
void test_3(void);
void test_4(void);
void test_5(void);

#endif
