#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os.h"

/* Task handles */
extern osThreadId MotorCtrlTaskHandle;
extern osThreadId CommTaskHandle;

/* Task priorities */
#define MOTOR_CTRL_TASK_PRIORITY    osPriorityHigh
#define COMM_TASK_PRIORITY          osPriorityNormal

/* Task stack sizes */
#define MOTOR_CTRL_TASK_STACK       256
#define COMM_TASK_STACK             256

/* Signal/Notification bits */
#define MOTOR_CTRL_SIGNAL_UPDATE    (1 << 0)  /* TIM6 2ms tick signal */

/* Task function prototypes */
void MotorCtrlTask(void const *argument);
void CommTask(void const *argument);

/* Initialization called from main */
void AppTasks_Init(void);

#endif /* APP_TASKS_H */