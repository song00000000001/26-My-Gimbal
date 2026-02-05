#include "internal.h"
#include "global_data.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"
#include "robot_config.h"

void LaunchCtrl(void *arg)
{
    Motor_CAN_COB Tx_Buff,Tx_Buff1;
 
    Debugger={
        .enable_debug_mode=0,
    };
	
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    uint32_t main_task_now = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();

        Remote_Ctrl_Snapshot_Copy(&FS_I6X_Snap, &FS_I6X);
        
        // 处理遥控器连接状态及模式切换
        if (FS_I6X_Snap.Status != ESTABLISHED) {

        }
        else{
            if(FS_I6X_Snap.S1==SW_UP){
                
            }
            if(FS_I6X_Snap.S1==SW_DOWN){
                
            }
            if(FS_I6X_Snap.S1==SW_MID){
                
            }
           }
        
        //记录任务剩余栈空间
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.LaunchCtrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}


