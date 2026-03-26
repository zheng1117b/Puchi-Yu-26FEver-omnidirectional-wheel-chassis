/*
* 描述：机器人主逻辑任务，周期性调用底盘/子系统接口完成运动控制。
* 作者：扑哧宇
* 日期：2025-12-23
* 说明：
* 1. 本任务作为示例任务入口，周期性调用 `set_chassis_motor_speeds()` 完成底盘目标速度下发。
* 2. 避免在任务中做长时间阻塞或复杂计算，推荐将耗时操作放到独立任务或异步事件中。
*/

#include "my_drive.h"
#include "cmsis_os.h"


/* 机器人逻辑任务 */
void robot_logic(void const * argument)
{
  (void)argument;
    
  while (1)
  {
    set_chassis_motor_speeds();
    osDelay(5);
  }
}
