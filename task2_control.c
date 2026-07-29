#include "task2_control.h"

/* 所有 ticks/samples 参数均以 10 ms 为一个状态机周期。 */
#define T2_CRUISE 150
#define T2_RAMP 20U
#define T2_ADVANCE 16U
#define T2_MIN_BRAKE 15U
#define T2_BRAKE_TIMEOUT 80U
#define T2_TURN_DEG 85.0f
#define T2_TURN_RPM 100
#define T2_REVERSE_RPM 75
#define T2_REVERSE_TICKS 50U
#define T2_PRESET_TURN_CROSS 2U

/* 日志接口允许为空，便于状态机脱离串口单独测试。 */
static void T2_log(Task2Control *c, const char *s)
{
    if (c->io.log != 0) {
        c->io.log(s);
    }
}

/* 四个车轮的编码器速度都进入死区后，才认为车体停稳。 */
static bool T2_stopped(const Task2Control *c)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        int32_t counts = c->io.motors[i]->speedCountsPerSample;
        int32_t deadband =
            c->io.motors[i]->config.zeroSpeedDeadbandCounts;
        if ((counts > deadband) || (counts < -deadband)) {
            return false;
        }
    }
    return true;
}

/* 从 0 线性加速到巡航速度，并执行一次灰度巡线更新。 */
static void T2_follow(Task2Control *c)
{
    /*
     * 每个 10 ms 周期只增加一级速度并更新一次巡线；
     * 20 级加速总计耗时 0.2 秒。
     */
    if (c->ramp < T2_RAMP) {
        c->ramp++;
    }
    c->io.setLineTrackingSpeed((int16_t) (((int32_t) T2_CRUISE *
        c->ramp) / T2_RAMP));
    c->io.updateLineTracking();
}

/* 八路全有效立即确认，否则六路以上需要连续两帧。 */
static bool T2_cross(Task2Control *c, uint8_t activeCount)
{
    if (activeCount == 8U) {
        c->intersectionCount = 0U;
        return true;
    }
    if (activeCount >= 6U) {
        if (c->intersectionCount < 255U) {
            c->intersectionCount++;
        }
        return c->intersectionCount >= 2U;
    }
    c->intersectionCount = 0U;
    return false;
}
/* 关闭巡线输出并让四轮进入零速闭环制动。 */
static void T2_stop(Task2Control *c)
{
    c->io.setLineTrackingEnabled(false);
    c->io.resetLineTracking();
    CarControl_stop(c->io.car);
}

/* 清空路线和阶段计数，回到等待视觉信息状态。 */
void Task2Control_reset(Task2Control *c) { c->state=TASK2_WAIT_INFO;c->pendingTurn=TASK2_TURN_NONE;c->outboundTurn=TASK2_TURN_NONE;c->selectedEndpoint=TASK_MANAGER_TASK2_ENDPOINT_AUTO;c->ramp=0U;c->intersectionCount=0U;c->outboundIntersectionsSeen=0U;c->advance=0U;c->blank=0U;c->reverse=0U;c->settle=0U;c->brake=0U;c->lineSeen=false;c->intersectionLatched=false;c->returning=false;c->numberReceived=false;c->turnCompletedThisLeg=false;c->isUTurn=false;c->io.setLineTrackingEnabled(false);c->io.resetLineTracking();AngleTurnControl_cancel(c->io.angleTurn);CarControl_stop(c->io.car); }
/* 保存底层接口并执行一次完整复位。 */
void Task2Control_init(Task2Control *c,const Task1Control_Config *io) { c->io=*io;Task2Control_reset(c); }
/* 离开 WAIT_INFO 后即认为 Task2 已开始占用车辆。 */
bool Task2Control_isActive(const Task2Control *c) { return c->state!=TASK2_WAIT_INFO; }

/* 根据按钮、端点、OpenMV 指令和转向结果推进一次 Task2 状态机。 */
void Task2Control_update(Task2Control *c,TaskManager_Task task,TaskManager_Task2Endpoint endpoint,bool press,bool release,bool number,Task2Control_Turn command,bool mpu,AngleTurnControl_Result result)
{
 uint8_t n;
 if(task!=TASK_MANAGER_TASK_2){if(c->state!=TASK2_WAIT_INFO)Task2Control_reset(c);return;}
 if(number)c->numberReceived=true;
 if(c->state==TASK2_WAIT_INFO){
     if(endpoint!=TASK_MANAGER_TASK2_ENDPOINT_AUTO||c->numberReceived)c->state=TASK2_WAIT_LOAD;
 }else if(c->state==TASK2_WAIT_LOAD&&endpoint==TASK_MANAGER_TASK2_ENDPOINT_AUTO&&!c->numberReceived){
     c->state=TASK2_WAIT_INFO;
 }
 if(command!=TASK2_TURN_NONE && !c->returning &&
    c->selectedEndpoint==TASK_MANAGER_TASK2_ENDPOINT_AUTO &&
    !c->turnCompletedThisLeg && c->pendingTurn==TASK2_TURN_NONE &&
    c->state==TASK2_FOLLOW){
     c->pendingTurn=command;c->outboundTurn=command;
     T2_log(c,command==TASK2_TURN_LEFT?"TASK2 NEXT LEFT\r\n":"TASK2 NEXT RIGHT\r\n");
 }
 if(c->state==TASK2_WAIT_INFO)return;
 if(c->state==TASK2_WAIT_LOAD){if(press){c->selectedEndpoint=endpoint;c->outboundIntersectionsSeen=0U;if(endpoint==TASK_MANAGER_TASK2_ENDPOINT_1){c->pendingTurn=TASK2_TURN_LEFT;c->outboundTurn=TASK2_TURN_LEFT;}else if(endpoint==TASK_MANAGER_TASK2_ENDPOINT_2){c->pendingTurn=TASK2_TURN_RIGHT;c->outboundTurn=TASK2_TURN_RIGHT;}c->ramp=0U;c->io.setLineTrackingEnabled(true);c->io.resetLineTracking();c->state=TASK2_FOLLOW;T2_log(c,endpoint==TASK_MANAGER_TASK2_ENDPOINT_AUTO?"TASK2 AUTO STARTED\r\n":"TASK2 ENDPOINT STARTED\r\n");}return;}
 switch(c->state){
 /* 去程/返程巡线；预设端点在第二个十字路口执行转向。 */
 case TASK2_FOLLOW: case TASK2_RETURNING:
   n=c->io.readActiveChannelCount();
   if(n<6U)c->intersectionLatched=false;
   if(!c->intersectionLatched&&T2_cross(c,n) && c->pendingTurn!=TASK2_TURN_NONE){
       if(!c->returning&&!c->turnCompletedThisLeg&&
          c->selectedEndpoint!=TASK_MANAGER_TASK2_ENDPOINT_AUTO){
           if(c->outboundIntersectionsSeen<255U)c->outboundIntersectionsSeen++;
           if(c->outboundIntersectionsSeen<T2_PRESET_TURN_CROSS){
               c->intersectionLatched=true;
               T2_follow(c);
               T2_log(c,"TASK2 PRESET CROSS PASSED STRAIGHT\r\n");
               break;
           }
       }
       c->intersectionLatched=true;c->io.setLineTrackingEnabled(false);
       c->io.resetLineTracking();c->advance=0U;c->state=TASK2_ADVANCE;
       CarControl_setMotion(c->io.car,CAR_CONTROL_FORWARD,T2_CRUISE,100U);
       T2_log(c,"TASK2 CROSS ADVANCE\r\n");
       break;
   }
   T2_follow(c);
   if(n>0U){c->lineSeen=true;c->blank=0U;}
   else if(c->lineSeen&&c->turnCompletedThisLeg){if(++c->blank>=3U){T2_stop(c);c->brake=0U;c->state=TASK2_BRAKE_END;}}
   break;
 /* 直行进入路口中心，然后停车。 */
 case TASK2_ADVANCE: CarControl_setMotion(c->io.car,CAR_CONTROL_FORWARD,T2_CRUISE,100U);if(++c->advance>=T2_ADVANCE){CarControl_stop(c->io.car);c->brake=0U;c->state=TASK2_BRAKE_TURN;}break;
 /* 等待车轮停稳，再用 MPU6050 启动 85 度定角转向。 */
 case TASK2_BRAKE_TURN: if(++c->brake>=T2_MIN_BRAKE && T2_stopped(c)){bool left=c->pendingTurn==TASK2_TURN_LEFT;if(mpu&&AngleTurnControl_start(c->io.angleTurn,left,T2_TURN_DEG,T2_TURN_RPM)){c->pendingTurn=TASK2_TURN_NONE;c->isUTurn=false;c->state=TASK2_TURNING;}else{c->state=TASK2_FAULT;}}else if(c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;}break;
 /* 处理普通路口转向或卸载后的 180 度掉头结果。 */
 case TASK2_TURNING: if(result==ANGLE_TURN_RESULT_COMPLETED){c->ramp=0U;c->intersectionLatched=false;c->intersectionCount=0U;c->lineSeen=false;c->blank=0U;if(c->isUTurn){c->isUTurn=false;c->turnCompletedThisLeg=false;c->pendingTurn=(c->outboundTurn==TASK2_TURN_LEFT)?TASK2_TURN_RIGHT:TASK2_TURN_LEFT;}else{c->turnCompletedThisLeg=true;}c->io.setLineTrackingEnabled(true);c->io.resetLineTracking();c->state=c->returning?TASK2_RETURNING:TASK2_FOLLOW;}else if(result==ANGLE_TURN_RESULT_TIMEOUT||result==ANGLE_TURN_RESULT_FAULT){c->state=TASK2_FAULT;}break;
 /* 终点制动，停稳后进入倒车调整。 */
 case TASK2_BRAKE_END: if(++c->brake>=T2_MIN_BRAKE&&T2_stopped(c)){c->reverse=0U;CarControl_setMotion(c->io.car,CAR_CONTROL_BACKWARD,T2_REVERSE_RPM,100U);c->state=TASK2_REVERSE;}else if(c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;}break;
 /* 固定速度倒车 0.5 秒。 */
 case TASK2_REVERSE: CarControl_setMotion(c->io.car,CAR_CONTROL_BACKWARD,T2_REVERSE_RPM,100U);if(++c->reverse>=T2_REVERSE_TICKS){CarControl_stop(c->io.car);c->brake=0U;c->settle=0U;c->state=TASK2_FINAL_BRAKE;}break;
 /* 连续多帧确认车轮静止后，等待卸载或结束返程。 */
 case TASK2_FINAL_BRAKE: if(T2_stopped(c)){if(++c->settle>=5U){CarControl_emergencyStop(c->io.car);if(c->returning){c->state=TASK2_DONE;}else{c->state=TASK2_WAIT_UNLOAD;}}}else c->settle=0U; if(++c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;}break;
 /* 卸载并释放按钮后统一右转 180 度，开始返程。 */
 case TASK2_WAIT_UNLOAD: if(release){if(mpu&&AngleTurnControl_start(c->io.angleTurn,false,180.0f,T2_TURN_RPM)){c->state=TASK2_TURNING;c->returning=true;c->isUTurn=true;c->lineSeen=false;c->blank=0U;}else{c->state=TASK2_FAULT;}}break;
 default: break; }
}
