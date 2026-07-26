#include "task2_control.h"

#define T2_CRUISE 110
#define T2_RAMP 20U
#define T2_ADVANCE 22U
#define T2_MIN_BRAKE 15U
#define T2_BRAKE_TIMEOUT 80U
#define T2_TURN_DEG 85.0f
#define T2_LEFT_AS_RIGHT_DEG 270.0f
#define T2_TURN_RPM 100
#define T2_REVERSE_RPM 75
#define T2_REVERSE_TICKS 50U

static void T2_log(Task2Control *c, const char *s) { if (c->io.log != 0) c->io.log(s); }
static bool T2_stopped(const Task2Control *c) { uint8_t i; for (i=0;i<4U;i++) { int32_t n=c->io.motors[i]->speedCountsPerSample; int32_t d=c->io.motors[i]->config.zeroSpeedDeadbandCounts; if ((n>d)||(n<-d)) return false; } return true; }
static void T2_follow(Task2Control *c)
{
    /*
     * Exactly one ramp step and one line-control update per 10 ms tick.
     * 20 steps take the target from 0 to 110 rpm in 0.2 seconds.
     */
    if (c->ramp < T2_RAMP) {
        c->ramp++;
    }
    c->io.setLineTrackingSpeed((int16_t) (((int32_t) T2_CRUISE *
        c->ramp) / T2_RAMP));
    c->io.updateLineTracking();
}

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
static void T2_stop(Task2Control *c) { c->io.setLineTrackingEnabled(false); c->io.resetLineTracking(); CarControl_stop(c->io.car); }
void Task2Control_reset(Task2Control *c) { c->state=TASK2_WAIT_INFO; c->pendingTurn=TASK2_TURN_NONE; c->outboundTurn=TASK2_TURN_NONE; c->ramp=0U;c->intersectionCount=0U;c->advance=0U;c->blank=0U;c->reverse=0U;c->settle=0U;c->brake=0U;c->lineSeen=false;c->intersectionLatched=false;c->returning=false;c->numberReceived=false;c->turnCompletedThisLeg=false;c->isUTurn=false;c->io.setLineTrackingEnabled(false);c->io.resetLineTracking();AngleTurnControl_cancel(c->io.angleTurn);CarControl_stop(c->io.car);c->io.setRedLed(false);c->io.setGreenLed(false); }
void Task2Control_init(Task2Control *c,const Task1Control_Config *io) { c->io=*io;Task2Control_reset(c); }
bool Task2Control_isActive(const Task2Control *c) { return c->state!=TASK2_WAIT_INFO; }
void Task2Control_update(Task2Control *c,TaskManager_Task task,bool press,bool release,bool number,Task2Control_Turn command,bool mpu,AngleTurnControl_Result result)
{
 uint8_t n;
 if(task!=TASK_MANAGER_TASK_2){if(c->state!=TASK2_WAIT_INFO)Task2Control_reset(c);return;}
 if(number){c->numberReceived=true;if(c->state==TASK2_WAIT_INFO)c->state=TASK2_WAIT_LOAD;}
 if(command!=TASK2_TURN_NONE && !c->returning &&
    !c->turnCompletedThisLeg && c->pendingTurn==TASK2_TURN_NONE &&
    c->state==TASK2_FOLLOW){
     c->pendingTurn=command;c->outboundTurn=command;
     T2_log(c,command==TASK2_TURN_LEFT?"TASK2 NEXT LEFT\r\n":"TASK2 NEXT RIGHT\r\n");
 }
 if(c->state==TASK2_WAIT_INFO)return;
 if(c->state==TASK2_WAIT_LOAD){if(press){c->ramp=0U;c->io.setLineTrackingEnabled(true);c->io.resetLineTracking();c->state=TASK2_FOLLOW;T2_log(c,"TASK2 STARTED\r\n");}return;}
 switch(c->state){
 case TASK2_FOLLOW: case TASK2_RETURNING:
   n=c->io.readActiveChannelCount();
   if(T2_cross(c,n) && c->pendingTurn!=TASK2_TURN_NONE){
       c->intersectionLatched=true;c->io.setLineTrackingEnabled(false);
       c->io.resetLineTracking();c->advance=0U;c->state=TASK2_ADVANCE;
       CarControl_setMotion(c->io.car,CAR_CONTROL_FORWARD,T2_CRUISE,100U);
       T2_log(c,"TASK2 CROSS ADVANCE\r\n");
       break;
   }
   c->intersectionLatched=(n>=6U);
   T2_follow(c);
   if(n>0U){c->lineSeen=true;c->blank=0U;}
   else if(c->lineSeen&&c->turnCompletedThisLeg){if(++c->blank>=3U){T2_stop(c);c->brake=0U;c->state=TASK2_BRAKE_END;}}
   break;
 case TASK2_ADVANCE: CarControl_setMotion(c->io.car,CAR_CONTROL_FORWARD,T2_CRUISE,100U);if(++c->advance>=T2_ADVANCE){CarControl_stop(c->io.car);c->brake=0U;c->state=TASK2_BRAKE_TURN;}break;
 case TASK2_BRAKE_TURN: if(++c->brake>=T2_MIN_BRAKE && T2_stopped(c)){bool left=c->pendingTurn==TASK2_TURN_LEFT;if(mpu&&AngleTurnControl_start(c->io.angleTurn,false,left?T2_LEFT_AS_RIGHT_DEG:T2_TURN_DEG,T2_TURN_RPM)){c->pendingTurn=TASK2_TURN_NONE;c->isUTurn=false;c->state=TASK2_TURNING;}else{c->state=TASK2_FAULT;c->io.setRedLed(true);}}else if(c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;c->io.setRedLed(true);}break;
 case TASK2_TURNING: if(result==ANGLE_TURN_RESULT_COMPLETED){c->ramp=0U;c->intersectionLatched=false;c->intersectionCount=0U;c->lineSeen=false;c->blank=0U;if(c->isUTurn){c->isUTurn=false;c->turnCompletedThisLeg=false;c->pendingTurn=(c->outboundTurn==TASK2_TURN_LEFT)?TASK2_TURN_RIGHT:TASK2_TURN_LEFT;}else{c->turnCompletedThisLeg=true;}c->io.setLineTrackingEnabled(true);c->io.resetLineTracking();c->state=c->returning?TASK2_RETURNING:TASK2_FOLLOW;}else if(result==ANGLE_TURN_RESULT_TIMEOUT||result==ANGLE_TURN_RESULT_FAULT){c->state=TASK2_FAULT;c->io.setRedLed(true);}break;
 case TASK2_BRAKE_END: if(++c->brake>=T2_MIN_BRAKE&&T2_stopped(c)){c->reverse=0U;CarControl_setMotion(c->io.car,CAR_CONTROL_BACKWARD,T2_REVERSE_RPM,100U);c->state=TASK2_REVERSE;}else if(c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;c->io.setRedLed(true);}break;
 case TASK2_REVERSE: CarControl_setMotion(c->io.car,CAR_CONTROL_BACKWARD,T2_REVERSE_RPM,100U);if(++c->reverse>=T2_REVERSE_TICKS){CarControl_stop(c->io.car);c->brake=0U;c->settle=0U;c->state=TASK2_FINAL_BRAKE;}break;
 case TASK2_FINAL_BRAKE: if(T2_stopped(c)){if(++c->settle>=5U){CarControl_emergencyStop(c->io.car);if(c->returning){c->state=TASK2_DONE;c->io.setGreenLed(true);}else{c->state=TASK2_WAIT_UNLOAD;c->io.setRedLed(true);}}}else c->settle=0U; if(++c->brake>=T2_BRAKE_TIMEOUT){c->state=TASK2_FAULT;c->io.setRedLed(true);}break;
 case TASK2_WAIT_UNLOAD: if(release){c->io.setRedLed(false);if(mpu&&AngleTurnControl_start(c->io.angleTurn,false,180.0f,T2_TURN_RPM)){c->state=TASK2_TURNING;c->returning=true;c->isUTurn=true;c->lineSeen=false;c->blank=0U;}else{c->state=TASK2_FAULT;c->io.setRedLed(true);}}break;
 default: break; }
}
