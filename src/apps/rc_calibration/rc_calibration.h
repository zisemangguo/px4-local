#ifndef _RC_CALIBRATION_H_
#define _RC_CALIBRATION_H_
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <../drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>

typedef void *(*switch_fun_ptr)(void *arg);

struct switch_event
{
	int switch_type;                  //开关类型
	uint8_t *switch_ptr;              //事件关联的manual结构体中的开关数据指针
	int switch_series;                //触发事件需要连续改变的次数
	int now_times;                    //目前连续次数
	int prv_status;               //上一次开关状态
	hrt_abstime last_check_time;       //上一次状态监测的时间
	void *data;                      //线程外带数据
	switch_fun_ptr fun;                //触发事件的线程函数
};

void *mag_calibration(void *arg);

int task_fun(int argc, char *argv[]);

pthread_t thread_run(struct switch_event *event);

#endif
