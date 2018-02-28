/****************************************************************************
*
*   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
* @file px4_simple_app.c
* Minimal application example for PX4 autopilot
*
* @author Example User <mail@example.com>
*/



#include "rc_calibration.h"


extern "C" __EXPORT int rc_calibration_main(int argc, char *argv[]);

const static char * SWITCH_EXIT_TIMES = "10";        //在非后台模式下退出程序需要连续拨动的次数

const static hrt_abstime time_interval = 1000000;   //变化最大时间间隔 微妙

static bool task_should_stop;


struct switch_event events[] = {
	{ rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL,NULL,6,0,-1,0,NULL ,mag_calibration }
};


struct manual_control_setpoint_s manual_data;



static void usage()
{
	warnx("rc_calibration [-p][-D] start");
	warnx("options:");
	warnx("    -p    (print armed info)");
	warnx("    -D    (to be daemon)");
}





static void print_armed_info()
{
	int actuator_armed_sub_fd;
	struct actuator_armed_s _armed_data;
	actuator_armed_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	printf("--------------------------------\n");
	if (OK == orb_copy(ORB_ID(actuator_armed), actuator_armed_sub_fd, &_armed_data))
	{
		if (_armed_data.armed == true)
		{
			printf("system is Armed\n");
		}
		else
		{
			printf("system is Disarmed\n");
		}
	}
	else
	{
		printf("orb copy error %d\n", actuator_armed_sub_fd);
	}

	orb_unsubscribe(actuator_armed_sub_fd);
	printf("--------------------------------\n");
}



//关联开关类型和manual中的开关数据
static void relation_event_switch(int event_num)
{
	int i;
	int switch_type;
	for (i = 0; i < event_num; i++)
	{
		switch_type = events[i].switch_type;
		if (events[i].switch_ptr != NULL)
		{
			continue;
		}
		switch (switch_type)
		{
		case rc_channels_s::RC_CHANNELS_FUNCTION_MODE:
			events[i].switch_ptr = &manual_data.mode_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE:
			events[i].switch_ptr = &manual_data.rattitude_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL:
			events[i].switch_ptr = &manual_data.posctl_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_RETURN:
			events[i].switch_ptr = &manual_data.return_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_LOITER:
			events[i].switch_ptr = &manual_data.loiter_switch;
			break;
		case  rc_channels_s::RC_CHANNELS_FUNCTION_ACRO:
			events[i].switch_ptr = &manual_data.acro_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD:
			events[i].switch_ptr = &manual_data.offboard_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH:
			events[i].switch_ptr = &manual_data.kill_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH:
			events[i].switch_ptr = &manual_data.arm_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION:
			events[i].switch_ptr = &manual_data.transition_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_GEAR:
			events[i].switch_ptr = &manual_data.gear_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_STAB:
			events[i].switch_ptr = &manual_data.stab_switch;
			break;
		case rc_channels_s::RC_CHANNELS_FUNCTION_MAN:
			events[i].switch_ptr = &manual_data.man_switch;
			break;
		default:
			events[i].switch_ptr = NULL;
		}
	}
}


static void reset_event(struct switch_event *event_i)
{
	event_i->now_times = 0;
	event_i->prv_status = (int)*event_i->switch_ptr;
	event_i->last_check_time = 0;
}


int task_fun(int argc, char *argv[])
{
	int exit_switch_times = 0;
	int poll_ret = -1;
	bool check_exit = false, armed_status = true, out_info = false;
	int event_num = sizeof(events) / sizeof(struct switch_event);
	int manual_sub_fd = -1, armed_sub_fd = -1;

	if (argc == 0)
	{
		exit_switch_times = atoi(argv[0]);
		check_exit = true;
		out_info = true;
	}



	relation_event_switch(event_num);

	manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	armed_sub_fd = orb_subscribe(ORB_ID(actuator_armed));

	orb_set_interval(manual_sub_fd, 40);   //限定更新时间间隔为50ms
	orb_set_interval(armed_sub_fd, 500);   //限定更新时间间隔为500ms

	px4_pollfd_struct_t fds[] = {
		{ manual_sub_fd,NULL,POLLIN,0,NULL },
		{ armed_sub_fd,NULL,POLLIN,0,NULL },
	};

	int poll_fd_num = sizeof(fds) / sizeof(px4_pollfd_struct_t);


	while (!task_should_stop)
	{
		if (armed_status)
		{
			//已布防   则减小调用频率 等待解除布放
			struct actuator_armed_s armed_st;
			sleep(1);
			if (OK!= orb_copy(ORB_ID(actuator_armed), armed_sub_fd, &armed_st))
			{
			continue;
			}
			armed_status = armed_st.armed;
			
			if (armed_status)
			{
				if (out_info)
				{
					PX4_INFO("wait to disarmed\n");
				}
			}
			else
			{
				if (out_info)
				{
					PX4_INFO("system is Disarmed\n");
				}
			}
			continue;
		}
		poll_ret = px4_poll(&fds[0], poll_fd_num, 1500);

		if (poll_ret == 0)
		{
			//遥控器超过1.5s没有更新(遥控器断开连接)清空所有的开关连续计数器
			//........  遥控器离线判断
			for (int i = 0; i < event_num; i++)
			{
				events[i].prv_status = -1;
				events[i].now_times = 0;
				events[i].last_check_time = 0;
			}
			if (out_info)
			{
				PX4_ERR("Timeout return value from poll(): %d", poll_ret);
			}
		}
		else if (poll_ret < 0)
		{
			if (out_info)
			{
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			return 1;
		}
		else
		{
			for (int k = 0; k < poll_fd_num; k++)
			{
				if (fds[k].revents & POLLIN)
				{
					//收到遥控器数据更新
					if (fds[k].fd == manual_sub_fd)
					{
						memset(&manual_data, 0, sizeof(manual_data));

						orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &manual_data);

						for (int i = 0; i < event_num; i++)
						{
							hrt_abstime now_time = hrt_absolute_time();
							bool switch_changed = false;

							if ((int)*events[i].switch_ptr != events[i].prv_status)
							{
								//监测到开关变动

								if (events[i].prv_status == -1)
								{
									events[i].prv_status = (int)*events[i].switch_ptr;
								}
								else
								{
									if (events[i].last_check_time == 0)
									{
										//连续计数开始
										events[i].last_check_time = now_time;
										events[i].now_times = 1;
										events[i].prv_status = (int)*events[i].switch_ptr;
									}
									else
									{
										events[i].now_times++;
										events[i].prv_status = (int)*events[i].switch_ptr;
									}
									switch_changed = true;
								}
							}
							//如果本次监测时间 距离上次监测时间超过最大连续时间间隔则重置连续状态
							if (now_time - events[i].last_check_time > time_interval)
							{
								//重置计数器
								if (events[i].now_times > 0)
								{
									//判断是否达到触发条件触发
									if (events[i].now_times >= events[i].switch_series)
									{
										if (check_exit&&events[i].now_times >= exit_switch_times)
										{
											//超过最大触发次数 退出 task
											for (int j = 0; j < event_num; j++)
											{
												events[j].prv_status = -1;
												events[j].now_times = 0;
												events[j].last_check_time = 0;
											}
											orb_unsubscribe(armed_sub_fd);
											orb_unsubscribe(manual_sub_fd);
											return 0;
										}
										if (out_info)
										{
											PX4_INFO("tirgge ----%d----   %d\n", events[i].now_times, events[i].switch_type);									
										}										
										//线程启动并分离
										thread_run(&events[i]);
									}
									if (out_info)
									{
										PX4_INFO("reset %d switch ------\n", events[i].switch_type);									
									}
								}
								reset_event(&events[i]);
							}
							else
							{
								if (switch_changed)
								{
									events[i].last_check_time = now_time;
									if (out_info)
									{
										PX4_INFO("now times = %d\n", events[i].now_times);
									}
								}
							}
						}
					}
					else if (fds[k].fd == armed_sub_fd)
					{
						//布放状态发生变化
						//PX4_INFO("armed copy");
						struct actuator_armed_s armed_data;
						orb_copy(ORB_ID(actuator_armed), armed_sub_fd, &armed_data);
						if (armed_data.armed)
						{
							for (int j = 0; j < event_num; j++)
							{
								events[j].prv_status = -1;
								events[j].now_times = 0;
								events[j].last_check_time = 0;
							}
						}
						armed_status = armed_data.armed;
						break;
					}
				}
			}
		}
	}
	return 0;
}





static void start(bool be_daemon)
{
	task_should_stop = false;
	if (be_daemon)
	{

		px4_task_spawn_cmd("rc_daemon",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT,
			2000,
			task_fun,
			NULL);
	}
	else
	{
		task_fun(0, (char **)&SWITCH_EXIT_TIMES);
	}

}


static void stop()
{
	task_should_stop = true;
}




int rc_calibration_main(int argc, char *argv[])
{
	int ch;
	bool be_daemon = false;
	while ((ch = getopt(argc, argv, ":pD")) != EOF)
	{
		switch (ch)
		{
		case 'p':
			print_armed_info();
			break;
		case 'D':
			be_daemon = true;
			break;
		default:
			usage();
			optind = 0;
			exit(1);
		}
	}

	if (optind >= argc)
	{
		exit(1);
	}

	const char *verb = argv[optind];
	if (0 == strcmp(verb, "start"))
	{
		PX4_INFO("call start...");
		start(be_daemon);
	}
	else if (0 == strcmp(verb, "stop"))
	{
		stop();
		PX4_INFO("call stop...");
	}
	else
	{
		usage();
	}


	return 0;
}
