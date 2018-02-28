#include "rc_calibration.h"


pthread_t thread_run(struct switch_event *event)
{
	pthread_t thread_id;
	if (0 != pthread_create(&thread_id, NULL,event->fun,event->data))
	{
		return 0;
	}
	pthread_detach(thread_id);
	return thread_id;
}


void *mag_calibration(void *arg)
{
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));
	orb_advert_t fd = orb_advertise(ORB_ID(vehicle_command), &cmd);

	cmd.param2 = 1.0;
	cmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;

	orb_publish(ORB_ID(vehicle_command), fd, &cmd);
	orb_unadvertise(fd);
	return NULL;
}