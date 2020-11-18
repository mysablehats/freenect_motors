/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * Andrew Miller <amiller@dappervision.com>
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;

#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)

#ifndef _WIN32
  #include <unistd.h>
#else
  // Microsoft Visual C++ does not provide the <unistd.h> header, but most of
  // its contents can be found within the <stdint.h> header:
  #include <stdint.h>
  // except for the UNIX sleep() function that has to be emulated:
  #include <windows.h>
  // http://pubs.opengroup.org/onlinepubs/009695399/functions/sleep.html
  // According to the link above, the semantics of UNIX sleep() is as follows:
  // "If sleep() returns because the requested time has elapsed, the value
  //  returned shall be 0. If sleep() returns due to delivery of a signal, the
  //  return value shall be the "unslept" amount (the requested time minus the
  //  time actually slept) in seconds."
  // The following function does not implement the return semantics, but
  // will do for now... A proper implementation would require Performance
  // Counters before and after the forward call to the Windows Sleep()...
  unsigned sleep(unsigned seconds)
  {
    Sleep(seconds*1000);  // The Windows Sleep operates on milliseconds
    return(0);
  }
  // Note for MinGW-gcc users: MinGW-gcc also does not provide the UNIX sleep()
  // function within <unistd.h>, but it does provide usleep(); trivial wrapping
  // of sleep() aroung usleep() is possible, however the usleep() documentation
  // (http://docs.hp.com/en/B2355-90682/usleep.2.html) clearly states that:
  // "The useconds argument must be less than 1,000,000. (...)
  //  (...) The usleep() function may fail if:
  //  [EINVAL]
  //     The time interval specified 1,000,000 or more microseconds."
  // which means that something like below can be potentially dangerous:
  // unsigned sleep(unsigned seconds)
  // {
  //   usleep(seconds*1000000);  // The usleep operates on microseconds
  //   return(0);
  // }
  // So, it is strongly advised to stick with the _WIN32/_MSC_VER
  // http://www.xinotes.org/notes/note/439/
#endif//_WIN32



/* This is a simple demo. It connects to the kinect and plays with the motor,
   the accelerometers, and the LED. It doesn't do anything with images. And,
   unlike the other examples, no OpenGL is required!
   So, this should serve as the reference example for working with the motor,
   accelerometers, and LEDs.   */

enum failon { SETTING_TILT = 1, SETTING_LED = 2, SOMETHIN_ELSE = 3 };

void no_kinect_quit(int error)
{
  switch (error) {
    case SETTING_TILT:
    ROS_ERROR_STREAM("Error in setting tilt angle." );
    break;
    case SETTING_LED:
    ROS_ERROR_STREAM("Error in setting LED options" );
    break;
    case SOMETHIN_ELSE:
    ROS_ERROR_STREAM("Error in setting tilt angle." );
    break;
    default:
    break;

  }

  ros::shutdown();
}

void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	uint8_t empty[0x1];
	double angle(angleMsg.data);

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	//angle = angle * 2;
  // Set the tilt angle (in degrees)
  if (freenect_sync_set_tilt_degs(angle, 0)) no_kinect_quit(SETTING_TILT);
ROS_INFO_STREAM("HELLO!. %f" << angle );

}

void setLedOption(const std_msgs::UInt16 optionMsg)
{
	uint8_t empty[0x1];
	const uint16_t option(optionMsg.data);

	freenect_led_options led = (freenect_led_options) (option % 6); // explicit cast

  // Set the LEDs to one of the possible states
  if (freenect_sync_set_led( led, 0)) no_kinect_quit(SETTING_LED);
}

int main(int argc, char *argv[])
{
	//srand(time(0));

  ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n;

  //set publishers and subscribers. shamelessly copypaste from the late kinect_aux

  pub_imu = n.advertise<sensor_msgs::Imu>("imu", 15);
  pub_tilt_angle = n.advertise<std_msgs::Float64>("cur_tilt_angle", 15);
  pub_tilt_status = n.advertise<std_msgs::UInt8>("cur_tilt_status", 15);

  sub_tilt_angle = n.subscribe("tilt_angle", 1, setTiltAngle);
  sub_led_option = n.subscribe("led_option", 1, setLedOption);

  ros::Rate r(1);

	while (ros::ok()) {
		
		//int tilt = (rand() % 30)-15;
		freenect_raw_tilt_state *state = 0;
		double dx, dy, dz;

		// Get the raw accelerometer values and tilt data
		if (freenect_sync_get_tilt_state(&state, 0)) no_kinect_quit(SOMETHIN_ELSE);

		// Get the processed accelerometer values (calibrated to gravity)
		freenect_get_mks_accel(state, &dx, &dy, &dz);

		//need to re-add led and tilt state publishers. tilt should come from state. led is presumed (I guess)

		printf("led[--] tilt[--] accel[%lf,%lf,%lf]\n", dx,dy,dz);
 		   //should be a publisher. I don't care about this.
		ros::spinOnce();
		r.sleep();
    
	}
}
