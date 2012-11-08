/****************************************************************************
 * px4/car_contorl/car_control_main.c
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *   Authors: Michael Smith <DrZiplok@me.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <v1.0/car/mavlink.h>

#include <math.h>

#include <arch/board/board.h>

#include <drivers/drv_led.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/controller.h>
#include <uORB/topics/manual_control.h>
#include <uORB/topics/wheel_speeds.h>

#include "px4_car.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 
#define PWM_MIN       980   // 5.63%
#define PWM_MAX       1860   // 11.25%
#define COMMAND_MAX   1000 // Maximal possible commande value
#define PWM_FREQUENCY 50 // Maximal possible commande value

__EXPORT int px4_car_main(int argc, char *argv[]);

int px4_car_thread_main(int argc, char *argv[]);



/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/


// File path handles for device drivers
int led, wspeed;

// Threads
pthread_t control_thread;

static bool main_thread_running = false;
static bool main_thread_should_exit = false;
static int  main_thread;


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/



// Control thread
static void *control(void * arg)
{
    static int controller_handle;
    static int manual_control_handle;
    float timestep = 0.02;
    
    // Subscribe to the topics
	controller_handle     = orb_subscribe(ORB_ID(controller));
    manual_control_handle = orb_subscribe(ORB_ID(manual_control));
    
    // uORB topic structures
    struct controller_s ctrl;
    struct manual_control_s manual_ctrl;
    
    struct wheel_speeds_s wheel_speeds;
    
    static orb_advert_t wheel_speeds_pub = 0;
    
    wheel_speeds.speed_front_left  = 1;
    wheel_speeds.speed_front_right = 1;
    wheel_speeds.speed_rear_left   = 1;
    wheel_speeds.speed_rear_right  = 1;
    
    wheel_speeds_pub = orb_advertise(ORB_ID(wheel_speeds), &wheel_speeds);
    
    // Loop forever
	while(1)
	{
        
        float wheel_speeds_raw[4] = {0,0,0,0};
        
        up_wsgetspeeds(wheel_speeds_raw);
        
        //printf("wheel speeds: %d, %d, %d, %d\n", (int)wheel_speeds_raw[0],(int)wheel_speeds_raw[1],(int)wheel_speeds_raw[2],(int)wheel_speeds_raw[3]);
    
        
        wheel_speeds.speed_front_left  = (uint16_t)wheel_speeds_raw[0];
        wheel_speeds.speed_front_right = (uint16_t)wheel_speeds_raw[1];
        wheel_speeds.speed_rear_left   = (uint16_t)wheel_speeds_raw[2];
        wheel_speeds.speed_rear_right  = (uint16_t)wheel_speeds_raw[3];
        
        
        orb_publish(ORB_ID(wheel_speeds), wheel_speeds_pub, &wheel_speeds);

        
        // Check controller
        bool updated;
        
        // check to see whether the topic has updated since the last time we read it
        orb_check(controller_handle, &updated);
        
        if (updated)
        {
            // make a local copy of the updated data structure
            orb_copy(ORB_ID(controller), controller_handle, &ctrl);
        }
        
        // OFF
        if(ctrl.controller == OFF)
        {
            // motor
            up_pwm_servo_set(2, (PWM_MAX + PWM_MIN)/2);
            // steering
            up_pwm_servo_set(3, (PWM_MAX + PWM_MIN)/2);
        }
        
        // MANUAL CONTROL
        else if(ctrl.controller == MANUAL)
        {
            // Get manual control signals
            bool updated;
            // Check if updated
            orb_check(manual_control_handle, &updated);
            
            if (updated)
            {
                // make a local copy of the updated data structure
                orb_copy(ORB_ID(manual_control), manual_control_handle, &manual_ctrl);
            }
            
            // Motor
            up_pwm_servo_set(2, PWM_MIN + ((PWM_MAX-PWM_MIN)*manual_ctrl.motor)/COMMAND_MAX);
            // Steering
            up_pwm_servo_set(3, PWM_MIN + ((PWM_MAX-PWM_MIN)*manual_ctrl.steering)/COMMAND_MAX);
        }
        
        // Led pulse
        static int ledon;
		if(ledon)
			ioctl(led, LED_ON, LED_BLUE);
		else
			ioctl(led, LED_OFF, LED_BLUE);
		ledon = !ledon;
        
        // Make this loop approx. 50 Hz
		usleep(1000000*timestep);
	}
    return true;
}

/****************************************************************************
 * Main thread
 ****************************************************************************/
int px4_car_thread_main(int argc, char *argv[])
{
    printf("[px4_car] staring\n");

    
	// Initialize LED driver
	led = open("/dev/led", O_RDONLY | O_NONBLOCK);
	if (led < 0) {
		printf("[px4_car] LED: open fail\n");
	}
    
    // Initialize wheel speed sensor
    up_wspeedinit();
    
    
    up_pwm_servo_init(0xC); // enable channels 3 and 4 (UART2 RX/TX)
    up_pwm_servo_set_rate(PWM_FREQUENCY);
    // Motor
    up_pwm_servo_set(2, (PWM_MIN+PWM_MAX)/2);
    // Steering
    up_pwm_servo_set(3, (PWM_MIN+PWM_MAX)/2);
    up_pwm_servo_arm(true);
    
    printf("[px4_car] Initialization done!\n");
    
    // Create threads (receive and control)
    pthread_attr_t my_thread_attr;
	pthread_attr_init(&my_thread_attr);
	pthread_attr_setstacksize(&my_thread_attr, 10000);
    
    printf("[px4_car] Starting threads...\n");
    
    pthread_create(&control_thread, NULL, control, NULL);
    
    printf("[px4_car] Threads started!\n");
    
    while(!main_thread_should_exit)
    {
        usleep(50000);
    }
    
	// Wait for threads to end (never happens)
	pthread_cancel(control_thread);
    
    printf("[px4_car] exiting\n");
	return 0;
}

static void usage(const char *reason)
{
    if(reason)
        printf(stderr, "%s\n", reason);
    fprintf(stderr,  "usage: px4_car {start|stop|status} \n\n");
    exit(1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: px4_car
 ****************************************************************************/

int px4_car_main(int argc, char *argv[])
{
    if(argc < 1)
    {
        usage("Missing command");
    }
    
    if(!strcmp(argv[1], "start"))
    {
        if(main_thread_running)
        {
            printf("[px4_car] already running\n");
            exit(0);
        }
        
        main_thread_should_exit = false;
        main_thread = task_spawn("main_thread",
                                 SCHED_RR,
                                 SCHED_PRIORITY_DEFAULT,
                                 4096,
                                 px4_car_thread_main,
                                 (argv) ? (const char **)&argv[2] : (const char **)NULL);
        main_thread_running = true;
        exit(0);
    }
    
    if(!strcmp(argv[1], "stop"))
    {
        main_thread_should_exit = true;
        main_thread_running = false;
        exit(0);
    }
    
    if(!strcmp(argv[1], "status"))
    {
        if(main_thread_running)
            printf("[px4_car] is running\n");
        else
            printf("[px4_car] not started\n");
    
        exit(0);
    }
    
    usage("Unrecognized command");
    exit(1);
}
