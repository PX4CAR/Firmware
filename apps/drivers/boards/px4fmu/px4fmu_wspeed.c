/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4fmu_wspeed.c
 *
 * PX4FMU Wheel Speed backend.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <math.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32_internal.h"
#include "px4fmu_internal.h"
#include "stm32_tim.h"


// Timers for speed measurement and filter interrupt
static struct stm32_tim_dev_s	*timer, *timer2;

// Wheel speeds in rad/s
float speeds[4] = {0, 0, 0, 0};
// Filtered wheel speeds in rad/s
float speeds_filtered[4] = {0, 0, 0, 0};
// Previous time
static uint32_t previous_time[4] = {0, 0, 0, 0};



// Sensor Interrupta
void sensor_int1(int irq, FAR void *context)
{
    compute_speed(0);
    return;
}

void sensor_int2(int irq, FAR void *context)
{
    
    compute_speed(1);
    return;
}

void sensor_int3(int irq, FAR void *context)
{
	compute_speed(2);
	return;
}

void sensor_int4(int irq, FAR void *context)
{
	compute_speed(3);
	return;
}


int compute_speed(int sensor)
{
    // speeds buffer
    float buffer[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
    // New Time
	uint32_t new_time;
    // Time interval between two passes
    float delta_t;
    
	
	// Get Time (read 32 bit register of Timer 5)
	new_time = getreg32(STM32_TIM5_BASE + STM32_GTIM_CNT_OFFSET);
	// Get delta t
	delta_t = ((float)(new_time - previous_time[sensor]))/1000000.0;
    
    
	// Rebounce protection
	if(delta_t > 0.001)
	{
		// set previous time
        previous_time[sensor] = new_time;
        
        // rotate buffer
        int i = 0;
        for(i = 3 ; i>0 ; i--)
        {
            buffer[sensor][i] = buffer[sensor][i-1];
        }
        
        // new speed
        buffer[sensor][0] = (0.5*M_PI/delta_t);
        
        // average over 4 last interrupts
        speeds[sensor] = (buffer[sensor][0]+buffer[sensor][1]+buffer[sensor][2]+buffer[sensor][3]);
	}
    
	return 1;
}


int filter_interrupt(int irq, FAR void *context)
{
    STM32_TIM_ACKINT(timer2, 0);
    
    // Previous wheel speeds for filtering
    static float previous_speeds_filtered[4] = {0, 0, 0, 0};
    
    // filter parameter alpha
    float alpha = 0.2;
    
    int sensor;
    float delta_t;
    
    // For every sensor (1-4)
    for(sensor = 0 ; sensor < 4 ; sensor++)
    {
        // Check for stand still
        delta_t = ((float)(getreg32(STM32_TIM5_BASE + STM32_GTIM_CNT_OFFSET) - previous_time[sensor]))/1000000.0;
        // If more than one second between transitions -> set speed to 0
        if(delta_t > 0.3)
        {
            speeds[sensor] = 0;
        }
        
        // Low pass filter
        speeds_filtered[sensor] = alpha*speeds[sensor] + (1-alpha)*previous_speeds_filtered[sensor];
        previous_speeds_filtered[sensor] = speeds_filtered[sensor];
    }
    
    // Publish speeds
}


__EXPORT void up_wsgetspeeds(float * data)
{
	data[0] = speeds_filtered[0];
	data[1] = speeds_filtered[1];
	data[2] = speeds_filtered[2];
	data[3] = speeds_filtered[3];
}


__EXPORT void up_wspeedinit()
{
	// Initialize interrupts (GPIO 1-4)
	stm32_gpiosetevent(GPIO_GPIO0_INPUT, true, true, false, sensor_int1);
	stm32_gpiosetevent(GPIO_GPIO1_INPUT, true, true, false, sensor_int2);
	stm32_gpiosetevent(GPIO_GPIO2_INPUT, true, true, false, sensor_int3);
	stm32_gpiosetevent(GPIO_GPIO3_INPUT, true, true, false, sensor_int4);
	
	// Initialize timer 5
	timer =  stm32_tim_init(5);
	// Configure timer 5 for a frequency of 1000000 Hz (overflow after 4295 s)
	STM32_TIM_SETCLOCK(timer, 1000000);
	STM32_TIM_SETMODE(timer, STM32_TIM_MODE_UP | STM32_TIM_MODE_CK_INT);
    
    // Initialize timer for filter interrupt at a interval of 5ms
    timer2 = stm32_tim_init(4);
    STM32_TIM_SETISR(timer2, filter_interrupt, 0);
    STM32_TIM_ENABLEINT(timer2, 0);
    STM32_TIM_SETPERIOD(timer2, 4096);
    STM32_TIM_SETCOMPARE(timer2, 1, 5000);
    
    STM32_TIM_SETCLOCK(timer2, 1000000); // 1000000 Hz -> period: 1us
	STM32_TIM_SETMODE(timer2, STM32_TIM_MODE_UP | STM32_TIM_MODE_CK_INT);
    
    printf("[WSPEED] initialized\n");
}


