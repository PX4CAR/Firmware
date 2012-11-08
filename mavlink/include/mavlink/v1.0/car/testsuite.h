/** @file
 *	@brief MAVLink comm protocol testsuite generated from car.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef CAR_TESTSUITE_H
#define CAR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_car(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_car(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_wheel_speeds(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_wheel_speeds_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	};
	mavlink_wheel_speeds_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed_front_left = packet_in.speed_front_left;
        	packet1.speed_front_right = packet_in.speed_front_right;
        	packet1.speed_rear_left = packet_in.speed_rear_left;
        	packet1.speed_rear_right = packet_in.speed_rear_right;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wheel_speeds_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_wheel_speeds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wheel_speeds_pack(system_id, component_id, &msg , packet1.speed_front_left , packet1.speed_front_right , packet1.speed_rear_left , packet1.speed_rear_right );
	mavlink_msg_wheel_speeds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wheel_speeds_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed_front_left , packet1.speed_front_right , packet1.speed_rear_left , packet1.speed_rear_right );
	mavlink_msg_wheel_speeds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_wheel_speeds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wheel_speeds_send(MAVLINK_COMM_1 , packet1.speed_front_left , packet1.speed_front_right , packet1.speed_rear_left , packet1.speed_rear_right );
	mavlink_msg_wheel_speeds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_controller(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_controller_t packet_in = {
		5,
	};
	mavlink_controller_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.controller = packet_in.controller;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_pack(system_id, component_id, &msg , packet1.controller );
	mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.controller );
	mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_send(MAVLINK_COMM_1 , packet1.controller );
	mavlink_msg_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_car(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_wheel_speeds(system_id, component_id, last_msg);
	mavlink_test_controller(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CAR_TESTSUITE_H
