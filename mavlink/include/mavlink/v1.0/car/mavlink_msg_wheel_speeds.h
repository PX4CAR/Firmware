// MESSAGE WHEEL_SPEEDS PACKING

#define MAVLINK_MSG_ID_WHEEL_SPEEDS 150

typedef struct __mavlink_wheel_speeds_t
{
 uint16_t speed_front_left; ///< front left wheel speed (rad/s)
 uint16_t speed_front_right; ///< front right wheel speed (rad/s)
 uint16_t speed_rear_left; ///< rear left wheel speed (rad/s)
 uint16_t speed_rear_right; ///< rear right wheel speed (rad/s)
} mavlink_wheel_speeds_t;

#define MAVLINK_MSG_ID_WHEEL_SPEEDS_LEN 8
#define MAVLINK_MSG_ID_150_LEN 8



#define MAVLINK_MESSAGE_INFO_WHEEL_SPEEDS { \
	"WHEEL_SPEEDS", \
	4, \
	{  { "speed_front_left", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_wheel_speeds_t, speed_front_left) }, \
         { "speed_front_right", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_wheel_speeds_t, speed_front_right) }, \
         { "speed_rear_left", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_wheel_speeds_t, speed_rear_left) }, \
         { "speed_rear_right", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_wheel_speeds_t, speed_rear_right) }, \
         } \
}


/**
 * @brief Pack a wheel_speeds message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed_front_left front left wheel speed (rad/s)
 * @param speed_front_right front right wheel speed (rad/s)
 * @param speed_rear_left rear left wheel speed (rad/s)
 * @param speed_rear_right rear right wheel speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_speeds_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t speed_front_left, uint16_t speed_front_right, uint16_t speed_rear_left, uint16_t speed_rear_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, speed_front_left);
	_mav_put_uint16_t(buf, 2, speed_front_right);
	_mav_put_uint16_t(buf, 4, speed_rear_left);
	_mav_put_uint16_t(buf, 6, speed_rear_right);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_wheel_speeds_t packet;
	packet.speed_front_left = speed_front_left;
	packet.speed_front_right = speed_front_right;
	packet.speed_rear_left = speed_rear_left;
	packet.speed_rear_right = speed_rear_right;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_WHEEL_SPEEDS;
	return mavlink_finalize_message(msg, system_id, component_id, 8, 239);
}

/**
 * @brief Pack a wheel_speeds message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed_front_left front left wheel speed (rad/s)
 * @param speed_front_right front right wheel speed (rad/s)
 * @param speed_rear_left rear left wheel speed (rad/s)
 * @param speed_rear_right rear right wheel speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_speeds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t speed_front_left,uint16_t speed_front_right,uint16_t speed_rear_left,uint16_t speed_rear_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, speed_front_left);
	_mav_put_uint16_t(buf, 2, speed_front_right);
	_mav_put_uint16_t(buf, 4, speed_rear_left);
	_mav_put_uint16_t(buf, 6, speed_rear_right);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_wheel_speeds_t packet;
	packet.speed_front_left = speed_front_left;
	packet.speed_front_right = speed_front_right;
	packet.speed_rear_left = speed_rear_left;
	packet.speed_rear_right = speed_rear_right;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_WHEEL_SPEEDS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 239);
}

/**
 * @brief Encode a wheel_speeds struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wheel_speeds C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_speeds_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wheel_speeds_t* wheel_speeds)
{
	return mavlink_msg_wheel_speeds_pack(system_id, component_id, msg, wheel_speeds->speed_front_left, wheel_speeds->speed_front_right, wheel_speeds->speed_rear_left, wheel_speeds->speed_rear_right);
}

/**
 * @brief Send a wheel_speeds message
 * @param chan MAVLink channel to send the message
 *
 * @param speed_front_left front left wheel speed (rad/s)
 * @param speed_front_right front right wheel speed (rad/s)
 * @param speed_rear_left rear left wheel speed (rad/s)
 * @param speed_rear_right rear right wheel speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wheel_speeds_send(mavlink_channel_t chan, uint16_t speed_front_left, uint16_t speed_front_right, uint16_t speed_rear_left, uint16_t speed_rear_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, speed_front_left);
	_mav_put_uint16_t(buf, 2, speed_front_right);
	_mav_put_uint16_t(buf, 4, speed_rear_left);
	_mav_put_uint16_t(buf, 6, speed_rear_right);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_SPEEDS, buf, 8, 239);
#else
	mavlink_wheel_speeds_t packet;
	packet.speed_front_left = speed_front_left;
	packet.speed_front_right = speed_front_right;
	packet.speed_rear_left = speed_rear_left;
	packet.speed_rear_right = speed_rear_right;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_SPEEDS, (const char *)&packet, 8, 239);
#endif
}

#endif

// MESSAGE WHEEL_SPEEDS UNPACKING


/**
 * @brief Get field speed_front_left from wheel_speeds message
 *
 * @return front left wheel speed (rad/s)
 */
static inline uint16_t mavlink_msg_wheel_speeds_get_speed_front_left(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field speed_front_right from wheel_speeds message
 *
 * @return front right wheel speed (rad/s)
 */
static inline uint16_t mavlink_msg_wheel_speeds_get_speed_front_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field speed_rear_left from wheel_speeds message
 *
 * @return rear left wheel speed (rad/s)
 */
static inline uint16_t mavlink_msg_wheel_speeds_get_speed_rear_left(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field speed_rear_right from wheel_speeds message
 *
 * @return rear right wheel speed (rad/s)
 */
static inline uint16_t mavlink_msg_wheel_speeds_get_speed_rear_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a wheel_speeds message into a struct
 *
 * @param msg The message to decode
 * @param wheel_speeds C-struct to decode the message contents into
 */
static inline void mavlink_msg_wheel_speeds_decode(const mavlink_message_t* msg, mavlink_wheel_speeds_t* wheel_speeds)
{
#if MAVLINK_NEED_BYTE_SWAP
	wheel_speeds->speed_front_left = mavlink_msg_wheel_speeds_get_speed_front_left(msg);
	wheel_speeds->speed_front_right = mavlink_msg_wheel_speeds_get_speed_front_right(msg);
	wheel_speeds->speed_rear_left = mavlink_msg_wheel_speeds_get_speed_rear_left(msg);
	wheel_speeds->speed_rear_right = mavlink_msg_wheel_speeds_get_speed_rear_right(msg);
#else
	memcpy(wheel_speeds, _MAV_PAYLOAD(msg), 8);
#endif
}
