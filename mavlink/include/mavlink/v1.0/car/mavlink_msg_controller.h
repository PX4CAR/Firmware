// MESSAGE CONTROLLER PACKING

#define MAVLINK_MSG_ID_CONTROLLER 151

typedef struct __mavlink_controller_t
{
 uint8_t controller; ///< Type of the controller
} mavlink_controller_t;

#define MAVLINK_MSG_ID_CONTROLLER_LEN 1
#define MAVLINK_MSG_ID_151_LEN 1



#define MAVLINK_MESSAGE_INFO_CONTROLLER { \
	"CONTROLLER", \
	1, \
	{  { "controller", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_controller_t, controller) }, \
         } \
}


/**
 * @brief Pack a controller message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param controller Type of the controller
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_controller_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t controller)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, controller);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_controller_t packet;
	packet.controller = controller;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROLLER;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 15);
}

/**
 * @brief Pack a controller message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param controller Type of the controller
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_controller_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t controller)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, controller);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_controller_t packet;
	packet.controller = controller;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROLLER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 15);
}

/**
 * @brief Encode a controller struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param controller C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_controller_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_controller_t* controller)
{
	return mavlink_msg_controller_pack(system_id, component_id, msg, controller->controller);
}

/**
 * @brief Send a controller message
 * @param chan MAVLink channel to send the message
 *
 * @param controller Type of the controller
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_controller_send(mavlink_channel_t chan, uint8_t controller)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, controller);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROLLER, buf, 1, 15);
#else
	mavlink_controller_t packet;
	packet.controller = controller;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROLLER, (const char *)&packet, 1, 15);
#endif
}

#endif

// MESSAGE CONTROLLER UNPACKING


/**
 * @brief Get field controller from controller message
 *
 * @return Type of the controller
 */
static inline uint8_t mavlink_msg_controller_get_controller(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a controller message into a struct
 *
 * @param msg The message to decode
 * @param controller C-struct to decode the message contents into
 */
static inline void mavlink_msg_controller_decode(const mavlink_message_t* msg, mavlink_controller_t* controller)
{
#if MAVLINK_NEED_BYTE_SWAP
	controller->controller = mavlink_msg_controller_get_controller(msg);
#else
	memcpy(controller, _MAV_PAYLOAD(msg), 1);
#endif
}
