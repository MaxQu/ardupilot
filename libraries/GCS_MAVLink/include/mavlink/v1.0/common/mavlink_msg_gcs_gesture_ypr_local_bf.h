// MESSAGE GCS_GESTURE_YPR_LOCAL_BF PACKING

#define MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF 80

typedef struct __mavlink_gcs_gesture_ypr_local_bf_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot
 float x; ///< X Position in NED frame in meters
 float y; ///< Y Position in NED frame in meters
 float z; ///< Z Position in NED frame in meters (note, altitude is negative in NED)
 float vx; ///< X velocity in NED frame in meter / s
 float vy; ///< Y velocity in NED frame in meter / s
 float vz; ///< Z velocity in NED frame in meter / s
 float afx; ///< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afy; ///< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afz; ///< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float lyaw; ///< lock yaw position in rad
 float lpitch; ///< lock pitch position in rad
 float lroll; ///< lock roll position in rad
 float yaw; ///< gcs yaw position in rad
 float pitch; ///< gcs pitch position in rad
 float roll; ///< gcs roll position in rad
 float vyaw; ///< gcs yaw rate in rad/sec
 float vpitch; ///< gcs pitch rate in rad/sec
 float vroll; ///< gcs roll rate in rad/sec
 uint16_t type_mask; ///< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t coordinate_frame; ///< Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
} mavlink_gcs_gesture_ypr_local_bf_t;

#define MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN 81
#define MAVLINK_MSG_ID_80_LEN 81

#define MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC 105
#define MAVLINK_MSG_ID_80_CRC 105



#define MAVLINK_MESSAGE_INFO_GCS_GESTURE_YPR_LOCAL_BF { \
	"GCS_GESTURE_YPR_LOCAL_BF", \
	23, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, afz) }, \
         { "lyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, lyaw) }, \
         { "lpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, lpitch) }, \
         { "lroll", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, lroll) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, roll) }, \
         { "vyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vyaw) }, \
         { "vpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vpitch) }, \
         { "vroll", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, vroll) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 76, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, type_mask) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_gcs_gesture_ypr_local_bf_t, coordinate_frame) }, \
         } \
}


/**
 * @brief Pack a gcs_gesture_ypr_local_bf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param lyaw lock yaw in rad
 * @param lpitch lock pitch in rad
 * @param lroll lock roll in rad
 * @param yaw gcs yaw in rad
 * @param pitch gcs pitch in rad
 * @param roll gcs roll in rad
 * @param vyaw gcs yaw rate in rad/sec
 * @param vpitch gcs pitch rate in rad/sec
 * @param vroll gcs roll rate in rad/sec
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_gesture_ypr_local_bf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z,
						       float vx, float vy, float vz, float afx, float afy, float afz,
						       float lyaw, float lpitch, float lroll, float yaw, float pitch, float roll,
						       float vyaw, float vpitch, float vroll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);//+4
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_float(buf, 40, lyaw);
    _mav_put_float(buf, 44, lpitch);
    _mav_put_float(buf, 48, lroll);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, pitch);
    _mav_put_float(buf, 60, roll);
    _mav_put_float(buf, 64, vyaw);
    _mav_put_float(buf, 68, vpitch);
    _mav_put_float(buf, 72, vroll);//+4
	_mav_put_uint16_t(buf, 76, type_mask); //+4
	_mav_put_uint8_t(buf, 78, target_system);//+2
	_mav_put_uint8_t(buf, 79, target_component);//+1
	_mav_put_uint8_t(buf, 80, coordinate_frame);//originally 52, +1

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#else
    mavlink_gcs_gesture_ypr_local_bf_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
    packet.lyaw = lyaw;
    packet.lpitch = lpitch;
    packet.lroll = lroll;
	packet.yaw = yaw;
	packet.pitch = pitch;
	packet.roll = roll;
    packet.vyaw = vyaw;
    packet.vpitch = vpitch;
    packet.vroll = vroll;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
}

/**
 * @brief Pack a gcs_gesture_ypr_local_bf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param lyaw lock yaw in rad
 * @param lpitch lock pitch in rad
 * @param lroll lock roll in rad
 * @param yaw gcs yaw in rad
 * @param pitch gcs pitch in rad
 * @param roll gcs roll in rad
 * @param vyaw gcs yaw rate in rad/sec
 * @param vpitch gcs pitch rate in rad/sec
 * @param vroll gcs roll rate in rad/sec
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_gesture_ypr_local_bf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,uint16_t type_mask,
						           float x,float y,float z,float vx,float vy,float vz,float afx,float afy,float afz,
	                               float lyaw, float lpitch, float lroll, float yaw, float pitch, float roll,
	                               float vyaw, float vpitch, float vroll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, lyaw);
    _mav_put_float(buf, 44, lpitch);
    _mav_put_float(buf, 48, lroll);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, pitch);
    _mav_put_float(buf, 60, roll);
    _mav_put_float(buf, 64, vyaw);
    _mav_put_float(buf, 68, vpitch);
    _mav_put_float(buf, 72, vroll);//+4
    _mav_put_uint16_t(buf, 76, type_mask); //+4
    _mav_put_uint8_t(buf, 78, target_system);//+2
    _mav_put_uint8_t(buf, 79, target_component);//+1
    _mav_put_uint8_t(buf, 80, coordinate_frame);//originally 52, +1

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#else
    mavlink_gcs_gesture_ypr_local_bf_t packet;
	packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.lyaw = lyaw;
    packet.lpitch = lpitch;
    packet.lroll = lroll;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.vyaw = vyaw;
    packet.vpitch = vpitch;
    packet.vroll = vroll;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
}

/**
 * @brief Encode a gcs_gesture_ypr_local_bf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_gesture_ypr_local_bf C-struct to read the message contents from
 */

static inline uint16_t mavlink_msg_gcs_gesture_ypr_local_bf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_gesture_ypr_local_bf_t* gcs_gesture_ypr_local_bf)
{
	return mavlink_msg_gcs_gesture_ypr_local_bf_pack(system_id, component_id, msg, gcs_gesture_ypr_local_bf->time_boot_ms, gcs_gesture_ypr_local_bf->target_system, gcs_gesture_ypr_local_bf->target_component, gcs_gesture_ypr_local_bf->coordinate_frame, gcs_gesture_ypr_local_bf->type_mask,
	                                                 gcs_gesture_ypr_local_bf->x, gcs_gesture_ypr_local_bf->y, gcs_gesture_ypr_local_bf->z,
	                                                 gcs_gesture_ypr_local_bf->vx, gcs_gesture_ypr_local_bf->vy, gcs_gesture_ypr_local_bf->vz,
	                                                 gcs_gesture_ypr_local_bf->afx, gcs_gesture_ypr_local_bf->afy, gcs_gesture_ypr_local_bf->afz,
	                                                 gcs_gesture_ypr_local_bf->lyaw, gcs_gesture_ypr_local_bf->lpitch, gcs_gesture_ypr_local_bf->lroll,
	                                                 gcs_gesture_ypr_local_bf->yaw, gcs_gesture_ypr_local_bf->pitch, gcs_gesture_ypr_local_bf->roll,
	                                                 gcs_gesture_ypr_local_bf->vyaw, gcs_gesture_ypr_local_bf->vpitch, gcs_gesture_ypr_local_bf->vroll);
}

/**
 * @brief Encode a gcs_gesture_ypr_local_bf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_gesture_ypr_local_bf C-struct to read the message contents from
 */

static inline uint16_t mavlink_msg_gcs_gesture_ypr_local_bf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_gesture_ypr_local_bf_t* gcs_gesture_ypr_local_bf)
{
	return mavlink_msg_gcs_gesture_ypr_local_bf_pack_chan(system_id, component_id, chan, msg, gcs_gesture_ypr_local_bf->time_boot_ms, gcs_gesture_ypr_local_bf->target_system, gcs_gesture_ypr_local_bf->target_component, gcs_gesture_ypr_local_bf->coordinate_frame, gcs_gesture_ypr_local_bf->type_mask,
	                                                  gcs_gesture_ypr_local_bf->x, gcs_gesture_ypr_local_bf->y, gcs_gesture_ypr_local_bf->z,
	                                                  gcs_gesture_ypr_local_bf->vx, gcs_gesture_ypr_local_bf->vy, gcs_gesture_ypr_local_bf->vz,
	                                                  gcs_gesture_ypr_local_bf->afx, gcs_gesture_ypr_local_bf->afy, gcs_gesture_ypr_local_bf->afz,
	                                                  gcs_gesture_ypr_local_bf->lyaw, gcs_gesture_ypr_local_bf->lpitch, gcs_gesture_ypr_local_bf->lroll,
	                                                  gcs_gesture_ypr_local_bf->yaw, gcs_gesture_ypr_local_bf->pitch, gcs_gesture_ypr_local_bf->roll,
	                                                  gcs_gesture_ypr_local_bf->vyaw, gcs_gesture_ypr_local_bf->vpitch, gcs_gesture_ypr_local_bf->vroll);
}

/**
 * @brief Send a gcs_gesture_ypr_local_bf message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param lyaw lock yaw in rad
 * @param lpitch lock pitch in rad
 * @param lroll lock roll in rad
 * @param yaw gcs yaw in rad
 * @param pitch gcs pitch in rad
 * @param roll gcs roll in rad
 * @param vyaw gcs yaw rate in rad/sec
 * @param vpitch gcs pitch rate in rad/sec
 * @param vroll gcs roll rate in rad/sec
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_gesture_ypr_local_bf_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask,
                                     float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz,
                                     float lyaw, float lpitch, float lroll, float yaw, float pitch, float roll,
                                     float vyaw, float vpitch, float vroll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, lyaw);
    _mav_put_float(buf, 44, lpitch);
    _mav_put_float(buf, 48, lroll);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, pitch);
    _mav_put_float(buf, 60, roll);
    _mav_put_float(buf, 64, vyaw);
    _mav_put_float(buf, 68, vpitch);
    _mav_put_float(buf, 72, vroll);//+4
    _mav_put_uint16_t(buf, 76, type_mask); //+4
    _mav_put_uint8_t(buf, 78, target_system);//+2
    _mav_put_uint8_t(buf, 79, target_component);//+1
    _mav_put_uint8_t(buf, 80, coordinate_frame);//originally 52, +1

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
#else
    mavlink_gcs_gesture_ypr_local_bf_t packet;
	packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.lyaw = lyaw;
    packet.lpitch = lpitch;
    packet.lroll = lroll;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.vyaw = vyaw;
    packet.vpitch = vpitch;
    packet.vroll = vroll;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.coordinate_frame = coordinate_frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, (const char *)&packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, (const char *)&packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_gesture_ypr_local_bf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask,
                            float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz,
                            float lyaw, float lpitch, float lroll, float yaw, float pitch, float roll,
                            float vyaw, float vpitch, float vroll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, lyaw);
    _mav_put_float(buf, 44, lpitch);
    _mav_put_float(buf, 48, lroll);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, pitch);
    _mav_put_float(buf, 60, roll);
    _mav_put_float(buf, 64, vyaw);
    _mav_put_float(buf, 68, vpitch);
    _mav_put_float(buf, 72, vroll);//+4
    _mav_put_uint16_t(buf, 76, type_mask); //+4
    _mav_put_uint8_t(buf, 78, target_system);//+2
    _mav_put_uint8_t(buf, 79, target_component);//+1
    _mav_put_uint8_t(buf, 80, coordinate_frame);//originally 52, +1

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, buf, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
#else
    mavlink_gcs_gesture_ypr_local_bf_t *packet = (mavlink_gcs_gesture_ypr_local_bf_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->afx = afx;
    packet->afy = afy;
    packet->afz = afz;
    packet->lyaw = lyaw;
    packet->lpitch = lpitch;
    packet->lroll = lroll;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->vyaw = vyaw;
    packet->vpitch = vpitch;
    packet->vroll = vroll;
    packet->type_mask = type_mask;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->coordinate_frame = coordinate_frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, (const char *)packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF, (const char *)packet, MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GCS_GESTURE_YPR_LOCAL_BF UNPACKING


/**
 * @brief Get field time_boot_ms from gcs_gesture_ypr_local_bf message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_gcs_gesture_ypr_local_bf_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from gcs_gesture_ypr_local_bf message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gcs_gesture_ypr_local_bf_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field target_component from gcs_gesture_ypr_local_bf message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gcs_gesture_ypr_local_bf_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field coordinate_frame from gcs_gesture_ypr_local_bf message
 *
 * @return Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 */
static inline uint8_t mavlink_msg_gcs_gesture_ypr_local_bf_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field type_mask from gcs_gesture_ypr_local_bf message
 *
 * @return Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 */
static inline uint16_t mavlink_msg_gcs_gesture_ypr_local_bf_get_type_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  76);
}

/**
 * @brief Get field x from gcs_gesture_ypr_local_bf message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from gcs_gesture_ypr_local_bf message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from gcs_gesture_ypr_local_bf message
 *
 * @return Z Position in NED frame in meters (note, altitude is negative in NED)
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from gcs_gesture_ypr_local_bf message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from gcs_gesture_ypr_local_bf message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from gcs_gesture_ypr_local_bf message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field afx from gcs_gesture_ypr_local_bf message
 *
 * @return X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_afx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field afy from gcs_gesture_ypr_local_bf message
 *
 * @return Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_afy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field afz from gcs_gesture_ypr_local_bf message
 *
 * @return Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_afz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field lyaw from gcs_gesture_ypr_local_bf message
 *
 * @return lock yaw position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_lyaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field lpitch from gcs_gesture_ypr_local_bf message
 *
 * @return lock pitch position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_lpitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field lroll from gcs_gesture_ypr_local_bf message
 *
 * @return lock roll position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_lroll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field yaw from gcs_gesture_ypr_local_bf message
 *
 * @return gcs yaw position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field pitch from gcs_gesture_ypr_local_bf message
 *
 * @return gcs pitch position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field roll from gcs_gesture_ypr_local_bf message
 *
 * @return gcs roll position in rad
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field yaw rate from gcs_gesture_ypr_local_bf message
 *
 * @return gcs yaw rate in rad/sec
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vyaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field pitch rate from gcs_gesture_ypr_local_bf message
 *
 * @return gcs pitch rate in rad/sec
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vpitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field roll rate from gcs_gesture_ypr_local_bf message
 *
 * @return gcs roll rate in rad/sec
 */
static inline float mavlink_msg_gcs_gesture_ypr_local_bf_get_vroll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Decode a gcs_gesture_ypr_local_bf message into a struct
 *
 * @param msg The message to decode
 * @param gcs_gesture_ypr_local_bf C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_gesture_ypr_local_bf_decode(const mavlink_message_t* msg, mavlink_gcs_gesture_ypr_local_bf_t* gcs_gesture_ypr_local_bf)
{
#if MAVLINK_NEED_BYTE_SWAP
	gcs_gesture_ypr_local_bf->time_boot_ms = mavlink_msg_gcs_gesture_ypr_local_bf_get_time_boot_ms(msg);
	gcs_gesture_ypr_local_bf->x = mavlink_msg_gcs_gesture_ypr_local_bf_get_x(msg);
	gcs_gesture_ypr_local_bf->y = mavlink_msg_gcs_gesture_ypr_local_bf_get_y(msg);
	gcs_gesture_ypr_local_bf->z = mavlink_msg_gcs_gesture_ypr_local_bf_get_z(msg);
	gcs_gesture_ypr_local_bf->vx = mavlink_msg_gcs_gesture_ypr_local_bf_get_vx(msg);
	gcs_gesture_ypr_local_bf->vy = mavlink_msg_gcs_gesture_ypr_local_bf_get_vy(msg);
	gcs_gesture_ypr_local_bf->vz = mavlink_msg_gcs_gesture_ypr_local_bf_get_vz(msg);
	gcs_gesture_ypr_local_bf->afx = mavlink_msg_gcs_gesture_ypr_local_bf_get_afx(msg);
	gcs_gesture_ypr_local_bf->afy = mavlink_msg_gcs_gesture_ypr_local_bf_get_afy(msg);
	gcs_gesture_ypr_local_bf->afz = mavlink_msg_gcs_gesture_ypr_local_bf_get_afz(msg);
    gcs_gesture_ypr_local_bf->lyaw = mavlink_msg_gcs_gesture_ypr_local_bf_get_lyaw(msg);
    gcs_gesture_ypr_local_bf->lpitch = mavlink_msg_gcs_gesture_ypr_local_bf_get_lpitch(msg);
    gcs_gesture_ypr_local_bf->lroll = mavlink_msg_gcs_gesture_ypr_local_bf_get_lroll(msg);
	gcs_gesture_ypr_local_bf->yaw = mavlink_msg_gcs_gesture_ypr_local_bf_get_yaw(msg);
    gcs_gesture_ypr_local_bf->pitch = mavlink_msg_gcs_gesture_ypr_local_bf_get_pitch(msg);
    gcs_gesture_ypr_local_bf->roll = mavlink_msg_gcs_gesture_ypr_local_bf_get_roll(msg);
    gcs_gesture_ypr_local_bf->vyaw = mavlink_msg_gcs_gesture_ypr_local_bf_get_vyaw(msg);
    gcs_gesture_ypr_local_bf->vpitch = mavlink_msg_gcs_gesture_ypr_local_bf_get_vpitch(msg);
    gcs_gesture_ypr_local_bf->vroll = mavlink_msg_gcs_gesture_ypr_local_bf_get_vroll(msg);
	gcs_gesture_ypr_local_bf->type_mask = mavlink_msg_gcs_gesture_ypr_local_bf_get_type_mask(msg);
	gcs_gesture_ypr_local_bf->target_system = mavlink_msg_gcs_gesture_ypr_local_bf_get_target_system(msg);
	gcs_gesture_ypr_local_bf->target_component = mavlink_msg_gcs_gesture_ypr_local_bf_get_target_component(msg);
	gcs_gesture_ypr_local_bf->coordinate_frame = mavlink_msg_gcs_gesture_ypr_local_bf_get_coordinate_frame(msg);
#else
	memcpy(gcs_gesture_ypr_local_bf, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GCS_GESTURE_YPR_LOCAL_BF_LEN);
#endif
}
