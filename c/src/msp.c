#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memset()

#include "msp.h"
#include "board.h" // msp_busy
#include "fc.h" // host_buffer_tx
#include "osd.h" // OSD_FLIGHT_MODE

/*
The DJI Air Unit polls the FC for the following MSP messages at around 4Hz.
Note: messages are polled in ascending hex id order.

  3 MSP_FC_VERSION
 10 MSP_NAME
 84 MSP_OSD_CONFIG (Custom OSD)
 92 MSP_FILTER_CONFIG
 94 MSP_PID_ADVANCED
101 MSP_STATUS
105 MSP_RC
106 MSP_RAW_GPS (Custom OSD)
107 MSP_COMP_GPS (Custom OSD)
108 MSP_ATTITUDE (Custom OSD)
109 MSP_ALTITUDE (Custom OSD)
110 MSP_ANALOG
111 MSP_RC_TUNING
112 MSP_PID
130 MSP_BATTERY_STATE
134 MSP_ESC_SENSOR_DATA (Custom OSD)
150 MSP_STATUS_EX
247 MSP_RTC (Custom OSD)
*/

/* Private defines --------------------------------------*/

#define MSP_FC_VERSION                  3    // out message: Get flight controller version
#define MSP_NAME                        10   // out message: Returns user set board name - betaflight
// Baseflight MSP commands (64-89)
#define MSP_OSD_CONFIG                  84   // out message: Get OSD settings
// Betaflight Additional Commands (90-99)
#define MSP_FILTER_CONFIG               92   // out message: Get filter configuration
#define MSP_PID_ADVANCED                94   // out message: Get advanced PID settings
// Multiwii original MSP commands (101-139)
#define MSP_STATUS                      101  // out message: Cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RC                          105  // out message: RC channels and more
#define MSP_RAW_GPS                     106  // out message: Fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS                    107  // out message: Distance home, direction home
#define MSP_ATTITUDE                    108  // out message: 2 angles 1 heading
#define MSP_ALTITUDE                    109  // out message: Altitude, variometer
#define MSP_ANALOG                      110  // out message: Vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING                   111  // out message: RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                         112  // out message: P I D coeff (9 are used currently)
#define MSP_WP                          118  // out message: Get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BATTERY_STATE               130  // out message: Connected/Disconnected, Voltage, Current Used
// Additional non-MultiWii commands (150-166)
#define MSP_STATUS_EX                   150  // out message: Cycletime, errors_count, CPU load, sensor present etc
// Multiple MSP and special commands (230-255)
#define MSP_RTC                         247  // out message: Get the RTC clock

#define MSP_OSD_START 2048
#define MSP_OSD_STEP_X 1
#define MSP_OSD_STEP_Y 32

/* Private macros --------------------------------------*/

#define MSP_OSD_POS(x,y) (MSP_OSD_START + (x)*MSP_OSD_STEP_X + (y)*MSP_OSD_STEP_Y)

/* Private types --------------------------------------*/

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

typedef enum {
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

typedef enum {
    OSD_TIMER_COUNT // MUST BE LAST
} osd_timer_e;

typedef enum {
    OSD_WARNING_COUNT // MUST BE LAST
} osd_warnings_flags_e;

typedef struct msp_status_s {
	uint16_t task_delta_time;
	uint16_t i2c_error_count;
	uint16_t sensor_status;
	uint32_t flight_mode_flags;
	uint8_t pid_profile;
	uint16_t system_load;
	uint16_t gyro_cycle_time;
	uint8_t box_mode_flags;
	uint8_t arming_disable_flags_count;
	uint32_t arming_disable_flags;
	uint8_t extra_flags;
} __attribute__((packed)) msp_status_t;

typedef struct msp_osd_config_s {
	uint8_t flags;
	uint8_t video_system;
	uint8_t units;
	uint8_t rssi_alarm;
	uint16_t capacity_alarm;
	uint8_t unused_0;
	uint8_t item_count;
	uint16_t alt_alarm;
	uint16_t items_position[OSD_ITEM_COUNT];
	uint8_t stats_items_count;
	uint16_t stats_items[OSD_STAT_COUNT] ;
	uint8_t timers_count;
	uint16_t timers[OSD_TIMER_COUNT];
	uint16_t enabled_warnings_old;
	uint8_t warnings_count_new;
	uint32_t enabled_warnings_new;
	uint8_t available_profiles;
	uint8_t selected_profile;
	uint8_t osd_stick_overlay;
} __attribute__((packed)) msp_osd_config_t;

typedef struct msp_analog_s {
	uint8_t voltage_dv;
	uint16_t mah;
	uint16_t rssi;
	int16_t current_ca;
	uint16_t voltage_cv;
} __attribute__((packed)) msp_analog_t;

typedef struct msp_battery_state_s {
	uint8_t cellcount;
	uint16_t capacity_mah;
	uint8_t voltage_dv;
	uint16_t mah;
	int16_t current_ca;
	uint8_t state;
	uint16_t voltage_cv;
} __attribute__((packed)) msp_battery_state_t;

typedef struct msp_frame_s {
	uint8_t header[2];
	uint8_t direction;
	uint8_t size;
	uint8_t type;
	uint8_t payload[sizeof(msp_osd_config_t)];
	uint8_t checksum;
} __attribute__((packed)) msp_frame_t;

/* Private variables -----------------------*/

msp_frame_t msp_frame;

/* Private Functions -----------------------*/

void msp_send(uint8_t cmd, uint8_t size)
{
	uint8_t i;
	uint8_t checksum;
	uint8_t* buf = (uint8_t*)&msp_frame;

	msp_frame.header[0] = '$';
	msp_frame.header[1] = 'M';
	msp_frame.direction = '>';
	msp_frame.size = size;
	msp_frame.type = cmd;
	checksum = 0;
	for (i = 3; i < msp_frame.size+5; i++)
		checksum ^= buf[i];
	buf[msp_frame.size+5] = checksum;

	msp_transfer((uint8_t*)&msp_frame, host_buffer_tx.u8, msp_frame.size+6, 0); // Use host_buffer_tx as Rx dummy pointer
	//while (msp_busy) {}
}

/* Public Functions -----------------------*/

void msp_name(uint8_t* str)
{
	memcpy(msp_frame.payload, str, 12);
	msp_send(MSP_NAME, 12);
}

void msp_status(bool armed, bool acro)
{
	msp_status_t* status = (msp_status_t*)msp_frame.payload;

	memset(msp_frame.payload, 0, sizeof(msp_status_t));
	status->flight_mode_flags = armed | (!acro << 1); // acro flag only works in MSP_STATUS_EX

	msp_send(MSP_STATUS, sizeof(msp_status_t));
}

void msp_status_ex(bool armed, bool acro)
{
	msp_status_t* status = (msp_status_t*)msp_frame.payload;

	memset(msp_frame.payload, 0, sizeof(msp_status_t));
	status->flight_mode_flags = armed | (!acro << 1); // armed flag not working in MSP_STATUS_EX

	msp_send(MSP_STATUS_EX, sizeof(msp_status_t));
}

void msp_osd_config(void)
{
	msp_osd_config_t* osd_config = (msp_osd_config_t*)msp_frame.payload;

	memset(msp_frame.payload, 0, sizeof(msp_osd_config_t));
	osd_config->flags = 1; // OSD_FLAGS_OSD_FEATURE
	osd_config->video_system = 3; // VIDEO_SYSTEM_HD
	osd_config->units = 1; // OSD_UNIT_METRIC
	osd_config->item_count = OSD_ITEM_COUNT;
	// Element position and visibility
#ifdef OSD_FLIGHT_MODE
	osd_config->items_position[OSD_FLYMODE] = MSP_OSD_POS(0,0); // Uncomment when enbaling MSP_STATUS_EX message
#endif
	osd_config->items_position[OSD_DISARMED] = MSP_OSD_POS(12,10);
	osd_config->items_position[OSD_CRAFT_NAME] = MSP_OSD_POS(15,11);
	osd_config->items_position[OSD_AVG_CELL_VOLTAGE] = MSP_OSD_POS(13,12);
#ifdef IBAT
	osd_config->items_position[OSD_CURRENT_DRAW] = MSP_OSD_POS(10,12);
	osd_config->items_position[OSD_MAH_DRAWN] = MSP_OSD_POS(16,12);
#endif
	// DJI OSD expects 1 OSD profile
	osd_config->available_profiles = 1;
	osd_config->selected_profile = 1;

	msp_send(MSP_OSD_CONFIG, sizeof(msp_osd_config_t));
}

void msp_analog(float vbat, float ibat, float imah, uint8_t rssi)
{
	msp_analog_t* analog = (msp_analog_t*)msp_frame.payload;
	
	analog->voltage_dv = (uint8_t)(vbat*10.0f);
	analog->mah = (uint16_t)imah;
	analog->rssi = (uint16_t)rssi * (1023/120);
	analog->current_ca = (int16_t)(ibat*100.0f);
	analog->voltage_cv = (uint16_t)(vbat*100.0f);

	msp_send(MSP_ANALOG, sizeof(msp_analog_t));
}

void msp_battery_state(float vbat, float ibat, float imah)
{
	msp_battery_state_t* battery_state = (msp_battery_state_t*)msp_frame.payload;

	battery_state->cellcount = vbat / REG_VBAT_MIN;
	battery_state->capacity_mah = 1450;
	battery_state->voltage_dv = (uint8_t)(vbat*10.0f);
	battery_state->mah = (uint16_t)imah;
	battery_state->current_ca  = (int16_t)(ibat*100.0f);
	battery_state->state = 0; // MSP_BATTERY_OK
	battery_state->voltage_cv = (uint16_t)(vbat*100.0f);

	msp_send(MSP_BATTERY_STATE, sizeof(msp_battery_state_t));
}




