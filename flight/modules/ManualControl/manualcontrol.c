/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup ManualControlModule Manual Control Module
 * @brief Provide manual control or allow it alter flight mode.
 * @{
 *
 * Reads in the ManualControlCommand FlightMode setting from receiver then either
 * pass the settings straght to ActuatorDesired object (manual mode) or to
 * AttitudeDesired object (stabilized mode)
 *
 * @file       manualcontrol.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2014.
 * @brief      ManualControl module. Handles safety R/C link and flight mode.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "inc/manualcontrol.h"
#include <sanitycheck.h>
#include <manualcontrolsettings.h>
#include <manualcontrolcommand.h>
#include <vtolselftuningstats.h>
#include <flightmodesettings.h>
#include <flightstatus.h>
#include <systemsettings.h>
#include <stabilizationdesired.h>
#include <callbackinfo.h>
#include <stabilizationsettings.h>
#if defined(PIOS_INCLUDE_BARO)
#include "altitudeholddesired.h"
#include "velocitystate.h"
#include <positionstate.h>
#endif

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
#include <vtolpathfollowersettings.h>
#endif

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES                               PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES                               1152
#endif

#define CALLBACK_PRIORITY                              CALLBACK_PRIORITY_REGULAR
#define CBTASK_PRIORITY                                CALLBACK_TASK_FLIGHTCONTROL

#define ASSISTEDCONTROL_NEUTRALTHROTTLERANGE_FACTOR    0.2f
#define ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_LO 0.92f
#define ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_HI 1.08f


// defined handlers

static const controlHandler handler_MANUAL = {
    .controlChain      = {
        .Stabilization = false,
        .PathFollower  = false,
        .PathPlanner   = false,
    },
    .handler           = &manualHandler,
};
static const controlHandler handler_STABILIZED = {
    .controlChain      = {
        .Stabilization = true,
        .PathFollower  = false,
        .PathPlanner   = false,
    },
    .handler           = &stabilizedHandler,
};


#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
static const controlHandler handler_PATHFOLLOWER = {
    .controlChain      = {
        .Stabilization = true,
        .PathFollower  = true,
        .PathPlanner   = false,
    },
    .handler           = &pathFollowerHandler,
};

static const controlHandler handler_PATHPLANNER = {
    .controlChain      = {
        .Stabilization = true,
        .PathFollower  = true,
        .PathPlanner   = true,
    },
    .handler           = &pathPlannerHandler,
};
static float thrustAtBrakeStart = 0.0f;
static float thrustLo = 0.0f;
static float thrustHi = 0.0f;

#endif /* ifndef PIOS_EXCLUDE_ADVANCED_FEATURES */
// Private variables
static DelayedCallbackInfo *callbackHandle;

// Private functions
static void configurationUpdatedCb(UAVObjEvent *ev);
static void commandUpdatedCb(UAVObjEvent *ev);
static void manualControlTask(void);
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
static uint8_t isAssistedFlightMode(uint8_t position, uint8_t flightMode, FlightModeSettingsData *modeSettings);
#endif

#define assumptions (assumptions1 && assumptions2 && assumptions3 && assumptions4 && assumptions5 && assumptions6 && assumptions7 && assumptions_flightmode)

#if defined(PIOS_INCLUDE_BARO)
// 用于检测是否接近地面
#define VELOCITY_ARRAY_LENGTH		14
#define VELOCITY_PROBABILITY		0.6f
#define VELOCITY_LADING				0.25f
#define VELOCITY_CLOSE_TO_GROUND	0.15f
static float velocityZDatas[VELOCITY_ARRAY_LENGTH];
static uint8_t velocityZDatasIndex = 0;
static uint32_t velocityLastTime = 0;
static bool isVelocityLessThan(float *velocityDatas, uint8_t length, float velocity, float probability);
static bool isVelocityMoreThan(float *velocityDatas, uint8_t length, float velocity, float probability);
#endif

/**
 * Module starting
 */
int32_t ManualControlStart()
{
    // Run this initially to make sure the configuration is checked
    configuration_check();

    // Whenever the configuration changes, make sure it is safe to fly
    SystemSettingsConnectCallback(configurationUpdatedCb);
    ManualControlSettingsConnectCallback(configurationUpdatedCb);
    ManualControlCommandConnectCallback(commandUpdatedCb);

    // clear alarms
    AlarmsClear(SYSTEMALARMS_ALARM_MANUALCONTROL);

    // Make sure unarmed on power up
    armHandler(true);

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    takeOffLocationHandlerInit();
#endif
    // Start main task
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);

    return 0;
}

/**
 * Module initialization
 */
int32_t ManualControlInitialize()
{
    /* Check the assumptions about uavobject enum's are correct */
    PIOS_STATIC_ASSERT(assumptions);

    ManualControlCommandInitialize();
    FlightStatusInitialize();
    ManualControlSettingsInitialize();
    FlightModeSettingsInitialize();
    SystemSettingsInitialize();
#if defined(PIOS_INCLUDE_BARO)
	PositionStateInitialize();
	VelocityStateInitialize();
	AltitudeHoldDesiredInitialize();
#endif
    StabilizationSettingsInitialize();
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    VtolSelfTuningStatsInitialize();
    VtolPathFollowerSettingsInitialize();
#endif
    callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&manualControlTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_MANUALCONTROL, STACK_SIZE_BYTES);

    return 0;
}
MODULE_INITCALL(ManualControlInitialize, ManualControlStart);

/**
 * Module task
 */
static void manualControlTask(void)
{
    // Process Arming
    armHandler(false);
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    takeOffLocationHandler();
#endif
    // Process flight mode
    FlightStatusData flightStatus;

    FlightStatusGet(&flightStatus);
    ManualControlCommandData cmd;
    ManualControlCommandGet(&cmd);
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    VtolPathFollowerSettingsThrustLimitsData thrustLimits;
    VtolPathFollowerSettingsThrustLimitsGet(&thrustLimits);
#endif

    FlightModeSettingsData modeSettings;
    FlightModeSettingsGet(&modeSettings);

    uint8_t position = cmd.FlightModeSwitchPosition;
    uint8_t newMode  = flightStatus.FlightMode;
    uint8_t newFlightModeAssist      = flightStatus.FlightModeAssist;
    uint8_t newAssistedControlState  = flightStatus.AssistedControlState;
    uint8_t newAssistedThrottleState = flightStatus.AssistedThrottleState;
    if (position < FLIGHTMODESETTINGS_FLIGHTMODEPOSITION_NUMELEM) {
        newMode = modeSettings.FlightModePosition[position];
    }

    // if a mode change occurs we default the assist mode and states here
    // to avoid having to add it to all of the below modes that are
    // otherwise unrelated
    if (newMode != flightStatus.FlightMode) {
        // set assist mode to none to avoid an assisted flight mode position
        // carrying over and impacting a newly selected non-assisted flight mode pos
        newFlightModeAssist      = FLIGHTSTATUS_FLIGHTMODEASSIST_NONE;
        // The following are eqivalent to none effectively. Code should always
        // check the flightmodeassist state.
        newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;

		// Add by Richile
#if defined(PIOS_INCLUDE_BARO)
		// Altitude hold
		if(newMode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED1){
			float down;
			PositionStateDownGet(&down);
			down = -down;
			AltitudeHoldDesiredAltitudeSet(&down);
		}
#endif
		// End Richile
    }
	
	// Add by Richile
#if defined(PIOS_INCLUDE_BARO)
	static float climbRateMax = 0.0f;
	uint8_t armed = 0;
	FlightStatusArmedGet(&armed);

	if(newMode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED1 && armed == FLIGHTSTATUS_ARMED_ARMED){
		float throttle;
		StabilizationDesiredThrustGet(&throttle);
		float down; 
		PositionStateDownGet(&down);
		down = -down;
		float climbRate = 0.0f;
		// Set climbRate, deadband=0.15f center=0.475f
		float deadband = 0.2f;
		float center = 0.475f;
		uint8_t enableOneKeyTakeOff = false;
		FlightStatusEnableOneKeyTakeOffGet(&enableOneKeyTakeOff);
		if(cmd.Throttle < (center - deadband) && cmd.Throttle >= 0.0f)
			climbRate = (cmd.Throttle - (center - deadband)) * 2.0f;
		else if(cmd.Throttle > (center + deadband))
			climbRate = (cmd.Throttle - (center + deadband)) * 2.5f * ((down > 1.0f && !enableOneKeyTakeOff)?0.6f:1.0f);
		uint8_t land = false;
		uint8_t connectionBreaked;
		float velocity_z;
		AltitudeHoldDesiredConnectionBreakedGet(&connectionBreaked);
		VelocityStateDownGet(&velocity_z);
		if(cmd.Throttle < 0.0f){
			land = true;
			climbRate = -0.7f;
		}

		/* 如果开启了一键降落而且接近地面了
		 *  接近地面时下降速度小于2.0m/s，测试多组数据
		 *  从数据中看小于2.0m/s的概率
		 */
		static uint8_t increaseClimbRate = false;
		static uint8_t isLanding = false;
		uint8_t enableOneKeyLanding = false;
		FlightStatusEnableOneKeyLandingGet(&enableOneKeyLanding);
		if(velocityLastTime == 0){
			velocityLastTime = PIOS_DELAY_GetRaw();
		}
		if((PIOS_DELAY_GetRaw() - velocityLastTime) > 50){
			velocityZDatas[velocityZDatasIndex++] = velocity_z;
			if(velocityZDatasIndex >= VELOCITY_ARRAY_LENGTH){
				velocityZDatasIndex = 0;
			}
			if(enableOneKeyLanding && isVelocityMoreThan(velocityZDatas, VELOCITY_ARRAY_LENGTH, VELOCITY_LADING, 0.7f)){
				isLanding = true;
			}
			if(enableOneKeyLanding && isLanding && isVelocityLessThan(velocityZDatas, VELOCITY_ARRAY_LENGTH, VELOCITY_CLOSE_TO_GROUND, VELOCITY_PROBABILITY)){
				increaseClimbRate = true;
			}
			// 如果摇杆拉在最下面保持了一段时间，并且飞机的下降速度
			// 一直很小的话，说明已经落地了，要快速减掉油门
			static uint8_t throttle_1_cnt = 0;
			if(cmd.Throttle < -0.99f){
				throttle_1_cnt++;
				if(throttle_1_cnt > 15){
					if(isVelocityLessThan(velocityZDatas, VELOCITY_ARRAY_LENGTH, VELOCITY_CLOSE_TO_GROUND, VELOCITY_PROBABILITY)){
						increaseClimbRate = true;
					}
				}
			}else{
				throttle_1_cnt = 0;
			}
			velocityLastTime = PIOS_DELAY_GetRaw();
		}
		if(increaseClimbRate){
			land = true;
			climbRate = -1.2f;
		}
		if(!enableOneKeyLanding || armed != FLIGHTSTATUS_ARMED_ARMED || throttle < 0.0001f){
			isLanding = false;
			increaseClimbRate = false;
		}

		
		
		// 靠近地面时快速下降
		//if(connectionBreaked && velocity_z < 0.5f && velocity_z > -0.01f){
		//if(cmd.Throttle < 0.01f && velocity_z < 0.5f){
		//	land = true;
		//	climbRate = -1.0f;
		//}
		
		//if(connectionBreaked && down < 0.1f){
		//	land = true;
		//	climbRate = -0.7f;
		//}
		//if(cmd.Throttle < 0.01f && down < 0.1f){
		//	land = true;
		//	climbRate = -0.7f;
		//}
		
		// climbRateMax > 0.0f ==> Takeoffed
		static uint32_t climbRatePositiveLastTime = 0;
		if(climbRate > 0.001f || (climbRate < -0.001f && climbRateMax > 0.0f)){
			// set new altitude
			AltitudeHoldDesiredAltitudeSet(&down);
			//if(climbRate > 0.001f){
				climbRatePositiveLastTime = PIOS_DELAY_GetRaw();
			//}
		}else{ 
			// altitude hold
			if(PIOS_DELAY_DiffuS(climbRatePositiveLastTime) < 100000){
				float altHoldDown = down - 0.1f;
				AltitudeHoldDesiredAltitudeSet(&altHoldDown);
			}
			if(climbRate < 0.001f && climbRate > 0.0f)
				climbRate = 0.0f;
		}
		if(climbRate > climbRateMax){
			climbRateMax = climbRate;
		}
		
		uint8_t enable = true;
		
		if(throttle < 0.005f && climbRate < 0.0001f){
			climbRateMax = 0.0f;
			throttle = -0.1f;
			StabilizationDesiredThrustSet(&throttle);
			enable = false;
		}
		AltitudeHoldDesiredEnableSet(&enable);
		AltitudeHoldDesiredLandSet(&land);
		AltitudeHoldDesiredClimbRateSet(&climbRate);
	}else if(newMode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED1 && armed != FLIGHTSTATUS_ARMED_ARMED){
		climbRateMax = 0.0f;
		float throttle;
		throttle = -0.1f;
		StabilizationDesiredThrustSet(&throttle);
		uint8_t enable = false;
		AltitudeHoldDesiredEnableSet(&enable);
	}
#endif
	// End Richile

    // Depending on the mode update the Stabilization or Actuator objects
    const controlHandler *handler = &handler_MANUAL;
    switch (newMode) {
    case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
        handler = &handler_MANUAL;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
        handler = &handler_STABILIZED;

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
        newFlightModeAssist = isAssistedFlightMode(position, newMode, &modeSettings);
        if (newFlightModeAssist) {
            // assess roll/pitch state
            bool flagRollPitchHasInput = (fabsf(cmd.Roll) > 0.0f || fabsf(cmd.Pitch) > 0.0f);

            // assess throttle state
            bool throttleNeutral = false;
            bool throttleNeutralOrLow = false;
            float neutralThrustOffset = 0.0f;
            VtolSelfTuningStatsNeutralThrustOffsetGet(&neutralThrustOffset);


            float throttleRangeDelta = (thrustLimits.Neutral + neutralThrustOffset) * ASSISTEDCONTROL_NEUTRALTHROTTLERANGE_FACTOR;
            float throttleNeutralLow = (thrustLimits.Neutral + neutralThrustOffset) - throttleRangeDelta;
            float throttleNeutralHi  = (thrustLimits.Neutral + neutralThrustOffset) + throttleRangeDelta;
            if (cmd.Thrust > throttleNeutralLow && cmd.Thrust < throttleNeutralHi) {
                throttleNeutral = true;
            }
            if (cmd.Thrust < throttleNeutralHi) {
                throttleNeutralOrLow = true;
            }

            // determine default thrust mode for hold/brake states
            uint8_t pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
            if (newFlightModeAssist == FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST) {
                pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO;
            }

            switch (newAssistedControlState) {
            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY:
                if (!flagRollPitchHasInput) {
                    // no stick input on roll/pitch so enter brake state
                    newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE;
                    newAssistedThrottleState = pathfollowerthrustmode;
                    handler = &handler_PATHFOLLOWER;
                    // retain thrust cmd for later comparison with actual in braking
                    thrustAtBrakeStart = cmd.Thrust;

                    // calculate hi and low value of +-8% as a mini-deadband
                    // for use in auto-override in brake sequence
                    thrustLo = ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_LO * thrustAtBrakeStart;
                    thrustHi = ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_HI * thrustAtBrakeStart;

                    // The purpose for auto throttle assist is to go from a mid to high thrust range to a
                    // neutral vertical-holding/maintaining ~50% thrust range.  It is not designed/intended
                    // to go from near zero to 50%...we don't want an auto-takeoff feature here!
                    // Also for rapid decents a user might have a bit of forward stick and low throttle
                    // then stick-off for auto-braking...but if below the vtol min of 20% it will not
                    // kick in...the flyer needs to manually manage throttle to slow down decent,
                    // and the next time they put in a bit of stick, revert to primary, and then
                    // sticks-off it will brake and hold with auto-thrust
                    if (thrustAtBrakeStart < thrustLimits.Min) {
                        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                    }
                } else {
                    // stick input so stay in primary mode control state
                    // newAssistedControlState = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
                    newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                }
                break;

            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE:
                if (flagRollPitchHasInput) {
                    // stick input during brake sequence allows immediate resumption of control
                    newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
                    newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                } else {
                    // no stick input, stay in brake mode
                    newAssistedThrottleState = pathfollowerthrustmode;
                    handler = &handler_PATHFOLLOWER;

                    // if auto thrust and user adjusts thrust outside of a deadband in which case revert to manual
                    if ((newAssistedThrottleState == FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO) &&
                        (cmd.Thrust < thrustLo || cmd.Thrust > thrustHi)) {
                        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
                    }
                }
                break;

            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_HOLD:

                if (newFlightModeAssist == FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST_PRIMARYTHRUST ||
                    (newFlightModeAssist == FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST &&
                     newAssistedThrottleState == FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL)) {
                    // for manual or primary throttle states/modes, stick control immediately reverts to primary mode control
                    if (flagRollPitchHasInput) {
                        newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
                        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                    } else {
                        // otherwise stay in the hold state
                        // newAssistedControlState = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_HOLD;
                        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
                        handler = &handler_PATHFOLLOWER;
                    }
                } else if (newFlightModeAssist == FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST &&
                           newAssistedThrottleState != FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL) {
                    // ok auto thrust mode applies in hold unless overridden

                    if (throttleNeutralOrLow && flagRollPitchHasInput) {
                        // throttle is neutral approximately and stick input present so revert to primary mode control
                        newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
                        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                    } else {
                        // otherwise hold, autothrust, no stick input...we watch throttle for need to change mode
                        switch (newAssistedThrottleState) {
                        case FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO:
                            // whilst in auto, watch for neutral range, from which we allow override
                            if (throttleNeutral) {
                                pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTOOVERRIDE;
                            } else { pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO; }
                            break;
                        case FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTOOVERRIDE:
                            // previously user has set throttle to neutral range, apply a deadband and revert to manual
                            // if moving out of deadband.  This allows for landing in hold state.
                            if (!throttleNeutral) {
                                pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
                            } else { pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTOOVERRIDE; }
                            break;
                        case FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL:
                            pathfollowerthrustmode = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
                            break;
                        }

                        newAssistedThrottleState = pathfollowerthrustmode;
                        handler = &handler_PATHFOLLOWER;
                    }
                }
                break;
            }
        }
#endif /* ifndef PIOS_EXCLUDE_ADVANCED_FEATURES */
        break;
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES

    case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
        newFlightModeAssist = isAssistedFlightMode(position, newMode, &modeSettings);
        if (newFlightModeAssist) {
            switch (newFlightModeAssist) {
            case FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST_PRIMARYTHRUST:
                newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
                break;
            case FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST:
                newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO;
                break;
            case FLIGHTSTATUS_FLIGHTMODEASSIST_NONE:
            default:
                newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL; // Effectively None
                break;
            }

            switch (newAssistedControlState) {
            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE:
                newAssistedControlState = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE;
                break;
            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY:
                newAssistedControlState = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE;
                break;
            case FLIGHTSTATUS_ASSISTEDCONTROLSTATE_HOLD:
                newAssistedControlState = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_HOLD;
                break;
            }
        }
        handler = &handler_PATHFOLLOWER;
        break;

    case FLIGHTSTATUS_FLIGHTMODE_COURSELOCK:
    case FLIGHTSTATUS_FLIGHTMODE_POSITIONROAM:
    case FLIGHTSTATUS_FLIGHTMODE_HOMELEASH:
    case FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION:
    case FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE:
    case FLIGHTSTATUS_FLIGHTMODE_LAND:
    case FLIGHTSTATUS_FLIGHTMODE_POI:
    case FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE:
        handler = &handler_PATHFOLLOWER;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
        handler = &handler_PATHPLANNER;
        break;
#endif /* ifndef PIOS_EXCLUDE_ADVANCED_FEATURES */
        // There is no default, so if a flightmode is forgotten the compiler can throw a warning!
    }

    bool newinit = false;

    // FlightMode needs to be set correctly on first run (otherwise ControlChain is invalid)
    static bool firstRun = true;

    if (flightStatus.FlightMode != newMode || firstRun ||
        newFlightModeAssist != flightStatus.FlightModeAssist ||
        newAssistedControlState != flightStatus.AssistedControlState ||
        flightStatus.AssistedThrottleState != newAssistedThrottleState) {
        firstRun = false;
        flightStatus.ControlChain          = handler->controlChain;
        flightStatus.FlightMode            = newMode;
        flightStatus.FlightModeAssist      = newFlightModeAssist;
        flightStatus.AssistedControlState  = newAssistedControlState;
        flightStatus.AssistedThrottleState = newAssistedThrottleState;
        FlightStatusSet(&flightStatus);
        newinit = true;
    }
    if (handler->handler) {
        handler->handler(newinit);
    }
}

/**
 * Called whenever a critical configuration component changes
 */
static void configurationUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    configuration_check();
}

/**
 * Called whenever a critical configuration component changes
 */
static void commandUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
}


/**
 * Check and set modes for gps assisted stablised flight modes
 */
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
static uint8_t isAssistedFlightMode(uint8_t position, uint8_t flightMode, FlightModeSettingsData *modeSettings)
{
    uint8_t flightModeAssistOption = STABILIZATIONSETTINGS_FLIGHTMODEASSISTMAP_NONE;
    uint8_t isAssistedFlag = FLIGHTSTATUS_FLIGHTMODEASSIST_NONE;
    uint8_t FlightModeAssistMap[STABILIZATIONSETTINGS_FLIGHTMODEASSISTMAP_NUMELEM];

    StabilizationSettingsFlightModeAssistMapGet(FlightModeAssistMap);
    if (position < STABILIZATIONSETTINGS_FLIGHTMODEASSISTMAP_NUMELEM) {
        flightModeAssistOption = FlightModeAssistMap[position];
    }

    switch (flightModeAssistOption) {
    case STABILIZATIONSETTINGS_FLIGHTMODEASSISTMAP_NONE:
        break;
    case STABILIZATIONSETTINGS_FLIGHTMODEASSISTMAP_GPSASSIST:
    {
        // default to cruise control.
        FlightModeSettingsStabilization1SettingsOptions thrustMode = FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_CRUISECONTROL;

        switch (flightMode) {
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
            thrustMode = modeSettings->Stabilization1Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
            thrustMode = modeSettings->Stabilization2Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
            thrustMode = modeSettings->Stabilization3Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
            thrustMode = modeSettings->Stabilization4Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
            thrustMode = modeSettings->Stabilization5Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
            thrustMode = modeSettings->Stabilization6Settings.Thrust;
            break;
        case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
            // we hard code the "GPS Assisted" PostionHold to use alt-vario which
            // is a more appropriate throttle mode.  "GPSAssist" adds braking
            // and a better throttle management to the standard Position Hold.
            thrustMode = FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_ALTITUDEVARIO;
            break;

            // other modes will use cruisecontrol as default
        }


        switch (thrustMode) {
        case FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_ALTITUDEHOLD:
        case FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_ALTITUDEVARIO:
            // this is only for use with stabi mods with althold/vario.
            isAssistedFlag = FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST_PRIMARYTHRUST;
            break;
        case FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_MANUAL:
        case FLIGHTMODESETTINGS_STABILIZATION1SETTINGS_CRUISECONTROL:
        default:
            // this is the default for non stabi modes also
            isAssistedFlag = FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST;
            break;
        }
    }
    break;
    }

    return isAssistedFlag;
}
#endif /* ifndef PIOS_EXCLUDE_ADVANCED_FEATURES */

#if defined(PIOS_INCLUDE_BARO)
static bool isVelocityLessThan(float *velocityDatas, uint8_t length, float velocity, float probability)
{
	uint8_t i, num = 0;
	for(i=0; i<length; i++){
		if(velocityDatas[i] < velocity){
			num++;
		}
	}

	return (((float)num / (float)length) >= probability);
}

static bool isVelocityMoreThan(float *velocityDatas, uint8_t length, float velocity, float probability)
{
	uint8_t i, num = 0;
	for(i=0; i<length; i++){
		if(velocityDatas[i] > velocity){
			num++;
		}
	}

	return (((float)num / (float)length) >= probability);
}
#endif

/**
 * @}
 * @}
 */
