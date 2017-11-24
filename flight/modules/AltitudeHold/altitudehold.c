/**
 ******************************************************************************
 * @addtogroup Openpilot
 * @{
 * @addtogroup AltitudeHoldModule Altitude hold module
 * @{
 *
 * @file       altitudehold.c
 * @author     Openpilot
 * @brief      This module runs an EKF to estimate altitude from just a barometric
 *             sensor and controls throttle to hold a fixed altitude
 *
 * @see        * 
 *
 *****************************************************************************/

/**
 * Input object: @ref PositionActual
 * Input object: @ref VelocityActual
 * Input object: @ref ManualControlCommand
 * Input object: @ref AltitudeHoldDesired
 * Output object: @ref StabilizationDesired
 *
 * Hold the VTOL aircraft at a fixed altitude by running nested
 * control loops to stay at the AltitudeHoldDesired height.
 */

#include "openpilot.h"
#include "pid.h"

#if defined(PIOS_INCLUDE_BARO)

#include "attitudestate.h"
#include "altitudeholdsettings.h"
#include "altitudeholddesired.h"
#include "flightstatus.h"
#include "stabilizationdesired.h"
#include "positionstate.h"
#include "velocitystate.h"
#include "queue.h"
#include "taskinfo.h"
#include "altitudeholdstatus.h"

// Private constants
#define MAX_QUEUE_SIZE 4
#define STACK_SIZE_BYTES 200
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1)

// Private variables
static xTaskHandle altitudeHoldTaskHandle;
static xQueueHandle queue;
static bool module_enabled;

// Private functions
static void altitudeHoldTask(void *parameters);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AltitudeHoldStart()
{
	// Start main task if it is enabled
	
	if (module_enabled) {
		xTaskCreate(altitudeHoldTask, "AltitudeHold", STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &altitudeHoldTaskHandle);
#if defined(PIOS_INCLUDE_TASK_MONITOR)
		PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_ALTITUDE, altitudeHoldTaskHandle);
#endif
		return 0;
	}
	return -1;

}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AltitudeHoldInitialize()
{
	module_enabled = true;

	if(module_enabled) {
		AttitudeStateInitialize();
		PositionStateInitialize();
		VelocityStateInitialize();
		StabilizationDesiredInitialize();
		AltitudeHoldSettingsInitialize();
		AltitudeHoldDesiredInitialize();
		AltitudeHoldStatusInitialize();

		// Create object queue
		queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

		return 0;
	}

	return -1;
}
MODULE_INITCALL(AltitudeHoldInitialize, AltitudeHoldStart);

/**
 * Module thread, should not return.
 */
static void altitudeHoldTask(__attribute__((unused)) void *parameters)
{
	bool engaged = false;

	AltitudeHoldDesiredData altitudeHoldDesired;
	AltitudeHoldSettingsData altitudeHoldSettings;

	UAVObjEvent ev;
	struct pid velocity_pid;

	// Listen for object updates.
	AltitudeHoldDesiredConnectQueue(queue);
	AltitudeHoldSettingsConnectQueue(queue);
	FlightStatusConnectQueue(queue);

	AltitudeHoldSettingsGet(&altitudeHoldSettings);
	pid_configure(&velocity_pid, altitudeHoldSettings.VelocityPI.Kp,
		          altitudeHoldSettings.VelocityPI.Ki, 0.000002f, 1.0f);

	// Main task loop
	const uint32_t dt_ms = 5;
	const float dt_s = dt_ms * 0.001f;
	uint32_t timeout = dt_ms;
	uint8_t last_armed;
	FlightStatusArmedGet(&last_armed);
	AltitudeHoldDesiredGet(&altitudeHoldDesired);

	while (1) {
		if (xQueueReceive(queue, &ev, timeout) != true) {

		} else if (ev.obj == FlightStatusHandle()) {

			uint8_t flight_mode;
			FlightStatusFlightModeGet(&flight_mode);

			if (flight_mode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED1 && !engaged) {
				// Copy the current throttle as a starting point for integral
				StabilizationDesiredThrustGet(&velocity_pid.iAccumulator);
				velocity_pid.iAccumulator *= 1000.0f; // pid library scales up accumulator by 1000
				engaged = true;

				// Make sure this uses a valid AltitudeHoldDesired. No delay is really required here
				// because ManualControl sets AltitudeHoldDesired first before the FlightStatus, but
				// this is just to be conservative at 1ms when engaging will not bother the pilot.
				PIOS_DELAY_WaitmS(1);
				AltitudeHoldDesiredGet(&altitudeHoldDesired);
			} else if (flight_mode != FLIGHTSTATUS_FLIGHTMODE_STABILIZED1)
				engaged = false;

			// Run loop at 20 Hz when engaged otherwise just slowly wait for it to be engaged
			timeout = engaged ? dt_ms : 100;

		} else if (ev.obj == AltitudeHoldDesiredHandle()) {
			AltitudeHoldDesiredGet(&altitudeHoldDesired);
		} else if (ev.obj == AltitudeHoldSettingsHandle()) {
			AltitudeHoldSettingsGet(&altitudeHoldSettings);

			pid_configure(&velocity_pid, altitudeHoldSettings.VelocityPI.Kp,
				          //altitudeHoldSettings.VelocityPI.Ki, 0.02f, 1.0f);
				          altitudeHoldSettings.VelocityPI.Ki, 0.000002f, 1.0f);
		}

		bool landing = altitudeHoldDesired.Land == true;

		// For landing mode allow throttle to go negative to allow the integrals
		// to stop winding up
		const float min_throttle = landing ? -0.1f : 0.0f;

		// When engaged compute altitude controller output
		if (engaged && altitudeHoldDesired.Enable) {
			float position_z, velocity_z, altitude_error;

			PositionStateDownGet(&position_z);
			VelocityStateDownGet(&velocity_z);
			position_z = -position_z; // Use positive up convention
			velocity_z = -velocity_z; // Use positive up convention
			if(position_z < 50 && velocity_z < 3.0f && velocity_z > -3.0f){

				// Compute the altitude error
				altitude_error = altitudeHoldDesired.Altitude - position_z;
				
				// Velocity desired is from the outer controller plus the set point
				float velocity_desired = altitude_error * altitudeHoldSettings.AltitudePI.Kp + altitudeHoldDesired.ClimbRate;
				AltitudeHoldStatusVelocityDesiredSet(&velocity_desired);
				float throttle_desired = pid_apply_antiwindup(&velocity_pid, 
				                    velocity_desired - velocity_z*altitudeHoldSettings.AltitudePI.Ki,
				                    min_throttle, 1.0f, // positive limits since this is throttle
				                    dt_s);

				if (altitudeHoldSettings.ThrustExp > 0) {
					// Throttle desired is at this point the mount desired in the up direction, we can
					// account for the attitude if desired
					AttitudeStateData attitudeActual;
					AttitudeStateGet(&attitudeActual);

					// Project a unit vector pointing up into the body frame and
					// get the z component
					// cos(roll)*cos(pitch)
					float fraction = attitudeActual.q1 * attitudeActual.q1 -
					                 attitudeActual.q2 * attitudeActual.q2 -
					                 attitudeActual.q3 * attitudeActual.q3 +
					                 attitudeActual.q4 * attitudeActual.q4;

					// Add ability to scale up the amount of compensation to achieve
					// level forward flight
					fraction = powf(fraction, (float) altitudeHoldSettings.ThrustExp / 100.0f) / 1.0095f;

					// Dividing by the fraction remaining in the vertical projection will
					// attempt to compensate for tilt. This acts like the thrust is linear
					// with the output which isn't really true. If the fraction is starting
					// to go negative we are inverted and should shut off throttle
					throttle_desired = (fraction > 0.1f) ? (throttle_desired / fraction) : 0.0f;
				}

				float thrust = (throttle_desired>1.0f)?1.0f:(throttle_desired<min_throttle?min_throttle:throttle_desired);
				StabilizationDesiredThrustSet(&thrust);
			}
		}else if(!altitudeHoldDesired.Enable){
			velocity_pid.iAccumulator = 0.0f;
		}

	}
}

#endif

/**
 * @}
 * @}
 */

