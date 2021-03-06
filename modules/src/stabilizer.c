/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
#include "lps25h.h"
#include "debug.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

#define ON_FLAG			0x1
#define START_FLAG		0x02
#define LAND_FLAG		0x04
#define CANCEL_FLAG		0x08
#define COMMAND_FLAG	0x10
#define PARAM_FLAG		0xFF00

typedef enum { MODE_OFF, MODE_START, MODE_LAND, MODE_HOVER, MODE_COMMAND, MODE_CANCEL } MODES;

static MODES currentMode = MODE_OFF;
static uint16_t cmdParam = 0;

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0;
static float accMAG    = 0.0;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust




static float klqr[4][12] = {
	{ 0.013219506, -0.013219506, -0.01539505,  0.001147392, -0.001147392, -0.0003576, -0.475786019, -0.475786019,  0.497825910, -0.279414560, -0.279414560,  0.248906127 },
	{ -0.013219506, -0.013219506,  0.01539505, -0.001147392, -0.001147392,  0.0003576,  0.475786016, -0.475786015,  0.497825911,  0.279414558, -0.279414558,  0.248906127 },
	{ -0.013219506,  0.013219506, -0.01539505, -0.001147392,  0.001147392, -0.0003576,  0.475786019,  0.475786022,  0.497825913,  0.279414560,  0.279414562,  0.248906128 },
	{ 0.013219506,  0.013219506,  0.01539505,  0.001147392,  0.001147392,  0.0003576, -0.475786015,  0.475786017,  0.497825912, -0.279414558,  0.279414559,  0.248906128 } };

static float kr[4][12] = {
	{ 0.0006806, -0.0006806, -0.0, -0.6806,  0.6806,  0.00011612, -0.0, -0.0,  0.028375,  0.0,  0.0, -28.375 },
	{ -0.0006806, -0.0006806,  0.0,  0.6806,  0.6806, -0.00011612,  0.0, -0.0,  0.028375, -0.0,  0.0, -28.375 },
	{ -0.0006806,  0.0006806, -0.0,  0.6806, -0.6806,  0.00011612,  0.0,  0.0,  0.028375, -0.0, -0.0, -28.375 },
	{ 0.0006806,  0.0006806,  0.0, -0.6806, -0.6806, -0.00011612, -0.0,  0.0,  0.028375,  0.0, -0.0, -28.375 } };

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t modeFlags;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void setThrust(const uint16_t thrust1, const uint16_t thrust2, const uint16_t thrust3, const uint16_t thrust4);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);
static void mxv(float u[4], float m[4][12], float v[12]);
static void runController();

static int vbatid;
static float vbat;

static uint8_t i = 0;
static uint8_t j = 0;
static float x[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float dx[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float x_op[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float kdx[4] = { 0, 0, 0, 0};
static float r[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float dr[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static float r_op[4] = { 0, 0, 0, 0};
//static float u_op[4] = { 2.275, 2.275, 2.275, 2.275 };
static float u_op[4] = { 2.15f, 2.15f, 2.15f, 2.15f};
//static float u_op[4] = { 0.0, 0.0, 0.0, 0.0};

static float u[4] = { 0.0, 0.0, 0.0, 0.0};
static float du[4] = { 0, 0, 0, 0};

static float pwmPerVolt;

static void mxv(float u[4], float m[4][12], float v[12]) {
	for (i = 0; i < 4; ++i) {
		u[i] = 0;
		for (j = 0; j < 12; ++j)
			u[i] += m[i][j] * v[j];
	}
}

static void sub(float out[12], float v1[12], float v2[12]) {
	for (i = 0; i < 12; ++i)
		out[i] = v1[i] - v2[i];
}

static void add(float out[12], float v1[12], float v2[12]) {
	for (i = 0; i < 12; ++i)
		out[i] = v1[i] + v2[i];
}


void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, (const signed char * const)STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  vbatid = logGetVarId("pm", "vbat");

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);
        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
      }

      // 100HZ
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&modeFlags);
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }

      if (modeFlags & ON_FLAG)
      {
		  switch (currentMode)
		  {
		  case MODE_OFF:
			  setThrust(0, 0, 0, 0);
			  if (modeFlags & START_FLAG) {
				  currentMode = MODE_START;
			  }
			  break;

		  case MODE_HOVER:
			  if (modeFlags & LAND_FLAG)
				  currentMode = MODE_LAND;
			  else
				  runController();
			  break;

		  case MODE_LAND:
			  //TODO: Set a lower thrust liek 2000 and a delay isntead
			  setThrust(0, 0, 0, 0);
			  currentMode = MODE_OFF;
			  break;

		  case MODE_START:
			  currentMode = MODE_HOVER;
			  break;

		  case MODE_COMMAND:
			  //Not in use since not all states are stable yet
			  cmdParam = modeFlags & PARAM_FLAG;
			  cmdParam >>= 8;
			  break;

		  case MODE_CANCEL:
			  r[0] = 0;
			  r[1] = 0;
			  r[2] = 0;
			  r[3] = 0;
			  r[4] = 0;
			  r[5] = 0;
			  r[6] = 0;
			  r[7] = 0;
			  r[8] = 0;
			  r[9] = 0;
			  r[10] = 0;
			  r[11] = 0;

			  //TODO: Wait for the states values before switching
			  currentMode = MODE_HOVER;
			  break;

		  default:
			  break;
		  }

      }
      else
      {
        distributePower(0, 0, 0, 0);
      }
    }
  }
}

static void runController() {
	//distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
	x[0] = eulerPitchActual;
	x[1] = eulerRollActual;
	x[2] = eulerYawActual;
	x[3] = -gyro.y;
	x[4] = gyro.x;
	x[5] = gyro.z;
	x[6] = eulerRollDesired / 1000.f;
	x[7] = eulerPitchDesired / 1000.f;
	x[8] = eulerYawDesired / 1000.f;
	x[9] = 0;
	x[10] = 0;
	x[11] = vSpeed;

	//Turn off states
	x[6] = 0;
	x[7] = 0;
	x[8] = 0;
	x[9] = 0;
	x[10] = 0;
	x[11] = 0;


	//LQR
	sub(dx, x, x_op);
	sub(dr, r, x_op);

	mxv(r_op, kr, dr);
	mxv(kdx, klqr, dx);

	sub(du, r_op, kdx);

	add(u, du, u_op);


	//Thrust
	vbat = logGetFloat(vbatid);
	pwmPerVolt = 65535.f / vbat;

	setThrust(
		u[0] * pwmPerVolt,
		u[1] * pwmPerVolt,
		u[2] * pwmPerVolt,
		u[3] * pwmPerVolt);
}

static void stabilizerAltHoldUpdate(void)
{
  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
  lps25hGetData(&pressure, &temperature, &aslRaw);

  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    modeFlags =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
  }
}

static void setThrust(const uint16_t thrust1, const uint16_t thrust2, const uint16_t thrust3, const uint16_t thrust4)
{
	motorPowerM1 = limitThrust(thrust1);
	motorPowerM2 = limitThrust(thrust2);
	motorPowerM3 = limitThrust(thrust3);
	motorPowerM4 = limitThrust(thrust4);

	motorsSetRatio(MOTOR_M1, motorPowerM1);
	motorsSetRatio(MOTOR_M2, motorPowerM2);
	motorsSetRatio(MOTOR_M3, motorPowerM3);
	motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  int16_t r = roll >> 1;
  int16_t p = pitch >> 1;
  motorPowerM1 = limitThrust(thrust - r + p + yaw);
  motorPowerM2 = limitThrust(thrust - r - p - yaw);
  motorPowerM3 =  limitThrust(thrust + r - p + yaw);
  motorPowerM4 =  limitThrust(thrust + r + p - yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &modeFlags)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)

