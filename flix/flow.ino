// Horizontal velocity estimation from a downward optical flow sensor (PMW3901-like) and velocity hold

#include "vector.h"
#include "pid.h"
#include "util.h"

extern float t;
extern Vector gyro;
extern bool armed;
extern float controlRoll, controlPitch, tiltMax;

float flowX = 0, flowY = 0; // latest flow, rad/s, in the body axes (see estimateFlow for the sign convention)
int flowQuality = 0; // surface quality reported by the sensor, 0-255
bool flowUpdated = false; // set by the flow driver when there is a new reading

float velX = 0, velY = 0; // estimated horizontal velocity in the body frame (forward, left), m/s
bool flowValid = false; // velocity estimate is trustworthy

int flowQualityMin = 50; // readings with lower surface quality are ignored (0 disables the check)
float flowRangeMin = 0.1; // m, the sensor is blind when closer to the ground
float flowRangeMax = 3.5; // m, the sensor is unreliable when higher
float flowRateMax = 6; // rad/s, readings near the sensor limit are ignored
float flowFilterA = 0.3; // low-pass filter gain per reading
float flowLossTimeout = 0.25; // s, how long to coast without valid readings

float velMax = 1.0; // m/s at full stick
float velTiltMax = radians(15); // max tilt commanded by the velocity controller
PID velXPID(0.15, 0.05, 0, 0.1);
PID velYPID(0.15, 0.05, 0, 0.1);

void syncVelocityPID() { // the Y axis uses the same gains as X
	velYPID.p = velXPID.p;
	velYPID.i = velXPID.i;
	velYPID.windup = velXPID.windup;
}

static float flowTime = NAN; // time of the last valid reading

void estimateFlow() {
	if (flowUpdated) {
		flowUpdated = false;
		float h = rangefinderRange; // distance to the ground along the sensor axis
		bool good = flowQuality >= flowQualityMin && !isnan(h) && h >= flowRangeMin && h <= flowRangeMax &&
			fabs(flowX) < flowRateMax && fabs(flowY) < flowRateMax;
		if (good) {
			// A ground point below the sensor moves with angular rate (-v / h + rotation of the body):
			// flowX = -vx / h + gyro.y, flowY = -vy / h - gyro.x
			float vx = (gyro.y - flowX) * h;
			float vy = -(flowY + gyro.x) * h;
			if (flowValid) {
				velX += flowFilterA * (vx - velX);
				velY += flowFilterA * (vy - velY);
			} else {
				velX = vx;
				velY = vy;
			}
			flowValid = true;
			flowTime = t;
		}
	}

	if (flowValid && t - flowTime > flowLossTimeout) {
		flowValid = false; // lost the ground
		velX = velY = 0;
	}
}

// Replace the roll and pitch sticks (range [-1, 1]) with the output of the velocity controller
void controlVelocity(float& roll, float& pitch) {
	static bool active = false;

	if (!armed || !flowValid || isnan(altitude)) { // no velocity information: plain angle control by the sticks
		if (active && armed) print("Flow lost, velocity hold off\n");
		active = false;
		velXPID.reset();
		velYPID.reset();
		return;
	}

	if (!active) print("Velocity hold on\n");
	active = true;

	float targetX = controlPitch * velMax; // pitch stick forward is forward
	float targetY = -controlRoll * velMax; // roll stick right is towards -y (left is +y)
	pitch = constrain(velXPID.update(targetX - velX), -velTiltMax, velTiltMax) / tiltMax;
	roll = -constrain(velYPID.update(targetY - velY), -velTiltMax, velTiltMax) / tiltMax;
}
