// Altitude estimation from a downward rangefinder and altitude hold

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "util.h"

extern float t, dt;
extern float controlThrottle;
extern Quaternion attitude;
extern bool armed;

float rangefinderRange = NAN; // latest rangefinder reading, m (NAN if invalid or out of range)
bool rangefinderUpdated = false; // set by the rangefinder driver when there is a new reading
float altitude = NAN; // estimated height above the ground, m (NAN if unknown)
float climbRate = 0; // estimated vertical speed, m/s

float altitudeTarget = NAN; // altitude to hold, m
float altitudeP = 2.0; // altitude error to climb rate target, 1/s
float climbRateMax = 1.0; // m/s
float altitudeMax = 3.0; // m, keep below rangefinder maximum range
float hoverThrust = 0.53; // initial guess, the velocity PID integral compensates the difference
float altFilterA = 0.3; // altitude correction gain per measurement
float altFilterB = 0.05; // climb rate correction gain per measurement
float altLossTimeout = 0.5; // s, how long to coast without valid readings
PID climbRatePID(0.5, 0.6, 0, 0.3);

static float altitudeTime = NAN; // time of the last valid reading

void estimateAltitude() {
	// alpha-beta filter: predict with the climb rate, correct with each new reading
	if (!isnan(altitude)) altitude += climbRate * dt;

	if (rangefinderUpdated) {
		rangefinderUpdated = false;
		if (!isnan(rangefinderRange)) {
			Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
			float height = rangefinderRange * up.z; // compensate for tilt
			if (isnan(altitude)) {
				altitude = height;
				climbRate = 0;
			} else {
				float residual = height - altitude;
				float measDt = t - altitudeTime;
				altitude += altFilterA * residual;
				if (measDt > 0) climbRate += altFilterB * residual / measDt;
			}
			altitudeTime = t;
		}
	}

	if (!isnan(altitude) && t - altitudeTime > altLossTimeout) {
		altitude = NAN; // lost the ground
		climbRate = 0;
	}
}

// Compute thrust for ALT mode: stick centered holds altitude, above/below climbs/descends
float controlAltitude() {
	if (!armed || isnan(altitude)) { // no height information, pass the throttle through
		altitudeTarget = NAN;
		climbRatePID.reset();
		return controlThrottle;
	}

	float stick = controlThrottle - 0.5; // -0.5..0.5, 0 is hold
	float climbRateTarget;
	const float deadZone = 0.1;

	if (fabs(stick) > deadZone) {
		float s = (fabs(stick) - deadZone) / (0.5 - deadZone) * (stick > 0 ? 1 : -1);
		climbRateTarget = s * climbRateMax;
		altitudeTarget = NAN; // recapture when the stick returns to center
	} else {
		if (isnan(altitudeTarget)) altitudeTarget = altitude;
		altitudeTarget = constrain(altitudeTarget, 0.05, altitudeMax);
		climbRateTarget = constrain(altitudeP * (altitudeTarget - altitude), -climbRateMax, climbRateMax);
	}

	if (altitude >= altitudeMax && climbRateTarget > 0) climbRateTarget = 0;

	if (altitude < 0.05 && climbRateTarget <= 0) { // on the ground, no thrust
		climbRatePID.reset();
		return 0;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tiltScale = 1 / constrain(up.z, 0.5, 1); // more thrust is needed to hover when tilted
	return constrain(hoverThrust * tiltScale + climbRatePID.update(climbRateTarget - climbRate), 0.1, 1);
}
