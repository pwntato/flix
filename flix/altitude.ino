// Altitude estimation from a downward rangefinder, bridged by a barometer where the rangefinder has no ground in range,
// and altitude hold

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "util.h"

extern float t, dt;
extern float controlThrottle;
extern Quaternion attitude;
extern bool armed;
extern Vector acc;

float rangefinderRange = NAN; // latest rangefinder reading, m (NAN if invalid or out of range)
bool rangefinderUpdated = false; // set by the rangefinder driver when there is a new reading
float altitude = NAN; // estimated height above the ground, m (NAN if unknown)
float climbRate = 0; // estimated vertical speed, m/s

float altitudeTarget = NAN; // altitude to hold, m
float altitudeP = 2.0; // altitude error to climb rate target, 1/s
float climbRateMax = 1.0; // m/s
float altitudeMax = 8.0; // m, the barometer takes over above the rangefinder maximum range (4 m)
float hoverThrust = 0.53; // initial guess, the velocity PID integral compensates the difference
float altFilterA = 0.3; // altitude correction gain per measurement
float altFilterB = 0.05; // climb rate correction gain per measurement
float altLossTimeout = 0.5; // s, how long to coast without valid readings

float baroPressure = NAN; // latest barometer reading, Pa
bool baroUpdated = false; // set by the barometer driver when there is a new reading
float baroAltitude = NAN; // pressure altitude from the standard atmosphere (smoothed), m: only differences mean anything
float baroOffset = NAN; // baroAltitude minus the height above the ground, learned while the rangefinder sees the ground, m
float baroSmooth = 0.3; // low-pass gain per barometer reading
float baroOffsetGain = 0.02; // offset correction gain per rangefinder reading: follows the weather drift while the ground is in view
float baroBandwidth = 2.5; // rad/s, how fast the barometer corrects the altitude (third order complementary filter with the accelerometer)
float baroAccBias = 0; // vertical accelerometer bias learned from the barometer, m/s^2
float rangeGate = 1.0; // m, a rangefinder reading this far from the barometer altitude is ignored ...
int rangeGateCount = 10; // ... until this many in a row agree with each other (the ground really is at a different height)
PID climbRatePID(0.5, 0.6, 0, 0.3);

static float altitudeTime = NAN; // time of the last valid reading
static float rangeValidTime = NAN; // time of the last valid rangefinder reading
static float baroTime = NAN; // time of the last barometer reading
static float rangeCorrTime = NAN; // time of the last rangefinder correction
static int rangeOutliers = 0; // readings in a row that the barometer contradicts
static float rangeOutlier = NAN; // the latest of them
static float rangeOutlierTime = NAN; // and when it came

void estimateAltitude() {
	// where the rangefinder has no ground in range the barometer keeps the altitude going, aided by the accelerometer
	bool baroMode = !isnan(baroOffset) && (isnan(rangeValidTime) || t - rangeValidTime > 0.1);

	// alpha-beta filter: predict with the climb rate, correct with each new reading
	if (!isnan(altitude)) {
		if (baroMode) { // the barometer is slow and noisy: without the accelerometer the climb rate lags the real motion
			Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
			Vector linear = acc - up * ONE_G;
			float accelUp = linear.x * up.x + linear.y * up.y + linear.z * up.z;
			climbRate += (accelUp - baroAccBias) * dt;
		}
		altitude += climbRate * dt;
	}

	if (rangefinderUpdated) {
		rangefinderUpdated = false;
		if (!isnan(rangefinderRange)) {
			Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
			float height = rangefinderRange * up.z; // compensate for tilt
			bool outlier = false; // the barometer says the height is somewhere else: a glitch, or the ground changed
			if (!isnan(baroOffset) && !isnan(baroAltitude) && !isnan(altitude) && fabs(height - (baroAltitude - baroOffset)) > rangeGate) {
				if (rangeOutliers > 0 && t - rangeOutlierTime < 0.15 && fabs(height - rangeOutlier) < 0.3) rangeOutliers++; // consecutive readings only
				else rangeOutliers = 1;
				rangeOutlier = height;
				rangeOutlierTime = t;
				if (rangeOutliers >= rangeGateCount) { // it persists: the ground is at a different height now, learn the barometer again
					baroOffset = NAN;
					rangeOutliers = 0;
					print("Ground height changed, barometer offset reset\n");
				} else {
					outlier = true;
				}
			} else {
				rangeOutliers = 0;
			}
			if (!outlier) {
				if (isnan(altitude)) {
					altitude = height;
					climbRate = 0;
				} else {
					float residual = height - altitude;
					// time since the previous rangefinder correction, not since the barometer's (which ran while the ground was out of range)
					float measDt = isnan(rangeCorrTime) ? 0.05f : fminf(t - rangeCorrTime, 0.2f);
					altitude += altFilterA * residual;
					if (measDt > 0) climbRate += altFilterB * residual / measDt;
				}
				altitudeTime = t;
				rangeValidTime = t;
				rangeCorrTime = t;
				if (!isnan(baroAltitude)) { // learn how the barometer relates to the height above the ground
					float sample = baroAltitude - height;
					baroOffset = isnan(baroOffset) ? sample : baroOffset + baroOffsetGain * (sample - baroOffset);
				}
			}
		}
	}

	if (baroUpdated) {
		baroUpdated = false;
		if (!isnan(baroPressure) && baroPressure > 1000) {
			float raw = 44330 * (1 - powf(baroPressure / 101325, 0.1903));
			baroAltitude = isnan(baroAltitude) ? raw : baroAltitude + baroSmooth * (raw - baroAltitude);
			if (baroMode && !isnan(altitude)) {
				float residual = baroAltitude - baroOffset - altitude;
				float measDt = isnan(baroTime) ? 0 : fminf(t - baroTime, 0.1f);
				float w = baroBandwidth;
				altitude += 3 * w * residual * measDt;
				climbRate += 3 * w * w * residual * measDt;
				baroAccBias -= w * w * w * residual * measDt;
				altitudeTime = t;
			}
			baroTime = t;
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
