// Horizontal velocity estimation from a downward optical flow sensor (PMW3901-like) fused with the accelerometer,
// and velocity hold

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "util.h"

extern float t, dt;
extern Vector gyro, acc;
extern Quaternion attitude;
extern bool armed;
extern bool landed;
extern float climbRate;
extern float rangefinderRange, altitude;
extern float controlRoll, controlPitch, tiltMax;

float flowX = 0, flowY = 0; // latest flow, rad/s, in the body axes (see estimateFlow for the sign convention)
int flowQuality = 0; // surface quality reported by the sensor, 0-255
bool flowUpdated = false; // set by the flow driver when there is a new reading

float velX = 0, velY = 0; // estimated horizontal velocity in the body frame (forward, left), m/s
float velSigma = INFINITY; // uncertainty of the velocity estimate, m/s
bool flowValid = false; // velocity estimate is trustworthy
bool flowTrusted = false; // the latest usable flow reading agreed with the accelerometer

int flowQualityMin = 50; // readings with lower surface quality are ignored (0 disables the check)
float flowRangeMin = 0.1; // m, the sensor is blind when closer to the ground
float flowRangeMax = 3.5; // m, the sensor is unreliable when higher
float flowRateMax = 6; // rad/s, readings near the sensor limit are ignored
float flowNoise = 0.04; // m/s, flow velocity noise at good quality
float flowGate = 3; // flow readings further than this many sigmas from the prediction are rejected
float accelNoise = 0.15; // m/s^2 per sqrt(Hz), accelerometer noise including attitude error
float accelBiasNoise = 0.02; // m/s^2 per sqrt(s), how fast the accelerometer bias may change
float velSigmaMax = 0.25; // m/s, the estimate is not used above this uncertainty
float flowLossTimeout = 0.5; // s, give up this long after the last accepted flow reading (the accelerometer cannot vouch for longer)

float velMax = 1.0; // m/s at full stick
float velTiltMax = radians(15); // max tilt commanded by the velocity controller
PID velXPID(0.15, 0.05, 0, 0.1);
PID velYPID(0.15, 0.05, 0, 0.1);

void syncVelocityPID() { // the Y axis uses the same gains as X
	velYPID.p = velXPID.p;
	velYPID.i = velXPID.i;
	velYPID.windup = velXPID.windup;
}

// Kalman filter of one axis: velocity and accelerometer bias
struct VelocityFilter {
	float v = 0, bias = 0;
	float p00 = 1, p01 = 0, p11 = 0.04; // covariance

	void predict(float accel, float coriolis) {
		v += (accel - bias - coriolis) * dt;
		p00 += -2 * dt * p01 + dt * dt * p11 + accelNoise * accelNoise * dt;
		p01 += -dt * p11;
		p11 += accelBiasNoise * accelBiasNoise * dt;
	}

	// returns false if the measurement is too far from the prediction and was ignored
	// gateNoise is the expected noise at good quality: bad-quality readings are weighed less, but not let through
	bool update(float z, float noise, float gateNoise = 0) {
		float y = z - v;
		float s = p00 + noise * noise;
		if (gateNoise > 0 && y * y > flowGate * flowGate * (p00 + gateNoise * gateNoise)) return false;
		float k0 = p00 / s, k1 = p01 / s;
		v += k0 * y;
		bias += k1 * y;
		p11 -= k1 * p01;
		p01 *= 1 - k0;
		p00 *= 1 - k0;
		return true;
	}

	void init(float z, float noise) {
		v = z;
		p00 = noise * noise;
		p01 = 0;
	}
} velFilterX, velFilterY;

static bool velInitialized = false;
static float flowTime = NAN; // time of the last accepted flow reading

void estimateFlow() {
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);

	if (velInitialized) {
		// linear acceleration in the body frame, and its rotation term (the body frame turns with the drone)
		Vector linear = acc - up * ONE_G;
		float vz = climbRate * up.z;
		velFilterX.predict(linear.x, gyro.y * vz - gyro.z * velFilterY.v);
		velFilterY.predict(linear.y, gyro.z * velFilterX.v - gyro.x * vz);
	}

	if (landed) { // standing still on the ground: known velocity, calibrates the bias
		if (!velInitialized) { velFilterX.init(0, 0.02); velFilterY.init(0, 0.02); velInitialized = true; }
		velFilterX.update(0, 0.02);
		velFilterY.update(0, 0.02);
		flowTime = t;
	}

	if (flowUpdated) {
		flowUpdated = false;
		float h = rangefinderRange; // distance to the ground along the sensor axis
		bool usable = flowQuality >= flowQualityMin && !isnan(h) && h >= flowRangeMin && h <= flowRangeMax &&
			fabs(flowX) < flowRateMax && fabs(flowY) < flowRateMax;
		if (usable) {
			// A ground point below the sensor moves with angular rate (-v / h + rotation of the body):
			// flowX = -vx / h + gyro.y, flowY = -vy / h - gyro.x
			float vx = (gyro.y - flowX) * h;
			float vy = -(flowY + gyro.x) * h;
			float gateNoise = fmaxf(flowNoise, 0.03f * h);
			float noise = gateNoise * constrain(200.0f / fmaxf(flowQuality, 25), 1.0f, 8.0f); // worse surface, less trust
			if (!velInitialized) {
				velFilterX.init(vx, noise);
				velFilterY.init(vy, noise);
				velInitialized = true;
				flowTrusted = true;
				flowTime = t;
			} else {
				bool okX = velFilterX.update(vx, noise, gateNoise);
				bool okY = velFilterY.update(vy, noise, gateNoise);
				flowTrusted = okX && okY;
				if (flowTrusted) flowTime = t;
			}
		}
	}

	velX = velFilterX.v;
	velY = velFilterY.v;
	velSigma = sqrt(fmaxf(velFilterX.p00, velFilterY.p00));
	flowValid = velInitialized && velSigma <= velSigmaMax && t - flowTime <= flowLossTimeout;
	if (!flowValid) velInitialized = false; // start over with the next good reading
	if (!flowValid) velX = velY = 0;
}

// Replace the roll and pitch sticks (range [-1, 1]) with the output of the velocity controller.
// The sticks never change meaning while deflected: velocity hold engages only with centered sticks, and when the
// velocity is lost with deflected sticks the drone levels out and ignores roll and pitch until they are centered.
void controlVelocity(float& roll, float& pitch) {
	static bool active = false, latched = false, trusted = true;
	bool centered = fabs(controlRoll) < 0.1 && fabs(controlPitch) < 0.1;
	bool available = flowValid && !isnan(altitude);

	if (!armed) {
		active = latched = false;
		velXPID.reset();
		velYPID.reset();
		return;
	}

	if (latched) {
		if (!centered) {
			roll = pitch = 0;
			return;
		}
		latched = false;
		print("Sticks centered, control restored\n");
	}

	if (active && !available) {
		active = false;
		velXPID.reset();
		velYPID.reset();
		if (centered) {
			print("Velocity lost, velocity hold off\n");
		} else {
			latched = true;
			roll = pitch = 0;
			print("Velocity lost, leveling out, center the sticks to take over\n");
		}
		return;
	}

	if (!active) {
		if (!available || !centered) return; // plain angle control by the sticks
		active = true;
		print("Velocity hold on\n");
	}

	if (trusted && !flowTrusted) print("Flow disagrees with the accelerometer, ignoring it\n");
	trusted = flowTrusted;

	float targetX = controlPitch * velMax; // pitch stick forward is forward
	float targetY = -controlRoll * velMax; // roll stick right is towards -y (left is +y)
	pitch = constrain(velXPID.update(targetX - velX), -velTiltMax, velTiltMax) / tiltMax;
	roll = -constrain(velYPID.update(targetY - velY), -velTiltMax, velTiltMax) / tiltMax;
}
