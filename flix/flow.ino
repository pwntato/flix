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
extern float wallVelX, wallVelY, wallLag;
extern float wallHold(int axis, bool centered, float speed);
extern bool wallVelUpdated;

float flowX = 0, flowY = 0; // latest flow, rad/s, in the body axes (see estimateFlow for the sign convention)
int flowQuality = 0; // surface quality reported by the sensor, 0-255
bool flowUpdated = false; // set by the flow driver when there is a new reading

float velX = 0, velY = 0; // estimated horizontal velocity in the body frame (forward, left), m/s
float velSigma = INFINITY; // uncertainty of the velocity estimate, m/s
bool flowValid = false; // velocity estimate is trustworthy on both axes
bool velValid[2] = { false, false }; // ... on each axis (forward, left): the walls can keep one when the flow is lost
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

float wallMismatch = 0.15; // m/s, the flow velocity may differ this much from the wall distance change
float wallVelNoise = 0.05; // m/s, noise of the wall velocity
float wallTakeoverDelay = 0.15; // s, the walls provide the velocity when the flow has not been accepted this long
float wallMismatchTime = 0.3; // s, flow is declared wrong after disagreeing with the walls this long (and right again after agreeing 3x as long)

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
static float velTime[2] = { NAN, NAN }; // time of the last accepted measurement of each axis, from the flow or the walls
static float flowAcceptedTime = NAN; // time of the last accepted flow reading
static float wallsSince = NAN; // since when the walls, not the flow, provide the velocity (NAN if not)
static bool wallsAnnounced = false;

// Compare the flow with the walls, which measure the speed independently: a surface that makes the flow sensor read the
// wrong speed at good quality (a repeating pattern) is invisible to the accelerometer at constant speed, but not to them.
// Returns true while the flow is declared wrong.
static bool flowVetoed(float vx, float vy, const Vector& linear) {
	static float disagree = 0, agree = 0, compared = NAN;
	static bool vetoed = false;
	float measDt = isnan(compared) ? 0 : fminf(t - compared, 0.1f);
	compared = t;
	if (!velInitialized) {
		vetoed = false;
		disagree = agree = 0;
		return false;
	}
	bool comparable = !isnan(wallVelX) || !isnan(wallVelY);
	// the wall velocity lags behind by its filter delay, which matters when speeding up
	float errX = isnan(wallVelX) ? 0 : fabs(vx - wallVelX) - wallLag * fabs(linear.x);
	float errY = isnan(wallVelY) ? 0 : fabs(vy - wallVelY) - wallLag * fabs(linear.y);
	if (comparable && fmaxf(errX, errY) > wallMismatch) { disagree += measDt; agree = 0; }
	else if (comparable) { agree += measDt; disagree = 0; }
	else disagree = agree = 0;
	if (!vetoed && disagree > wallMismatchTime) { vetoed = true; print("Flow disagrees with the walls, ignoring it\n"); }
	if (vetoed && agree > 3 * wallMismatchTime) { vetoed = false; print("Flow agrees with the walls again\n"); }
	return vetoed;
}

void estimateFlow() {
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector linear = acc - up * ONE_G; // linear acceleration in the body frame
	static bool vetoed = false;

	if (velInitialized) {
		// the rotation term: the body frame turns with the drone
		float vz = climbRate * up.z;
		velFilterX.predict(linear.x, gyro.y * vz - gyro.z * velFilterY.v);
		velFilterY.predict(linear.y, gyro.z * velFilterX.v - gyro.x * vz);
	}

	if (landed) { // standing still on the ground: known velocity, calibrates the bias
		if (!velInitialized) { velFilterX.init(0, 0.02); velFilterY.init(0, 0.02); velInitialized = true; }
		velFilterX.update(0, 0.02);
		velFilterY.update(0, 0.02);
		velTime[0] = velTime[1] = flowAcceptedTime = t;
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
			vetoed = flowVetoed(vx, vy, linear);
			float gateNoise = fmaxf(flowNoise, 0.03f * h);
			float noise = gateNoise * constrain(200.0f / fmaxf(flowQuality, 25), 1.0f, 8.0f); // worse surface, less trust
			if (vetoed) {
				// the walls provide the velocity
			} else if (!velInitialized) {
				velFilterX.init(vx, noise);
				velFilterY.init(vy, noise);
				velInitialized = true;
				flowTrusted = true;
				velTime[0] = velTime[1] = flowAcceptedTime = t;
			} else {
				bool okX = velFilterX.update(vx, noise, gateNoise);
				bool okY = velFilterY.update(vy, noise, gateNoise);
				flowTrusted = okX && okY;
				if (flowTrusted) velTime[0] = velTime[1] = flowAcceptedTime = t;
			}
		}
	}

	// The walls take over when the flow is vetoed or missing
	if (wallVelUpdated) {
		wallVelUpdated = false;
		bool flowMissing = vetoed || isnan(flowAcceptedTime) || t - flowAcceptedTime > wallTakeoverDelay;
		if (flowMissing && !(isnan(wallVelX) && isnan(wallVelY))) {
			if (!velInitialized && !isnan(wallVelX) && !isnan(wallVelY)) { // both axes needed to start
				velFilterX.init(wallVelX, wallVelNoise);
				velFilterY.init(wallVelY, wallVelNoise);
				velInitialized = true;
				velTime[0] = velTime[1] = t;
			} else if (velInitialized) {
				if (!isnan(wallVelX)) { velFilterX.update(wallVelX, wallVelNoise); velTime[0] = t; }
				if (!isnan(wallVelY)) { velFilterY.update(wallVelY, wallVelNoise); velTime[1] = t; }
			}
			if (isnan(wallsSince) && velInitialized) wallsSince = t;
		}
	}
	if (!isnan(wallsSince) && !vetoed && !isnan(flowAcceptedTime) && t - flowAcceptedTime <= wallTakeoverDelay) { // flow is back
		if (wallsAnnounced) print("Velocity from the flow\n");
		wallsSince = NAN;
		wallsAnnounced = false;
	}
	if (!isnan(wallsSince) && !wallsAnnounced && t - wallsSince > 0.5) { // not just a blip
		wallsAnnounced = true;
		print("Velocity from the walls\n");
	}

	velX = velFilterX.v;
	velY = velFilterY.v;
	velSigma = sqrt(fmaxf(velFilterX.p00, velFilterY.p00));
	// each axis has to be updated recently and be certain enough; the walls can keep one while the flow sees nothing
	velValid[0] = velInitialized && velFilterX.p00 <= velSigmaMax * velSigmaMax && t - velTime[0] <= flowLossTimeout;
	velValid[1] = velInitialized && velFilterY.p00 <= velSigmaMax * velSigmaMax && t - velTime[1] <= flowLossTimeout;
	flowValid = velValid[0] && velValid[1];
	if (!velValid[0] && !velValid[1]) velInitialized = false; // start over with the next good reading
	if (!velValid[0]) velX = 0;
	if (!velValid[1]) velY = 0;
}

// Replace the roll and pitch sticks (range [-1, 1]) with the output of the velocity controller, axis by axis.
// The sticks never change meaning while deflected: velocity hold engages only with centered sticks, and when an axis
// loses its velocity with a deflected stick the drone levels out on that axis and ignores the stick until it is
// centered. With the stick centered the distance to a wall along the axis is held, if there is a wall in view.
void controlVelocity(float& roll, float& pitch) {
	enum { OFF, ON, LATCHED };
	static bool active = false;
	static int state[2] = { OFF, OFF }; // per axis: forward (pitch), left (roll)
	static bool trusted = true;
	const char *name[2] = { "forward", "sideways" };
	float stick[2] = { controlPitch, controlRoll };
	bool centered[2] = { fabs(controlPitch) < 0.1, fabs(controlRoll) < 0.1 };
	bool available[2] = { velValid[0] && !isnan(altitude), velValid[1] && !isnan(altitude) };

	if (!armed) {
		active = false;
		state[0] = state[1] = OFF;
		velXPID.reset();
		velYPID.reset();
		return;
	}

	if (!active) {
		if (!available[0] || !available[1] || !centered[0] || !centered[1]) return; // plain angle control by the sticks
		active = true;
		state[0] = state[1] = ON;
		print("Velocity hold on\n");
	}

	if (trusted && !flowTrusted) print("Flow disagrees with the accelerometer, ignoring it\n");
	trusted = flowTrusted;

	float velocity[2] = { velX, velY };
	float tilt[2] = { pitch, roll }; // output, in the stick range
	PID *pid[2] = { &velXPID, &velYPID };
	for (int a = 0; a < 2; a++) {
		if (state[a] == ON && !available[a]) {
			pid[a]->reset();
			if (centered[a]) {
				state[a] = OFF;
				print("Velocity lost (%s), velocity hold off\n", name[a]);
			} else {
				state[a] = LATCHED;
				print("Velocity lost (%s), leveling out, center the stick to take over\n", name[a]);
			}
		} else if (state[a] != ON && available[a] && centered[a]) {
			if (state[a] == LATCHED) print("Stick centered (%s), control restored\n", name[a]);
			else if (active && state[1 - a] != OFF) print("Velocity back (%s), velocity hold on\n", name[a]);
			state[a] = ON;
		}

		if (state[a] == LATCHED) {
			tilt[a] = 0;
		} else if (state[a] == ON) {
			float target = stick[a] * (a == 0 ? velMax : -velMax); // pitch stick forward is forward, roll stick right is -y
			float hold = wallHold(a, centered[a], velocity[a]);
			if (!isnan(hold)) target += hold;
			float out = constrain(pid[a]->update(target - velocity[a]), -velTiltMax, velTiltMax) / tiltMax;
			tilt[a] = a == 0 ? out : -out;
		} else {
			wallHold(a, false, 0); // forget the distance
		}
	}
	pitch = tilt[0];
	roll = tilt[1];
}
