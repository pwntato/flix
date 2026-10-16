// Horizontal rangefinders (VL53L1X-like) looking at walls: wall distance and velocity along the body axes,
// an independent cross-check for the optical flow velocity

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t, dt;
extern Vector gyro;
extern Quaternion attitude;
extern float velX, velY; // fused velocity estimate (flow.ino)
extern bool velValid[2];

const int WALL_COUNT = 4; // front, left, back, right
const Vector wallDirection[WALL_COUNT] = { Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0) }; // body axes

float wallRange[WALL_COUNT] = { NAN, NAN, NAN, NAN }; // latest readings, m (NAN if no wall in range)
bool wallUpdated[WALL_COUNT] = { false, false, false, false }; // set by the driver when there is a new reading
float wallDistance[WALL_COUNT] = { NAN, NAN, NAN, NAN }; // filtered horizontal distance to the wall, m
float wallRate[WALL_COUNT] = { 0, 0, 0, 0 }; // d(distance)/dt, m/s
bool wallVelUpdated = false; // set when there is a new wall velocity
float wallLag = 0.1; // s, delay of the wall velocity (the rate filter)
float wallVelX = NAN, wallVelY = NAN; // velocity in the body frame (forward, left) from the wall distance change, NAN if unknown
float wallVelNoiseBase = 0.05; // m/s, noise of the measured wall velocity
float wallVelSigma = 0.05; // m/s, noise of the wall velocity: larger while a rate is still the carried-over seed
bool wallTurning = false; // the drone is turning fast: the wall rates are not used (the sensors sweep along the walls)

float wallFilterA = 0.3; // distance correction gain per measurement
float wallFilterB = 0.06; // rate correction gain per measurement (lower is smoother but lags, and a lagging wall velocity makes the hold oscillate)
float wallJump = 0.5; // m, a reading this far from the filter means a new surface: start over
float wallSettle = 0.6; // s, the rate is used this long after the wall is acquired
float wallLossTimeout = 0.3; // s, how long to coast without valid readings
float wallGyroMax = 1.5; // rad/s, the rate is not used while turning fast (the sensors sweep along the wall)
float wallSeedTime = 1.0; // s, a rate carried over from the velocity estimate counts as less certain this long
float wallSeedSigma = 0.15; // m/s, noise of the wall velocity while it is carried over

float wallHoldP = 0.4; // 1/s, distance error to velocity target (0 disables the wall position hold)
float wallHoldMax = 0.3; // m/s, the most the hold moves the drone back
float wallHoldSettle = 1.0; // s, the stick centered and the wall in view this long before the distance is captured
float wallHoldSpeedMax = 0.1; // m/s, the drone must be this still along the axis to capture the distance

static float wallTime[WALL_COUNT]; // time of the last valid reading
static float wallSince[WALL_COUNT]; // time the wall was acquired
static float wallSeededUntil[WALL_COUNT] = { 0, 0, 0, 0 }; // the rate was seeded from the velocity estimate until this time

void estimateWalls() {
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude); // world up in the body axes

	for (int i = 0; i < WALL_COUNT; i++) {
		if (!isnan(wallDistance[i])) wallDistance[i] += wallRate[i] * dt;

		if (wallUpdated[i]) {
			wallUpdated[i] = false;
			if (!isnan(wallRange[i])) {
				wallVelUpdated = true;
				float elevation = wallDirection[i].x * up.x + wallDirection[i].y * up.y + wallDirection[i].z * up.z; // sin of the ray tilt
				float distance = wallRange[i] * sqrt(fmaxf(1 - elevation * elevation, 0)); // compensate for tilt
				float residual = distance - wallDistance[i];
				if (isnan(wallDistance[i]) || fabs(residual) > wallJump) { // a new surface (a doorway, or the turn put a different wall in view)
					wallDistance[i] = distance;
					int axis = i % 2;
					float v = axis == 0 ? velX : velY;
					if (velValid[axis]) { // carry the velocity over: the distance to the new wall changes at the closing speed
						wallRate[i] = i < 2 ? -v : v;
						wallSince[i] = t - wallSettle;
						wallSeededUntil[i] = t + wallSeedTime;
					} else {
						wallRate[i] = 0;
						wallSince[i] = t;
					}
				} else {
					float measDt = t - wallTime[i];
					wallDistance[i] += wallFilterA * residual;
					if (measDt > 0) wallRate[i] += wallFilterB * residual / measDt;
				}
				wallTime[i] = t;
			}
		}

		if (!isnan(wallDistance[i]) && t - wallTime[i] > wallLossTimeout) { // lost the wall
			wallDistance[i] = NAN;
			wallRate[i] = 0;
		}
	}

	// Body velocity along an axis is the closing speed of the wall in front of it, or the opening speed of the one behind
	bool turning = fabs(gyro.x) > wallGyroMax || fabs(gyro.y) > wallGyroMax || fabs(gyro.z) > wallGyroMax;
	wallTurning = turning;
	float vel[2];
	for (int axis = 0; axis < 2; axis++) {
		int pos = axis, neg = axis + 2; // sensors looking along + and - of the axis
		bool posOk = !isnan(wallDistance[pos]) && t - wallSince[pos] > wallSettle;
		bool negOk = !isnan(wallDistance[neg]) && t - wallSince[neg] > wallSettle;
		if (turning || (!posOk && !negOk)) vel[axis] = NAN;
		else if (posOk && negOk) vel[axis] = (wallRate[neg] - wallRate[pos]) / 2;
		else vel[axis] = posOk ? -wallRate[pos] : wallRate[neg];
	}
	wallVelX = vel[0];
	wallVelY = vel[1];
	wallVelSigma = wallVelNoiseBase;
	for (int i = 0; i < WALL_COUNT; i++) {
		if (t < wallSeededUntil[i] && !isnan(vel[i % 2])) wallVelSigma = wallSeedSigma;
	}
}

// Wall position hold: with the stick of an axis centered and a wall in view along it, remember the distance to the
// nearest wall and return velocity targets that bring the drone back to it. Unlike the velocity estimate this
// does not drift. `axis` is 0 for forward/back (front and back walls), 1 for left/right; `speed` is the velocity
// along the axis. Returns the velocity target along the body axis (forward, left), NAN when not holding.
float wallHold(int axis, bool centered, float speed) {
	static float target[2] = { NAN, NAN }; // distance to hold
	static int sensor[2] = { -1, -1 }; // wall it is measured to
	static float since[2] = { NAN, NAN }; // when the conditions for capturing it began

	int nearest = -1;
	float best = INFINITY;
	for (int i = axis; i < WALL_COUNT; i += 2) { // the two sensors of this axis
		if (!isnan(wallDistance[i]) && t - wallSince[i] > wallSettle && wallDistance[i] < best) {
			best = wallDistance[i];
			nearest = i;
		}
	}
	bool turning = fabs(gyro.x) > wallGyroMax || fabs(gyro.y) > wallGyroMax || fabs(gyro.z) > wallGyroMax;
	if (wallHoldP <= 0 || !centered || nearest < 0 || turning) {
		target[axis] = NAN;
		sensor[axis] = -1;
		return NAN;
	}
	if (nearest != sensor[axis]) { // a different wall: start over
		sensor[axis] = nearest;
		target[axis] = NAN;
		since[axis] = t;
	}
	if (isnan(target[axis])) {
		if (t - since[axis] < wallHoldSettle || fabs(speed) > wallHoldSpeedMax) return NAN;
		target[axis] = wallDistance[nearest]; // capture
	}
	float error = wallDistance[nearest] - target[axis];
	// further from a wall on the positive side of the axis means moving towards it, and the opposite for the other side
	float velocity = wallHoldP * error * (nearest < 2 ? 1 : -1);
	return constrain(velocity, -wallHoldMax, wallHoldMax);
}
