// Horizontal rangefinders (VL53L1X-like) looking at walls: wall distance and velocity along the body axes,
// an independent cross-check for the optical flow velocity

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t, dt;
extern Vector gyro;
extern Quaternion attitude;

const int WALL_COUNT = 4; // front, left, back, right
const Vector wallDirection[WALL_COUNT] = { Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0) }; // body axes

float wallRange[WALL_COUNT] = { NAN, NAN, NAN, NAN }; // latest readings, m (NAN if no wall in range)
bool wallUpdated[WALL_COUNT] = { false, false, false, false }; // set by the driver when there is a new reading
float wallDistance[WALL_COUNT] = { NAN, NAN, NAN, NAN }; // filtered horizontal distance to the wall, m
float wallRate[WALL_COUNT] = { 0, 0, 0, 0 }; // d(distance)/dt, m/s
bool wallVelUpdated = false; // set when there is a new wall velocity
float wallLag = 0.25; // s, delay of the wall velocity (the rate filter)
float wallVelX = NAN, wallVelY = NAN; // velocity in the body frame (forward, left) from the wall distance change, NAN if unknown

float wallFilterA = 0.3; // distance correction gain per measurement
float wallFilterB = 0.02; // rate correction gain per measurement
float wallJump = 0.5; // m, a reading this far from the filter means a new surface: start over
float wallSettle = 0.6; // s, the rate is used this long after the wall is acquired
float wallLossTimeout = 0.3; // s, how long to coast without valid readings
float wallGyroMax = 1.5; // rad/s, the rate is not used while turning fast (the sensors sweep along the wall)

static float wallTime[WALL_COUNT]; // time of the last valid reading
static float wallSince[WALL_COUNT]; // time the wall was acquired

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
				if (isnan(wallDistance[i]) || fabs(residual) > wallJump) {
					wallDistance[i] = distance;
					wallRate[i] = 0;
					wallSince[i] = t;
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
}
