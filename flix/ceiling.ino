// Upward rangefinder (VL53L1X-like): keeps the drone from climbing into the ceiling

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t;
extern Quaternion attitude;

float ceilingRange = NAN; // latest reading, m (NAN if no ceiling in range)
bool ceilingUpdated = false; // set by the driver when there is a new reading
float ceilingDistance = NAN; // filtered vertical distance to the ceiling, m (NAN if unknown)
float ceilingMargin = 0.4; // m, the drone is not allowed to climb closer to the ceiling than this
float ceilingFilterA = 0.3; // distance correction gain per measurement
float ceilingLossTimeout = 0.3; // s, how long the ceiling stays known without readings

static float ceilingTime = NAN; // time of the last valid reading

void estimateCeiling() {
	if (ceilingUpdated) {
		ceilingUpdated = false;
		if (!isnan(ceilingRange) && ceilingRange > 0.02) {
			Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
			if (up.z > 0.5) { // compensate for tilt
				float distance = ceilingRange * up.z;
				if (isnan(ceilingDistance)) ceilingDistance = distance;
				else ceilingDistance += ceilingFilterA * (distance - ceilingDistance);
				ceilingTime = t;
			}
		}
	}
	if (!isnan(ceilingDistance) && t - ceilingTime > ceilingLossTimeout) ceilingDistance = NAN;
}
