// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Gazebo plugin for running Arduino code and simulating the drone

#include <functional>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <iostream>
#include <fstream>

#include "Arduino.h"
#include "flix.h"

#include "altitude.ino"
#include "flow.ino"
#include "walls.ino"
#include "console.ino"
#include "control.ino"
#include "estimate.ino"
#include "safety.ino"
#include "log.ino"
#include "filter.h"
#include "mavlink.ino"
#include "motors.ino"
#include "parameters.ino"
#include "power.ino"
#include "rc.ino"
#include "time.ino"

using ignition::math::Vector3d;

// Floor surfaces as seen by the optical flow sensor (the visuals are in flix.world)
struct Surface {
	const char *name;
	double x0, x1, y0, y1; // area, m
	int quality; // surface quality the sensor reports at low height, 0-255
	double noise; // flow noise multiplier
	double gain; // measured flow / true flow
};
const Surface defaultSurface = { "textured", 0, 0, 0, 0, 200, 1, 1 };
const Surface surfaces[] = {
	{ "blank", 2, 4, -2, 2, 0, 1, 0 }, // no features: the sensor reports no motion and a quality of zero
	{ "dim", 5, 7, -2, 2, 35, 6, 1 }, // too dark: low quality, very noisy
	{ "stripes", 8, 10, -2, 2, 180, 1, 0.5 }, // repeating pattern: good quality but the wrong speed
};
using namespace gazebo;
using namespace std;

class ModelFlix : public ModelPlugin {
private:
	physics::ModelPtr model;
	physics::LinkPtr body;
	sensors::ImuSensorPtr imu;
	sensors::RaySensorPtr rangefinder;
	common::Time rangefinderTime;
	sensors::RaySensorPtr walls[WALL_COUNT];
	common::Time wallTimes[WALL_COUNT];
	double flowTime = 0;
	event::ConnectionPtr updateConnection, resetConnection;
	transport::NodePtr nodeHandle;
	transport::PublisherPtr motorPub[4];
	LowPassFilter<Vector> accFilter = LowPassFilter<Vector>(0.1);

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		this->model = _parent;
		this->body = this->model->GetLink("body");
		this->imu = dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor(model->GetScopedName(true) + "::body::imu")); // default::flix::body::imu
		this->rangefinder = dynamic_pointer_cast<sensors::RaySensor>(sensors::get_sensor(model->GetScopedName(true) + "::body::rangefinder")); // default::flix::body::rangefinder
		const char *wallNames[WALL_COUNT] = { "wall_front", "wall_left", "wall_back", "wall_right" };
		for (int i = 0; i < WALL_COUNT; i++) this->walls[i] = dynamic_pointer_cast<sensors::RaySensor>(sensors::get_sensor(model->GetScopedName(true) + "::body::" + wallNames[i]));
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelFlix::OnUpdate, this));
		this->resetConnection = event::Events::ConnectWorldReset(std::bind(&ModelFlix::OnReset, this));
		initNode();
		Serial.begin(0);
		setupParameters();
		rcRxPin = 1; // set rc pin to enable rc reading
		gzmsg << "Flix plugin loaded" << endl;
	}

	void OnReset() {
		attitude = Quaternion(); // reset estimated attitude
		armed = false;
		__resetTime += __micros;
		gzmsg << "Flix plugin reset" << endl;
	}

	void OnUpdate() {
		__micros = model->GetWorld()->SimTime().Double() * 1000000;
		step();

		// read virtual imu
		gyro = Vector(imu->AngularVelocity().X(), imu->AngularVelocity().Y(), imu->AngularVelocity().Z());
		acc = this->accFilter.update(Vector(imu->LinearAcceleration().X(), imu->LinearAcceleration().Y(), imu->LinearAcceleration().Z()));

		readRangefinder();
		readWalls();
		readFlow();

		voltage = 4.2f; // dummy voltage value

		readRC();
		estimate();

		// correct yaw to the actual yaw
		attitude.setYaw(this->model->WorldPose().Yaw());

		control();
		handleConsole();
		processMavlink();

		applyMotorForces();
		publishTopics();
		logData();
		syncParameters();
	}

	void readRangefinder() {
		common::Time measured = rangefinder->LastMeasurementTime();
		if (measured == rangefinderTime) return; // no new reading
		rangefinderTime = measured;
		double range = rangefinder->Range(0);
		bool hit = std::isfinite(range) && range < rangefinder->RangeMax();
		if (!hit && model->WorldPose().Z() < 0.05) { range = 0; hit = true; } // the ray starts inside the ground when landed
		rangefinderRange = hit ? range : NAN;
		rangefinderUpdated = true;
	}

	void readWalls() {
		for (int i = 0; i < WALL_COUNT; i++) {
			common::Time measured = walls[i]->LastMeasurementTime();
			if (measured == wallTimes[i]) continue; // no new reading
			wallTimes[i] = measured;
			double range = walls[i]->Range(0);
			bool hit = std::isfinite(range) && range < walls[i]->RangeMax();
			wallRange[i] = hit ? range : NAN;
			wallUpdated[i] = true;
		}
	}

	// Downward optical flow sensor (PMW3901-like): 100 Hz, angular rate of the ground in the body axes
	void readFlow() {
		double now = model->GetWorld()->SimTime().Double();
		if (now - flowTime < 0.01) return;
		flowTime = now;

		Vector3d pos = model->WorldPose().Pos();
		Vector3d v = body->RelativeLinearVel();
		Vector3d w = imu->AngularVelocity();
		double h = std::isnan(rangefinderRange) ? 0 : rangefinderRange;

		Surface s = defaultSurface;
		for (const Surface& area : surfaces) {
			if (pos.X() >= area.x0 && pos.X() <= area.x1 && pos.Y() >= area.y0 && pos.Y() <= area.y1) s = area;
		}

		double q = s.quality;
		if (h < 0.08) q = 0; // too close, out of focus
		else if (h > 1) q *= std::max(0.2, 1 - (h - 1) / 4); // less detail from higher up

		double noise = 0.02 * s.noise; // rad/s
		double fx = (-v.X() / std::max(h, 0.05) + w.Y()) * s.gain;
		double fy = (-v.Y() / std::max(h, 0.05) - w.X()) * s.gain;
		if (s.gain == 0) fx = fy = 0;
		flowX = fx + ignition::math::Rand::DblNormal(0, noise);
		flowY = fy + ignition::math::Rand::DblNormal(0, noise);
		flowQuality = (int)q;
		flowUpdated = true;
	}

	void applyMotorForces() {
		// thrusts
		const double dist = 0.035355; // motors shift from the center, m
		const double maxThrust = 0.03 * ONE_G; // ~30 g, https://youtu.be/VtKI4Pjx8Sk?&t=78

		const float scale0 = 1.0, scale1 = 1.1, scale2 = 0.9, scale3 = 1.05; // imitating motors asymmetry
		float mfl = scale0 * maxThrust * motors[MOT_FL];
		float mfr = scale1 * maxThrust * motors[MOT_FR];
		float mrl = scale2 * maxThrust * motors[MOT_RL];
		float mrr = scale3 * maxThrust * motors[MOT_RR];

		body->AddLinkForce(Vector3d(0.0, 0.0, mfl), Vector3d(dist, dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mfr), Vector3d(dist, -dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mrl), Vector3d(-dist, dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mrr), Vector3d(-dist, -dist, 0.0));

		// torque
		const double maxTorque = 0.0024 * ONE_G; // ~24 g*cm
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale0 * maxTorque * motors[MOT_FL]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale1 * -maxTorque * motors[MOT_FR]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale2 * -maxTorque * motors[MOT_RL]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale3 * maxTorque * motors[MOT_RR]));
	}

	void initNode() {
		nodeHandle = transport::NodePtr(new transport::Node());
		nodeHandle->Init();
		string ns = "~/" + model->GetName();
		// create motors output topics for debugging and plotting
		motorPub[0] = nodeHandle->Advertise<msgs::Int>(ns + "/motor0");
		motorPub[1] = nodeHandle->Advertise<msgs::Int>(ns + "/motor1");
		motorPub[2] = nodeHandle->Advertise<msgs::Int>(ns + "/motor2");
		motorPub[3] = nodeHandle->Advertise<msgs::Int>(ns + "/motor3");
	}

	void publishTopics() {
		for (int i = 0; i < 4; i++) {
			msgs::Int msg;
			msg.set_data(static_cast<int>(round(motors[i] * 1000)));
			motorPub[i]->Publish(msg);
		}
	}
};

GZ_REGISTER_MODEL_PLUGIN(ModelFlix)
