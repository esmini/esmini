/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "DynamicVehicle.hpp"
#include "CommonMini.hpp"

using namespace dynamicvehicle;

DynamicVehicle::~DynamicVehicle()
{
	if (vehicle_ != nullptr)
	{
		delete vehicle_;
		vehicle_ = nullptr;
	}

	if (dynamics_world_ != nullptr)
	{
		delete vehicle_;
		vehicle_ = nullptr;
	}
}

void DynamicVehicle::Init(
	double length,
	double width,
	double height,
	double mass,
	double suspension_stiffness,
	double friction_slip,
	double roll_influence
)
{
	if (vehicle_ != nullptr)
	{
		LOG("DynamicVehicle already initialized - use Reset() to re-initialize");
		return;
	}

	// Setup dynamics world
	btConstraintSolver* constraint_solver_ = new btSequentialImpulseConstraintSolver();
	btDefaultCollisionConfiguration* collision_config_ = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher_ = new btCollisionDispatcher(collision_config_);
	btVector3 world_min(-1.0e+6, -1.0e+6, -1.0e+6);
	btVector3 world_max(1.0e+6, 1.0e+6, 1.0e+6);
	btBroadphaseInterface* pair_cache_ = new btAxisSweep3(world_min, world_max);
	dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, pair_cache_, constraint_solver_, collision_config_);
	dynamics_world_->setGravity(btVector3(0, 0, -10));


	// Create dynamic chassis object
	btBoxShape* veh_shape_ = new btBoxShape(btVector3(0.5f * static_cast<float>(length), 0.5f * static_cast<float>(width), 0.5f * static_cast<float>(height)));

	// shift the center of mass with respect to the chassis
	btTransform veh_xform_;
	veh_xform_.setIdentity();
	veh_xform_.setOrigin(btVector3(0, 0, 1));
	btCompoundShape compound_;
	compound_.addChildShape(veh_xform_, veh_shape_);

	// create a rigid body

	btVector3 local_inertia(0, 0, 0);
	veh_shape_->calculateLocalInertia(static_cast<float>(mass), local_inertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(static_cast<float>(mass), 0, veh_shape_, local_inertia);

	btRigidBody* veh_body_ = new btRigidBody(rbInfo);

	//Adds the vehicle chassis to the world
	dynamics_world_->addRigidBody(veh_body_);

	btVehicleRaycaster* veh_ray_caster_ = new btDefaultVehicleRaycaster(dynamics_world_);

	btRaycastVehicle::btVehicleTuning tuning;

	// create a new instance of the raycast vehicle
	vehicle_ = new btRaycastVehicle(tuning, veh_body_, veh_ray_caster_);

	// setup wheels
	vehicle_->setCoordinateSystem(1, 2, 0);  // forward, right, up
	btScalar wheel_radius = 0.4f;
	btScalar wheel_width = 0.3f;
	btScalar connection_height = 0;
	btVector3 wheel_dir(0, 0, -1);  // downwards
	btVector3 wheel_axle(0, 1, 0);  // the axis which the wheel rotates arround
	btScalar suspension_rest_length((btScalar)0.6);
	bool is_front_wheel = true;
	btVector3 connectionPointCS0;

	// wheel configuration assumes the vehicle is centered at the origin
	connectionPointCS0 = btVector3(0.5f * static_cast<float>(length) - 1.0f, 0.5f * static_cast<float>(width) - wheel_width, connection_height);

	vehicle_->addWheel(connectionPointCS0, wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	is_front_wheel = false;
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, 1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);

	for (int i = 0; i < vehicle_->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle_->getWheelInfo(i);
		wheel.m_suspensionStiffness = static_cast<float>(suspension_stiffness);
		wheel.m_wheelsDampingRelaxation = 0.3f * 2.0f * btSqrt(wheel.m_suspensionStiffness);
		wheel.m_wheelsDampingCompression = 0.5f * 2.0f * btSqrt(wheel.m_suspensionStiffness);
		wheel.m_frictionSlip = static_cast<float>(friction_slip);
		wheel.m_rollInfluence = static_cast<float>(roll_influence);
	}

	// never deactivate the vehicle
	veh_body_->setActivationState(DISABLE_DEACTIVATION);

	//Adds the vehicle to the world
	dynamics_world_->addVehicle(vehicle_);
}

void DynamicVehicle::DrivingControlTarget(double dt, double target_speed, double heading_to_target)
{
}

void DynamicVehicle::DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering)
{

}

void DynamicVehicle::DrivingControlAnalog(double dt, double throttle, double steering)
{

}

void DynamicVehicle::Step(double dt)
{
	// Add wind force
	float wf = powf(vehicle_->getRigidBody()->getLinearVelocity().length(), 2.0f);
	btVector3 windForce = -5.0f * wf * vehicle_->getRigidBody()->getLinearVelocity();
	vehicle_->getRigidBody()->applyCentralForce(windForce);

	// add forces
	int wheelIndex = 2;
	float engine_force = 10.0;
	float brake_force = -1.0;
	float throttle = 1.0;
	vehicle_->applyEngineForce(engine_force, wheelIndex);
	vehicle_->setBrake(brake_force + 5 * (1 - throttle), wheelIndex);
	wheelIndex = 3;
	vehicle_->applyEngineForce(engine_force, wheelIndex);
	vehicle_->setBrake(brake_force + 5 * (1 - throttle), wheelIndex);

#if 0
	wheelIndex = 0;
	m_vehicle->setSteeringValue(steeringClamp * gVehicleSteering, wheelIndex);
	wheelIndex = 1;
	m_vehicle->setSteeringValue(steeringClamp * gVehicleSteering, wheelIndex);
#endif
	dynamics_world_->stepSimulation(static_cast<float>(dt));
}

void DynamicVehicle::GetPosition(double& x, double& y, double& z)
{
	const btTransform* xform = &(vehicle_->getRigidBody()->getCenterOfMassTransform());

	x = xform->getOrigin()[0];
	y = xform->getOrigin()[1];
	z = xform->getOrigin()[2];
}

void DynamicVehicle::GetRotation(double& h, double& p, double& r)
{
	btScalar h_ = 0.0;
	btScalar p_ = 0.0;
	btScalar r_ = 0.0;
	const btTransform* xform = &(vehicle_->getRigidBody()->getCenterOfMassTransform());
	xform->getRotation().getEulerZYX(h_, p_, r_);
	h = h_;
	p = p_;
	r = r_;
}

void DynamicVehicle::Reset(double x, double y, double z, double h, double p, double r, double speed)
{
	vehicle_->getRigidBody()->clearForces();
	vehicle_->getRigidBody()->setLinearVelocity(btVector3(static_cast<float>(speed) * cos(static_cast<float>(h)),
		static_cast<float>(speed) * sin(static_cast<float>(h)), 0));
	vehicle_->getRigidBody()->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
	vehicle_->getRigidBody()->getWorldTransform().setOrigin(btVector3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
	vehicle_->getRigidBody()->getWorldTransform().setRotation(btQuaternion(
		static_cast<float>(h), static_cast<float>(p), static_cast<float>(r)));
}