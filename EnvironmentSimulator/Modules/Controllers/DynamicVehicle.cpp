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
#include "RoadManager.hpp"

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

	//delete collision shapes
	for (int j = 0; j < collision_shapes_.size(); j++)
	{
		delete collision_shapes_[j];
	}
	collision_shapes_.clear();
}

class odrRaycaster : public btVehicleRaycaster
{
	roadmanager::OpenDrive* odr_;

public:
	odrRaycaster(roadmanager::OpenDrive* odr) : odr_(odr) {}

	void* castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result);
};


void* odrRaycaster::castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result)
{
	if (odr_ == nullptr)
	{
		return 0;
	}

	// find out road z level of query position
	roadmanager::Position pos;
	pos.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
	pos.SetInertiaPos(from[0], from[1], 0.0f);

	if (pos.GetZ() > from[2])
	{
		// hit
		result.m_hitNormalInWorld.setValue(0.0f, 0.0f, 1.0f);
		result.m_hitPointInWorld.setValue(from[0], from[1], static_cast<float>(pos.GetZ()));
		result.m_distFraction = from[2] - static_cast<float>(pos.GetZ());
		return reinterpret_cast<void*>(1);  // Just return anything other than 0. Pointer is not used anyhow.
	}

	return 0;
};

void DynamicVehicle::Init(
	double length,
	double width,
	double height,
	double mass,
	double wheel_diameter,
	double connection_point_z,
	roadmanager::OpenDrive* odr,
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

	height_ = height;
	length_ = length;
	width_ = width;
	connection_point_z_ = connection_point_z;

	// Setup dynamics world
	btConstraintSolver* constraint_solver_ = new btSequentialImpulseConstraintSolver();
	btDefaultCollisionConfiguration* collision_config_ = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher_ = new btCollisionDispatcher(collision_config_);
	btVector3 world_min(-1.0e+3, -1.0e+3, -1.0e+3);
	btVector3 world_max(1.0e+3, 1.0e+3, 1.0e+3);
	btBroadphaseInterface* pair_cache_ = new btAxisSweep3(world_min, world_max);
	dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, pair_cache_, constraint_solver_, collision_config_);
	dynamics_world_->setGravity(btVector3(0, 0, -10));
	dynamics_world_->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
	dynamics_world_->getSolverInfo().m_globalCfm = 0.00001f;


	// Create dynamic chassis object
	btBoxShape* veh_shape_ = new btBoxShape(btVector3(0.5f * static_cast<float>(length), 0.5f * static_cast<float>(width), 0.5f * static_cast<float>(height)));
	collision_shapes_.push_back(veh_shape_);

	// shift the center of mass with respect to the chassis
	btTransform veh_xform_;
	veh_xform_.setIdentity();
	veh_xform_.setOrigin(btVector3(0, 0, 1.0f * static_cast<float>(height)));
	btCompoundShape* compound_ = new btCompoundShape();
	compound_->addChildShape(veh_xform_, veh_shape_);

	// create a rigid body

	btVector3 local_inertia(0, 0, 0);
	veh_shape_->calculateLocalInertia(static_cast<float>(mass), local_inertia);

	//btRigidBody::btRigidBodyConstructionInfo rbInfo(static_cast<float>(mass), 0, compound_, local_inertia);

	btRigidBody* veh_body_ = new btRigidBody(static_cast<float>(mass), 0, compound_, local_inertia);

	//Adds the vehicle chassis to the world
	dynamics_world_->addRigidBody(veh_body_);

#if 0
	odrRaycaster* veh_ray_caster_ = new odrRaycaster(odr);
#else
	btVehicleRaycaster* veh_ray_caster_ = new btDefaultVehicleRaycaster(dynamics_world_);
#endif

	btRaycastVehicle::btVehicleTuning tuning;

	// create a new instance of the raycast vehicle
	vehicle_ = new btRaycastVehicle(tuning, veh_body_, veh_ray_caster_);

	// setup wheels
	vehicle_->setCoordinateSystem(1, 2, 0);  // forward, right, up
	btScalar wheel_radius = static_cast<float>(0.5 * wheel_diameter);
	btScalar wheel_width = 0.3f;
	btVector3 wheel_dir(0, 0, -1);  // downwards
	btVector3 wheel_axle(0, -1, 0);  // the axis which the wheel rotates arround
	btScalar suspension_rest_length((btScalar)0.5);
	bool is_front_wheel = true;
	btVector3 connectionPointCS0;

	// wheel configuration assumes the vehicle is centered at the origin
	connectionPointCS0 = btVector3(0.5f * static_cast<float>(length) - 1.0f, 0.5f * static_cast<float>(width) - wheel_width,
		static_cast<float>(connection_point_z_ + 0.5 * height));

	vehicle_->addWheel(connectionPointCS0, wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	is_front_wheel = false;
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, 1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);

	for (int i = 0; i < vehicle_->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle_->getWheelInfo(i);

		// The stiffness constant for the suspension.  10.0 - Offroad buggy, 50.0 - Sports car, 200.0 - F1 Car
		wheel.m_suspensionStiffness = 12.0f;
		// From Vehicle Simulation With Bullet, Date: 16/8/2010, Kester Maddock
		// https://docs.google.com/document/d/18edpOwtGgCwNyvakS78jxMajCuezotCU_0iezcwiFQc/edit?usp=sharing
		// The damping coefficient for when the suspension is compressed.
		// Set to k * 2.0 * btSqrt(m_suspensionStiffness) so k is proportional to critical damping.
		// k = 0.0 undamped & bouncy, k = 1.0 critical damping
		// k = 0.1 to 0.3 are good values
		double k = 0.1f;
		wheel.m_wheelsDampingCompression = static_cast<btScalar>(k * 2.0f * sqrt(suspension_stiffness));
		// The damping coefficient for when the suspension is expanding.
		// m_wheelsDampingRelaxation should be slightly larger than m_wheelsDampingCompression, eg k = 0.2 to 0.5
		k = 0.2f;
		wheel.m_wheelsDampingCompression = static_cast<btScalar>(k * 2.0f * sqrt(suspension_stiffness));
		wheel.m_frictionSlip = 1000.0f;
		wheel.m_rollInfluence = 0.1f;
	}

	vehicle_->setSteeringValue(0.0, 0);
	vehicle_->setSteeringValue(0.0, 1);

	veh_body_->setCenterOfMassTransform(btTransform::getIdentity());
	veh_body_->setLinearVelocity(btVector3(0, 0, 0));
	veh_body_->setAngularVelocity(btVector3(0, 0, 0));
	vehicle_->resetSuspension();
	for (int i = 0; i < vehicle_->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		vehicle_->updateWheelTransform(i, true);
		vehicle_->applyEngineForce(0.0, i);
		vehicle_->setBrake(0.0, i);
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
#if 0
	float wf = powf(vehicle_->getRigidBody()->getLinearVelocity().length(), 2.0f);
	btVector3 windForce = -5.0f * wf * vehicle_->getRigidBody()->getLinearVelocity();
	vehicle_->getRigidBody()->applyCentralForce(windForce);

	// add forces
	int wheelIndex = 2;
	float engine_force = 0.0;
	float brake_force = 0.0;
	float throttle = 0.0;
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
#endif
	vehicle_->setSteeringValue(0.0, 0);
	vehicle_->setSteeringValue(0.0, 1);
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

int DynamicVehicle::GetWheelInfo(int index, double& h, double& z)
{
	btScalar r[3] = { 0.0f, 0.0f, 0.0f };
	const btTransform* xform = &(vehicle_->getRigidBody()->getCenterOfMassTransform());
	xform->getRotation().getEulerZYX(r[0], r[1], r[2]);

	if (index < vehicle_->getNumWheels())
	{
		btScalar wh, wp, wr;
		xform = &(vehicle_->getWheelTransformWS(index));
		xform->getRotation().getEulerZYX(wh, wp, wr);
		h = GetAngleInInterval2PI(wh - r[0]);

		z = connection_point_z_ - vehicle_->getWheelInfo(index).m_raycastInfo.m_suspensionLength;
		printf("connect point %.2f suspensionRestLength %.2f suspLen %.2f wheel xform z %.2f z %.2f\n",
			connection_point_z_, vehicle_->getWheelInfo(index).getSuspensionRestLength(),
			vehicle_->getWheelInfo(index).m_raycastInfo.m_suspensionLength, xform->getOrigin()[2], z);
		return 0;
	}

	return -1;
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

void DynamicVehicle::SetupFlatGround(double size)
{
	btCollisionShape* ground_shape = new btBoxShape(btVector3(static_cast<float>(size), static_cast<float>(size), 10.0f));


	collision_shapes_.push_back(ground_shape);

	// create ground object, replaces localCreateRigidBody()
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0, -10.0f));

	btVector3 localInertia(0, 0, 0);
	btRigidBody* body = new btRigidBody(0, 0, ground_shape, localInertia);
	body->setWorldTransform(tr);
	dynamics_world_->addRigidBody(body);
}