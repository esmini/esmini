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


#include "ControllerDynamics.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"

#include <random>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerDynamics(void* args)
{
	Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

	return new ControllerDynamics(initArgs);
}

ControllerDynamics::ControllerDynamics(InitArgs* args) :
	suspension_stiffness_(50.0), friction_slip_(100.0), roll_influence_(1.0),
	Controller(args)
{
	if (args && args->properties)
	{

	}
}

ControllerDynamics::~ControllerDynamics()
{
	delete veh_shape_;
	delete constraint_solver_;
	delete dynamics_world_;
}

void ControllerDynamics::Init()
{
	printf("dynamics init\n");


	// Setup dynamics world
	constraint_solver_ = new btSequentialImpulseConstraintSolver();
	collision_config_ = new btDefaultCollisionConfiguration();
	dispatcher_ = new btCollisionDispatcher(collision_config_);
	btVector3 world_min(-1.0e+6, -1.0e+6, -1.0e+6);
	btVector3 world_max(1.0e+6, 1.0e+6, 1.0e+6);
	pair_cache_ = new btAxisSweep3(world_min, world_max);
	dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, pair_cache_, constraint_solver_, collision_config_);
	dynamics_world_->setGravity(btVector3(0, 0, -10));


	// Create dynamic chassis object
	double half_length = 0.5 * object_->boundingbox_.dimensions_.length_;
	double half_width = 0.5 * object_->boundingbox_.dimensions_.width_;
	double half_height = 0.5 * object_->boundingbox_.dimensions_.height_;

	veh_shape_ = new btBoxShape(btVector3(half_length, half_width, half_height));

	// shift the center of mass with respect to the chassis
	veh_xform_.setIdentity();
	veh_xform_.setOrigin(btVector3(0, 0, 1));
	compound_.addChildShape(veh_xform_, veh_shape_);

	// create a rigid body

	// chassis mass
	btScalar mass(1200);

	btVector3 local_inertia(0, 0, 0);
	veh_shape_->calculateLocalInertia(mass, local_inertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, 0, veh_shape_, local_inertia);


	veh_body_ = new btRigidBody(rbInfo);
	veh_body_->getWorldTransform().setOrigin(btVector3(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ()));
	veh_body_->getWorldTransform().setRotation(btQuaternion(object_->pos_.GetH(), object_->pos_.GetP(), object_->pos_.GetR()));

	//Adds the vehicle chassis to the world
	dynamics_world_->addRigidBody(veh_body_);

	veh_ray_caster_ = new btDefaultVehicleRaycaster(dynamics_world_);

	btRaycastVehicle::btVehicleTuning tuning;

	// create a new instance of the raycast vehicle
	vehicle_ = new btRaycastVehicle(tuning, veh_body_, veh_ray_caster_);

	// setup wheels
	vehicle_->setCoordinateSystem(1, 2, 0);  // forward, right, up
	double wheel_radius = 0.4;
	double wheel_width = 0.3;
	double connection_height = 0;
	btVector3 wheel_dir(0, 0, -1);  // downwards
	btVector3 wheel_axle(0, 1, 0);  // the axis which the wheel rotates arround
	btScalar suspension_rest_length((btScalar)0.6);
	bool is_front_wheel = true;
	btVector3 connectionPointCS0;

	// wheel configuration assumes the vehicle is centered at the origin
	connectionPointCS0 = btVector3(half_length - 1.0, half_width - wheel_width, connection_height);

	vehicle_->addWheel(connectionPointCS0, wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	is_front_wheel = false;
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, -1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);
	vehicle_->addWheel(connectionPointCS0 * btVector3(-1, 1, 1), wheel_dir, wheel_axle, suspension_rest_length, wheel_radius, tuning, is_front_wheel);

	for (int i = 0; i < vehicle_->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle_->getWheelInfo(i);
		wheel.m_suspensionStiffness = suspension_stiffness_;
		wheel.m_wheelsDampingRelaxation = 0.3 * 2 * btSqrt(wheel.m_suspensionStiffness);
		wheel.m_wheelsDampingCompression = 0.5 * 2 * btSqrt(wheel.m_suspensionStiffness);
		wheel.m_frictionSlip = friction_slip_;
		wheel.m_rollInfluence = roll_influence_;
	}

	// never deactivate the vehicle
	veh_body_->setActivationState(DISABLE_DEACTIVATION);

	//Adds the vehicle to the world
	dynamics_world_->addVehicle(vehicle_);

	// Detach object from any constraints aligning to road
	object_->pos_.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_NONE);

	Controller::Init();
}

void ControllerDynamics::Step(double timeStep)
{
	printf("dynamics step\n");

	// Add wind force
	double wf = pow(veh_body_->getLinearVelocity().length(), 2);
	btVector3 windForce = -5.0 * wf * veh_body_->getLinearVelocity();
	veh_body_->applyCentralForce(windForce);

	// add forces
	int wheelIndex = 2;
	double engine_force = 10.0;
	double brake_force = -1.0;
	double throttle = 1.0;
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
	dynamics_world_->stepSimulation(timeStep);

	const btTransform* xform = &(vehicle_->getRigidBody()->getCenterOfMassTransform());
	btScalar h = 0.0;
	btScalar p = 0.0;
	btScalar r = 0.0;
	xform->getRotation().getEulerZYX(h, p, r);
	printf("z %.2f h %.2f\n", xform->getOrigin()[2], h);
	gateway_->updateObjectWorldPos(object_->GetId(), 0.0,
		xform->getOrigin()[0], xform->getOrigin()[1], xform->getOrigin()[2],
		h, p, r
	);

	Controller::Step(timeStep);
}

void ControllerDynamics::Activate(ControlDomains domainMask)
{
	printf("dynamics activate\n");

	veh_body_->clearForces();
	veh_body_->setLinearVelocity(btVector3(0, 0, 0));
	veh_body_->setAngularVelocity(btVector3(0, 0, 0));
	veh_body_->getWorldTransform().setOrigin(btVector3(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ()));
	veh_body_->getWorldTransform().setRotation(btQuaternion(object_->pos_.GetH(), object_->pos_.GetP(), object_->pos_.GetR()));

	Controller::Activate(domainMask);
}

void ControllerDynamics::ReportKeyEvent(int key, bool down)
{

}
