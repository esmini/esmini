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

#include "RubberbandManipulator.hpp"
#include <osg/Quat>
#include <osg/Notify>
#include <osg/Transform>
#include <osg/ComputeBoundsVisitor>

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>


using namespace osg;
using namespace osgGA;

#define NODE_CENTER_OFFSET_X 0.0
#define NODE_CENTER_OFFSET_Y 0.0
#define NODE_CENTER_OFFSET_Z 1.0

#define DRIVER_CENTER_OFFSET_X 1.32
#define DRIVER_CENTER_OFFSET_Y 0.0
#define DRIVER_CENTER_OFFSET_Z 1.42

const float springFC = 16.0f;
const float orbitCameraDistance = 25.0f;
const float topCameraDistance = 2300;
const float orbitCameraAngle = 13.0f;
const float orbitCameraRotation = -14.0f;

#define MAX(a, b) ((a)>(b) ? (a) : (b))

RubberbandManipulator::RubberbandManipulator(unsigned int mode)
{
	_cameraAngle = orbitCameraAngle;
	_cameraDistance = orbitCameraDistance;
	_cameraRotation = orbitCameraRotation;
	setMode(mode);
}

RubberbandManipulator::~RubberbandManipulator()
{
}

void RubberbandManipulator::setMode(unsigned int mode)
{
	// If leaving top view, then reset camera rotations
	if (_mode == RB_MODE_TOP || _mode == RB_MODE_DRIVER)
	{
		_cameraAngle = orbitCameraAngle;
		_cameraRotation = orbitCameraRotation;
		_cameraDistance = orbitCameraDistance;
	}

	_mode = mode;

	if (mode == RB_MODE_RUBBER_BAND)
	{
		_cameraAngle = orbitCameraAngle;
		_cameraDistance = orbitCameraDistance;
	}
	else if (mode == RB_MODE_TOP)
	{
		_cameraDistance = topCameraDistance;
	}
}

void RubberbandManipulator::setTrackNode(osg::PositionAttitudeTransform *node, bool calcDistance)
{
    if (!node)
    {
        osg::notify(osg::NOTICE)<<"RubberbandManipulator::setTrackNode(Node*):  Unable to set tracked node due to null Node*"<<std::endl;
        return;
    }
	_trackNode = node;

	if (calcDistance)
	{
		calculateCameraDistance();
	}
}

void RubberbandManipulator::calculateCameraDistance()
{
	const osg::MatrixList& m = _trackNode->getWorldMatrices();
	osg::ComputeBoundsVisitor cbv;
	_trackNode->accept(cbv);
	osg::BoundingBox bb = cbv.getBoundingBox();
	osg::Vec3 minV = bb._min * m.front();
	osg::Vec3 maxV = bb._max * m.front();
	_cameraDistance = MAX((maxV.x() - minV.x() + maxV.y() - minV.y())/2, orbitCameraDistance);
}

void RubberbandManipulator::init(const GUIEventAdapter&, GUIActionAdapter& us)
{
	calcMovement(0, true);
}

void RubberbandManipulator::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("NodeTracker: Space","Reset the viewing position to home");
    usage.addKeyboardMouseBinding("NodeTracker: +","Increase camera distance");
    usage.addKeyboardMouseBinding("NodeTracker: -","Decrease camera distance");
}

bool RubberbandManipulator::handle(const GUIEventAdapter& ea,GUIActionAdapter& us)
{
	static int lpush = 0;
	static int rpush = 0;
	static float lx0 = 0;
	static float ly0 = 0;
	static float rx0 = 0;
	static float ry0 = 0;
	float angleScale = 30.0;
	float zoomScale = 1.0;
	float scrollScale = 0.2;

	if(ea.getEventType() & GUIEventAdapter::PUSH)
	{
		if (ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON)
		{
			lpush = true;
			lx0 = ea.getXnormalized();
			ly0 = ea.getYnormalized();
		}
		else if(ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		{
			rpush = true;
			rx0 = ea.getXnormalized();
			ry0 = ea.getYnormalized();
		}
	}

	if(ea.getEventType() & GUIEventAdapter::RELEASE)
	{
		if (ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON)
		{
			lpush = false;
		}
		else if(ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		{
			rpush = false;
		}
	}

	switch(ea.getEventType())
    {
		case(GUIEventAdapter::DRAG):
			if (ea.getButtonMask() == GUIEventAdapter::LEFT_MOUSE_BUTTON)
			{
				_cameraAngle += angleScale * (ly0 - ea.getYnormalized());
				if(_cameraAngle > 89)
				{
					_cameraAngle = 89;
				}
				if(_cameraAngle < -89)
				{
					_cameraAngle = -89;
				}
				_cameraRotation += 2 * angleScale * (lx0 - ea.getXnormalized());

				lx0 = ea.getXnormalized();
				ly0 = ea.getYnormalized();
			}
			if (ea.getButtonMask() == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
			{
				_cameraDistance -= zoomScale * _cameraDistance * (ry0 - ea.getYnormalized());
				if(_cameraDistance < 1)
				{
					_cameraDistance = 1;
				}
				ry0 = ea.getYnormalized();
			}
			break;
            return false;

		case(GUIEventAdapter::SCROLL):
		{
			static int scroll = 0;
			switch (ea.getScrollingMotion())
			{
			case(osgGA::GUIEventAdapter::SCROLL_DOWN):
				scroll = -1;
				break;
			case(osgGA::GUIEventAdapter::SCROLL_UP):
				scroll = 1;
				break;
			}
			_cameraDistance -= scrollScale * _cameraDistance * (scroll);
			return false;
		}
		case(GUIEventAdapter::FRAME):
		{
			static double old_frametime = 0;
#if 1
			double current_frame_time = ea.getTime();
			double dt = current_frame_time - old_frametime;
			old_frametime = current_frame_time;

#else
			double current_frame_time =  elapsedTime();
#endif

			if (dt > 1)
			{
				dt = 0.1;
			}

			addEvent(ea);
			if (calcMovement(dt, false)) us.requestRedraw();
			return false;
		}

		default:
            return false;
    }
	return false;
}

void RubberbandManipulator::computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const
{
	osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*> (_trackNode);
	nodeCenter = pat->getPosition();
	nodeRotation = pat->getAttitude();
}

void RubberbandManipulator::flushEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
}


void RubberbandManipulator::addEvent(const GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
}

void RubberbandManipulator::setByMatrix(const osg::Matrixd& matrix)
{
}

osg::Matrixd RubberbandManipulator::getMatrix() const
{
	return osg::Matrixd::inverse(_matrix);
}

osg::Matrixd RubberbandManipulator::getInverseMatrix() const
{
	return _matrix;
}

bool RubberbandManipulator::calcMovement(double dt, bool reset)
{
	osg::Vec3 up(0.0, 0.0, 1.0);
	osg::Vec3d nodeCenter;
	osg::Quat nodeRotation;
	osg::Matrix cameraTargetRotation;
	float springDC;
	osg::Vec3 cameraOffset(0, 0, 0);
	osg::Vec3 cameraTargetPosition(0, 0, 0);
	osg::Vec3 cameraToTarget(0, 0, 0);
	float x, y, z;

	computeNodeCenterAndRotation(nodeCenter, nodeRotation);
	osg::Matrix nodeRot;
	nodeRot.makeRotate(nodeRotation);
	osg::Vec3 nodeFocusPoint = nodeRot * osg::Vec3(NODE_CENTER_OFFSET_X, NODE_CENTER_OFFSET_Y, NODE_CENTER_OFFSET_Z) + nodeCenter;

	if (_mode == RB_MODE_TOP)
	{
		_cameraRotation = -90;
		_cameraAngle = 90;
		x = -_cameraDistance * (cosf(_cameraRotation*0.0174533) * cosf(_cameraAngle*0.0174533));
		y = -_cameraDistance * (sinf(_cameraRotation*0.0174533) * cosf(_cameraAngle*0.0174533));
		cameraOffset.set(x, y, _cameraDistance);  // Put a small number to prevent undefined camera angle
	}
	else if(_mode == RB_MODE_RUBBER_BAND)
	{
		cameraOffset.set(-_cameraDistance, 0.0, _cameraDistance * atan(_cameraAngle*0.0174533));
	}
	else if (_mode == RB_MODE_DRIVER)
	{
		_cameraRotation = 0;
		_cameraAngle = 0;
		cameraOffset.set(1.0, 0.0, 0.0);
	}
	else
	{
		if(_mode == RB_MODE_FIXED)
		{
			_cameraRotation = 0;
		}
		x = -_cameraDistance * (cosf(_cameraRotation*0.0174533) * cosf(_cameraAngle*0.0174533));
		y = -_cameraDistance * (sinf(_cameraRotation*0.0174533) * cosf(_cameraAngle*0.0174533));
		z = _cameraDistance * sinf(_cameraAngle*0.0174533);

		cameraOffset.set(x, y, z);
	}

	// Transform the camera target offset position
	cameraTargetRotation.setRotate(nodeRotation);

	cameraTargetPosition = cameraTargetRotation.preMult(cameraOffset);

	if(reset)
	{
		_eye = nodeFocusPoint + cameraTargetPosition;
		cameraVel.set(0,0,0);
		cameraAcc.set(0,0,0);
	}
	else
	{
		if (dt < 0)
		{
			dt = 0.0f;
		}
		if (dt > 0.1)
		{
			dt = 0.1f;
		}

		// Find the vector between target position and actual camera position
		cameraToTarget = (nodeFocusPoint + cameraTargetPosition) - _eye;

		// Update camera state
		springDC = 2 * sqrt(springFC);
		cameraAcc = cameraToTarget * springFC - cameraVel * springDC;
		cameraVel += cameraAcc * dt;

		if (_mode == RB_MODE_FIXED || _mode == RB_MODE_ORBIT || _mode == RB_MODE_TOP  || _mode == RB_MODE_DRIVER)
		{
			_eye = nodeFocusPoint + cameraTargetPosition;
		}
		else
		{
			_eye += cameraVel * dt;
		}
	}

	// Create the view matrix
	if (_mode == RB_MODE_DRIVER)
	{
		// Create a view matrix for driver position, or center Front Looking Camrera (FLC)
		osg::Matrix localTx (
				0.0, 0.0, -1.0, 0.0,
				-1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				DRIVER_CENTER_OFFSET_Y, -DRIVER_CENTER_OFFSET_Z, DRIVER_CENTER_OFFSET_X, 1.0
			);

		// Camera transform is the inverse of focus object rotation and position
		_matrix.makeRotate(nodeRotation.inverse());
		osg::Matrix trans;
		trans.makeTranslate(-nodeCenter);
		_matrix.preMult(trans);

		// Combine driver and camera transform
		_matrix = _matrix * localTx;
	}
	else if (_mode == RB_MODE_TOP)
	{
		_matrix.makeLookAt(_eye, nodeFocusPoint, osg::Vec3(nodeRotation*osg::Vec3(0, -1, 0)));
	}
	else
	{
		_matrix.makeLookAt(_eye, nodeFocusPoint, up);
	}

    return true;
}

