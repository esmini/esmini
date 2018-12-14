
#include <windows.h>
#include "RubberbandManipulator.h"
#include <osg/Quat>
#include <osg/Notify>
#include <osg/Transform>
#include <osg/ComputeBoundsVisitor>

#include <iostream>

using namespace osg;
using namespace osgGA;

extern double deltaSimTime;

const float springFC = 14.0f;
const float springDampingRatio = 0.9f;
const float orbitCameraDistance = 16.0f;
const float orbitCameraAngle = 15.0f;

#define MAX(a, b) ((a)>(b) ? (a) : (b))

RubberbandManipulator::RubberbandManipulator(unsigned int mode)
{
	setMode(mode);
	_cameraRotation = 0;
	_cameraAngle = orbitCameraAngle;
	_cameraDistance = orbitCameraDistance;
}

RubberbandManipulator::~RubberbandManipulator()
{
}

void RubberbandManipulator::setMode(unsigned int mode)
{
	_mode = mode;

	if(mode == RB_MODE_RUBBER_BAND)
	{
		_cameraAngle = orbitCameraAngle;
		_cameraDistance = orbitCameraDistance;
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

void RubberbandManipulator::init(const GUIEventAdapter& ,GUIActionAdapter& us)
{
	calcMovement(true);
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

		case(GUIEventAdapter::FRAME):
            addEvent(ea);
			if (calcMovement(false)) us.requestRedraw();
            return false;

		default:
            return false;
    }
	return false;
}

void RubberbandManipulator::computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const
{
	osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*> (_trackNode);
	nodeCenter = pat->getPosition();
	nodeCenter[2]+=1;
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

bool RubberbandManipulator::calcMovement(bool reset)
{
	osg::Vec3 up(0.0, 0.0, 1.0);
	osg::Vec3d nodeCenter;
	osg::Quat nodeRotation;
	osg::Matrix cameraTargetRotation;
	float springDC;
	double dt;
	osg::Vec3 cameraOffset(0, 0, 0);
	osg::Vec3 cameraTargetPosition(0, 0, 0);
	osg::Vec3 cameraToTarget(0, 0, 0);


	computeNodeCenterAndRotation(nodeCenter, nodeRotation);

	if(_mode == RB_MODE_RUBBER_BAND)
	{
		cameraOffset.set(-_cameraDistance, 0.0, _cameraDistance * atan(_cameraAngle*0.0174533));
	}
	else
	{
		float x, y, z;

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

	dt = deltaSimTime;
	if(reset)
	{
		_eye = nodeCenter + cameraTargetPosition;
		cameraVel.set(0,0,0);
		cameraAcc.set(0,0,0);
	}
	else
	{
		if (dt < 0)
		{
			notify(INFO) << "warning dt = "<< dt << "\n";
			dt = 0.0f;
		}

		// Find the vector between target position and actual camera position
		cameraToTarget = (nodeCenter + cameraTargetPosition) - _eye;
		// Update camera state
		springDC = 2 * springDampingRatio * sqrt(springFC);
		cameraAcc = cameraToTarget*springFC - cameraVel*springDC;
		cameraVel+=cameraAcc * dt;
		
		if(_mode == RB_MODE_FIXED || _mode == RB_MODE_ORBIT)
		{
			_eye = nodeCenter + cameraTargetPosition;
		}
		else
		{
			_eye+=cameraVel * dt;
		}
	}

	// Create the view matrix
	_matrix.makeLookAt(_eye, nodeCenter, up);

    return true;
}

