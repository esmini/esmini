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

#ifndef OSGGA_RUBBERBANDMANIPULATOR
#define OSGGA_RUBBERBANDMANIPULATOR 1

#include <osgGA/CameraManipulator>
#include <osg/PositionAttitudeTransform>

#include <osg/Quat>
#include <osg/observer_ptr>

namespace osgGA{

class RubberbandManipulator : public osgGA::CameraManipulator
{
	public:

		enum CAMERA_MODE
		{
			RB_MODE_ORBIT,
			RB_MODE_FIXED,
			RB_MODE_RUBBER_BAND,
			RB_MODE_RUBBER_BAND_ORBIT,
			RB_MODE_TOP,
            RB_MODE_DRIVER,
			RB_NUM_MODES
		};

		RubberbandManipulator(unsigned int mode = RB_MODE_RUBBER_BAND_ORBIT);

        virtual const char* className() const { return "RubberbandManipulator"; }

        typedef std::vector< osg::observer_ptr<osg::Node> >   ObserverNodePath;

		void setTrackNode(osg::PositionAttitudeTransform *node, bool calcDistance = false);

        /** set the position of the matrix manipulator using a 4x4 Matrix.*/
        virtual void setByMatrix(const osg::Matrixd& matrix);

        /** set the position of the matrix manipulator using a 4x4 Matrix.*/
        virtual void setByInverseMatrix(const osg::Matrixd& matrix) { setByMatrix(osg::Matrixd::inverse(matrix)); }

        /** get the position of the manipulator as 4x4 Matrix.*/
        virtual osg::Matrixd getMatrix() const;

        /** get the position of the manipulator as a inverse matrix of the manipulator, typically used as a model view matrix.*/
        virtual osg::Matrixd getInverseMatrix() const;

        /** Start/restart the manipulator.*/
        virtual void init(const GUIEventAdapter& ea,GUIActionAdapter& us);

        /** handle events, return true if handled, false otherwise.*/
        virtual bool handle(const GUIEventAdapter& ea,GUIActionAdapter& us);

        /** Get the keyboard and mouse usage of this manipulator.*/
        virtual void getUsage(osg::ApplicationUsage& usage) const;

		void setMode(unsigned int mode);

		int getMode() { return _mode; }

		void computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const;

		void calculateCameraDistance();

    protected:

        virtual ~RubberbandManipulator();

        /** Reset the internal GUIEvent stack.*/
        void flushEventStack();
        /** Add the current mouse GUIEvent to internal stack.*/
        void addEvent(const GUIEventAdapter& ea);

		// Internal event stack comprising last two events.
        osg::ref_ptr<const GUIEventAdapter> _ga_t1;
        osg::ref_ptr<const GUIEventAdapter> _ga_t0;

        /** For the give mouse movement calculate the movement of the camera.
            Return true is camera has moved and a redraw is required.*/
        bool calcMovement(double dt, bool reset);

        osg::ref_ptr<osg::Node> _node;
		osg::PositionAttitudeTransform *_trackNode;

        osg::Vec3d				_eye;
		osg::Matrix				_matrix;
		osg::Vec3 cameraAcc;
		osg::Vec3 cameraVel;

		float _cameraDistance;
		float _cameraAngle;
		float _cameraRotation;

		unsigned int _mode;
};

}

#endif

