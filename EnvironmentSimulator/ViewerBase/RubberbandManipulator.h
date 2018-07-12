/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
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

		enum 
		{
			RB_MODE_RUBBER_BAND, 
			RB_MODE_ORBIT,
			RB_MODE_FIXED,
			RB_NUM_MODES
		};

		RubberbandManipulator(unsigned int mode = RB_MODE_RUBBER_BAND);

        virtual const char* className() const { return "RubberbandManipulator"; }

        typedef std::vector< osg::observer_ptr<osg::Node> >   ObserverNodePath;

		void RubberbandManipulator::setTrackNode(osg::PositionAttitudeTransform *node, bool calcDistance = false);

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
        bool calcMovement(bool reset);

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

