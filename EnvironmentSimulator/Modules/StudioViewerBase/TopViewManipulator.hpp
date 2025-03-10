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

#pragma once

#include <osg/BoundingSphere>
#include <osgGA/CameraManipulator>
#include <osg/PositionAttitudeTransform>

#include <osg/Quat>
#include <osg/observer_ptr>

#define _USE_MATH_DEFINES
#include <math.h>

namespace osgGA
{

    class TopViewManipulator : public osgGA::CameraManipulator
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
            RB_MODE_CUSTOM,
            RB_NUM_MODES
        };

        TopViewManipulator(osg::Vec3d origin = {0.0, 0.0, 0.0});

        virtual const char* className() const
        {
            return "TopViewManipulator";
        }

        typedef std::vector<osg::observer_ptr<osg::Node> > ObserverNodePath;

        void setTrackNode(osg::ref_ptr<osg::Node> node, bool calcDistance = false);
        void setTrackTransform(osg::ref_ptr<osg::PositionAttitudeTransform> tx);

        /** set the position of the matrix manipulator using a 4x4 Matrix.*/
        virtual void setByMatrix(const osg::Matrixd& matrix);

        /** set the position of the matrix manipulator using a 4x4 Matrix.*/
        virtual void setByInverseMatrix(const osg::Matrixd& matrix)
        {
            setByMatrix(osg::Matrixd::inverse(matrix));
        }

        /** get the position of the manipulator as 4x4 Matrix.*/
        virtual osg::Matrixd getMatrix() const;

        /** get the position of the manipulator as a inverse matrix of the manipulator, typically used as a model view matrix.*/
        virtual osg::Matrixd getInverseMatrix() const;

        /** Start/restart the manipulator.*/
        virtual void init(const GUIEventAdapter& ea, GUIActionAdapter& us);

        using osgGA::CameraManipulator::handle;
        /** handle events, return true if handled, false otherwise.*/
        bool handle(const GUIEventAdapter& ea, GUIActionAdapter& us) override;

        /** Get the keyboard and mouse usage of this manipulator.*/
        virtual void getUsage(osg::ApplicationUsage& usage) const;

        void setMode(unsigned int mode);

        int getMode()
        {
            return static_cast<int>(_mode);
        }

        void computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const;

        void calculateCameraDistance();

        unsigned int GetNumberOfCameraModes()
        {
            return RB_NUM_MODES;
        }

        osg::Vec3d getRelativePos() const
        {
            return relative_pos_;
        }
        osg::Vec3d origin_;

        const osg::Vec2d& getMousePos() const
        {
            return mousePos_;
        }

        void moveCameraTo(double x, double y);

        void setCameraDistance(double d);

        std::vector<double> getCameraSettings();

        void setCameraSettings(const std::vector<double>& settings);

        void backupCameraSettings();

        void restoreCameraSettings();

        void SetBound(const osg::BoundingSphere& bs);

    protected:
        virtual ~TopViewManipulator();

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

        osg::ref_ptr<osg::Node>                      _node;
        osg::ref_ptr<osg::Node>                      track_node_ = nullptr;
        osg::ref_ptr<osg::PositionAttitudeTransform> track_tx_   = nullptr;

        osg::Vec3d  _eye;
        osg::Matrix _matrix;
        osg::Vec3d  cameraAcc;
        osg::Vec3d  cameraVel;
        osg::Vec3d  relative_pos_;

        osg::Vec3d cameraPos_;
        osg::Vec3d cameraTargetPos_;
        osg::Vec2d mousePos_;

        std::vector<double> cameraSettings_;

        osg::BoundingSphere bs_;

        double _cameraDistance;
        double _cameraAngle;
        double _cameraRotation;

        unsigned int _mode;
    };

}  // namespace osgGA
