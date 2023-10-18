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

#define _USE_MATH_DEFINES
#include <math.h>

namespace osgGA
{

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
            RB_MODE_CUSTOM,
            RB_NUM_MODES
        };

        class CustomCamera
        {
        public:
            CustomCamera(osg::Vec3 pos, osg::Vec3 rot, bool fixed_pos) : pos_(pos), rot_(rot), fixed_pos_(fixed_pos), fixed_rot_(true), ortho_(false)
            {
            }
            CustomCamera(osg::Vec3 pos, bool fixed_pos) : pos_(pos), fixed_pos_(fixed_pos), fixed_rot_(false), ortho_(false)
            {
            }
            CustomCamera(osg::Vec3 pos, double rot)
                : pos_(pos),
                  rot_(osg::Vec3(static_cast<float>(rot), static_cast<float>(M_PI_2) - 1e-5f, 0.0f)),
                  fixed_pos_(true),
                  fixed_rot_(true),
                  ortho_(true)
            {
            }

            bool GetFixPos()
            {
                return fixed_pos_;
            }
            bool GetFixRot()
            {
                return fixed_rot_;
            }
            bool GetOrtho()
            {
                return ortho_;
            }
            osg::Vec3 GetPos()
            {
                return pos_;
            }
            osg::Vec3 GetRot()
            {
                return rot_;
            }

        private:
            osg::Vec3 pos_;
            osg::Vec3 rot_;
            bool      fixed_pos_;
            bool      fixed_rot_;
            bool      ortho_;
        };

        RubberbandManipulator(unsigned int mode = RB_MODE_RUBBER_BAND_ORBIT);

        virtual const char* className() const
        {
            return "RubberbandManipulator";
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

        void AddCustomCamera(CustomCamera customCamera)
        {
            customCamera_.push_back(customCamera);
        }

        unsigned int GetNumberOfCameraModes()
        {
            return static_cast<unsigned int>(CAMERA_MODE::RB_NUM_MODES + customCamera_.size() - 1);
        }

        CustomCamera* GetCurrentCustomCamera();
        osg::Vec3     getRelativePos() const
        {
            return relative_pos_;
        }

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

        osg::ref_ptr<osg::Node>                      _node;
        osg::ref_ptr<osg::Node>                      track_node_ = nullptr;
        osg::ref_ptr<osg::PositionAttitudeTransform> track_tx_   = nullptr;

        osg::Vec3d  _eye;
        osg::Matrix _matrix;
        osg::Vec3   cameraAcc;
        osg::Vec3   cameraVel;
        osg::Vec3   relative_pos_;

        float _cameraDistance;
        float _cameraAngle;
        float _cameraRotation;

        unsigned int _mode;
        bool         fix_camera_;

        std::vector<CustomCamera> customCamera_;
    };

}  // namespace osgGA

#endif
