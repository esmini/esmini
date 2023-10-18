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
#define NODE_CENTER_OFFSET_Z 0.25

#define DRIVER_CENTER_OFFSET_X 1.32
#define DRIVER_CENTER_OFFSET_Y 0.0
#define DRIVER_CENTER_OFFSET_Z 1.42

const float springFC            = 16.0f;
const float orbitCameraDistance = 25.0f;
const float topCameraDistance   = 2300;
const float orbitCameraAngle    = 13.0f;
const float orbitCameraRotation = -14.0f;

#define MAX(a, b) ((a) > (b) ? (a) : (b))

RubberbandManipulator::RubberbandManipulator(unsigned int mode)
{
    _cameraAngle    = orbitCameraAngle;
    _cameraDistance = orbitCameraDistance;
    _cameraRotation = orbitCameraRotation;
    track_node_     = nullptr;
    track_tx_       = nullptr;
    setMode(mode);
}

RubberbandManipulator::~RubberbandManipulator()
{
}

RubberbandManipulator::CustomCamera* RubberbandManipulator::RubberbandManipulator::GetCurrentCustomCamera()
{
    if (customCamera_.size() == 0)
    {
        return nullptr;
    }

    // Custom camera models starts on index RB_MODE_CUSTOM
    int index = static_cast<int>(_mode) - RB_MODE_CUSTOM;

    // make sure index is withing range
    if (index >= static_cast<int>(customCamera_.size()))
    {
        index = static_cast<int>(customCamera_.size()) - 1;
    }
    else if (index < 0)
    {
        index = 0;
        return 0;
    }

    return &customCamera_[static_cast<unsigned int>(index)];
}

void RubberbandManipulator::setMode(unsigned int mode)
{
    if (mode > GetNumberOfCameraModes())
    {
        mode = GetNumberOfCameraModes() - 1;
    }

    // If leaving locked views, then reset camera rotations
    if (_mode == RB_MODE_TOP || _mode == RB_MODE_DRIVER || _mode >= RB_MODE_CUSTOM)
    {
        _cameraAngle    = orbitCameraAngle;
        _cameraRotation = orbitCameraRotation;
        _cameraDistance = orbitCameraDistance;
    }

    _mode = mode;

    if (mode == RB_MODE_RUBBER_BAND)
    {
        _cameraAngle    = orbitCameraAngle;
        _cameraDistance = orbitCameraDistance;
    }
    else if (mode == RB_MODE_TOP)
    {
        _cameraDistance = topCameraDistance;
    }
    else if (mode >= CAMERA_MODE::RB_MODE_CUSTOM)
    {
    }
}

void RubberbandManipulator::setTrackNode(osg::ref_ptr<osg::Node> node, bool calcDistance)
{
    if (!node)
    {
        osg::notify(osg::NOTICE) << "RubberbandManipulator::setTrackBB(bb):  Unable to set tracked bounding box due to null Node" << std::endl;
        return;
    }
    track_node_ = node;

    if (calcDistance)
    {
        calculateCameraDistance();
    }
}

void RubberbandManipulator::setTrackTransform(osg::ref_ptr<osg::PositionAttitudeTransform> tx)
{
    if (!tx)
    {
        osg::notify(osg::NOTICE) << "RubberbandManipulator::setTrackTX(tx):  Unable to set tracked transofmration node due to null Node" << std::endl;
        return;
    }
    track_tx_ = tx;
}

void RubberbandManipulator::calculateCameraDistance()
{
    const osg::MatrixList&    m = track_node_->getWorldMatrices();
    osg::ComputeBoundsVisitor cbv;
    track_node_->accept(cbv);
    osg::BoundingBox bb   = cbv.getBoundingBox();
    osg::Vec3        minV = bb._min * m.front();
    osg::Vec3        maxV = bb._max * m.front();
    _cameraDistance       = MAX((maxV.x() - minV.x() + maxV.y() - minV.y()) / 2, orbitCameraDistance);
}

void RubberbandManipulator::init(const GUIEventAdapter&, GUIActionAdapter& us)
{
    (void)us;
    calcMovement(0, true);
}

void RubberbandManipulator::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("NodeTracker: Space", "Reset the viewing position to home");
    usage.addKeyboardMouseBinding("NodeTracker: +", "Increase camera distance");
    usage.addKeyboardMouseBinding("NodeTracker: -", "Decrease camera distance");
}

bool RubberbandManipulator::handle(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
    static float lx0         = 0;
    static float ly0         = 0;
    static float ry0         = 0;
    float        angleScale  = 30.0;
    float        zoomScale   = 1.0;
    float        scrollScale = 0.2f;

    if (ea.getEventType() & GUIEventAdapter::PUSH)
    {
        if (ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON)
        {
            // int lpush = true;
            lx0 = ea.getXnormalized();
            ly0 = ea.getYnormalized();
        }
        else if (ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            // int rpush = true;
            // float rx0 = ea.getXnormalized();
            ry0 = ea.getYnormalized();
        }
    }

    if (ea.getEventType() & GUIEventAdapter::RELEASE)
    {
        if (ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON)
        {
            // int lpush = false;
        }
        else if (ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            // int rpush = false;
        }
    }

    switch (ea.getEventType())
    {
        case (GUIEventAdapter::DRAG):
            if (ea.getButtonMask() == GUIEventAdapter::LEFT_MOUSE_BUTTON)
            {
                _cameraAngle += angleScale * (ly0 - ea.getYnormalized());
                if (_cameraAngle > 89)
                {
                    _cameraAngle = 89;
                }
                if (_cameraAngle < -89)
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
                if (_cameraDistance < 1)
                {
                    _cameraDistance = 1;
                }
                ry0 = ea.getYnormalized();
            }
            break;
            return false;

        case (GUIEventAdapter::SCROLL):
        {
            static int scroll = 0;
            switch (ea.getScrollingMotion())
            {
                case (osgGA::GUIEventAdapter::SCROLL_NONE):
                    break;
                case (osgGA::GUIEventAdapter::SCROLL_LEFT):
                    break;
                case (osgGA::GUIEventAdapter::SCROLL_RIGHT):
                    break;
                case (osgGA::GUIEventAdapter::SCROLL_2D):
                    break;
                case (osgGA::GUIEventAdapter::SCROLL_DOWN):
                    scroll = -1;
                    break;
                case (osgGA::GUIEventAdapter::SCROLL_UP):
                    scroll = 1;
                    break;
            }
            _cameraDistance -= scrollScale * _cameraDistance * (static_cast<float>(scroll));
            return false;
        }
        case (GUIEventAdapter::FRAME):
        {
            static double old_frametime = 0;
#if 1
            double current_frame_time = ea.getTime();
            double dt                 = current_frame_time - old_frametime;
            old_frametime             = current_frame_time;

#else
            double current_frame_time = elapsedTime();
#endif

            if (dt > 1)
            {
                dt = 0.1;
            }

            addEvent(ea);
            if (calcMovement(dt, false))
                us.requestRedraw();
            return false;
        }

        default:
            return false;
    }
    return false;
}

void RubberbandManipulator::computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const
{
    if (track_tx_ != nullptr)
    {
        nodeRotation = track_tx_->getAttitude();
        if (track_node_ != nullptr)
        {
            nodeCenter = track_tx_->getPosition() + nodeRotation * track_node_->getBound().center();
        }
        else
        {
            nodeCenter = track_tx_->getPosition();
        }
    }
    else
    {
        if (track_node_ != nullptr)
        {
            nodeCenter = track_node_->getBound().center();
        }
    }
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
    (void)matrix;
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
    osg::Vec3     up(0.0, 0.0, 1.0);
    osg::Vec3d    nodeCenter;
    osg::Quat     nodeRotation;
    osg::Matrix   cameraTargetRotation;
    float         springDC;
    osg::Vec3     cameraTargetPosition(0, 0, 0);
    osg::Vec3     cameraToTarget(0, 0, 0);
    float         x, y, z;
    CustomCamera* custom_cam = GetCurrentCustomCamera();

    relative_pos_.set(0.0, 0.0, 0.0);

    computeNodeCenterAndRotation(nodeCenter, nodeRotation);
    osg::Matrix nodeRot;
    nodeRot.makeRotate(nodeRotation);
    osg::Vec3 nodeFocusPoint = osg::Vec3(NODE_CENTER_OFFSET_X, NODE_CENTER_OFFSET_Y, NODE_CENTER_OFFSET_Z) + nodeCenter;

    if (_mode == RB_MODE_TOP)
    {
        _cameraRotation = -90;
        _cameraAngle    = 90;
        x               = -_cameraDistance * (cosf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        y               = -_cameraDistance * (sinf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        relative_pos_.set(x, y, _cameraDistance);  // Put a small number to prevent undefined camera angle
    }
    else if (_mode == RB_MODE_RUBBER_BAND)
    {
        relative_pos_.set(-_cameraDistance, 0.0, _cameraDistance * atan(_cameraAngle * 0.0174533f));
    }
    else if (_mode == RB_MODE_DRIVER)
    {
        _cameraRotation = 0;
        _cameraAngle    = 0;
        relative_pos_.set(1.0, 0.0, 0.0);
    }
    else if (_mode >= RB_MODE_CUSTOM)
    {
        _cameraRotation = 0.0;
        _cameraAngle    = 0;
        if (custom_cam && !custom_cam->GetFixPos() && !custom_cam->GetFixRot())
        {
            relative_pos_.set(custom_cam->GetPos());
        }
        else
        {
            relative_pos_.set(0.0, 0.0, 0.0);
        }
    }
    else
    {
        if (_mode == RB_MODE_FIXED)
        {
            _cameraRotation = 0;
        }
        x = -_cameraDistance * (cosf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        y = -_cameraDistance * (sinf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        z = _cameraDistance * sinf(_cameraAngle * 0.0174533f);

        relative_pos_.set(x, y, z);
    }

    // Transform the camera target offset position
#if 1  // use only heading to align camera position
    osg::Matrix m_rot(nodeRotation);
    cameraTargetRotation.setRotate(osg::Quat(atan2(m_rot(0, 1), m_rot(0, 0)), osg::Z_AXIS));
#else  // use also pitch and roll to align camera position
    cameraTargetRotation.setRotate(nodeRotation);
#endif

    cameraTargetPosition = cameraTargetRotation.preMult(relative_pos_);

    if (reset)
    {
        _eye = nodeFocusPoint + cameraTargetPosition;
        cameraVel.set(0, 0, 0);
        cameraAcc.set(0, 0, 0);
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
        springDC  = 2 * sqrt(springFC);
        cameraAcc = cameraToTarget * springFC - cameraVel * springDC;
        cameraVel += cameraAcc * static_cast<float>(dt);

        if (_mode == RB_MODE_FIXED || _mode == RB_MODE_ORBIT || _mode == RB_MODE_TOP || _mode == RB_MODE_DRIVER || _mode >= RB_MODE_CUSTOM)
        {
            _eye = nodeFocusPoint + cameraTargetPosition;
        }
        else
        {
            _eye += cameraVel * static_cast<float>(dt);
        }
    }

    // Create the view matrix
    if (_mode >= RB_MODE_CUSTOM && custom_cam && (custom_cam->GetFixPos() || custom_cam->GetFixRot()))
    {
        osg::Vec3 cam_pos = custom_cam->GetPos();
        osg::Vec3 cam_rot = custom_cam->GetRot();

        // Create a view matrix for custom position, or center Front Looking Camrera (FLC)

        if (custom_cam->GetFixRot())
        {
            osg::Matrix translate = osg::Matrix::translate(-cam_pos);

            osg::Quat   rot(static_cast<float>(-M_PI_2),
                          osg::Vec3(osg::X_AXIS),  // rotate so that Z axis points up
                          static_cast<float>(M_PI_2) - cam_rot[0],
                          osg::Vec3(osg::Y_AXIS),  // rotate so that X is forward and apply heading
                          cam_rot[1],
                          osg::Vec3(osg::X_AXIS)  // apply pitch
            );
            osg::Matrix localRotation = osg::Matrix::rotate(rot);

            if (custom_cam->GetFixPos())
            {
                _matrix = translate * localRotation;
            }
            else
            {
                // Find position relative focus object
                // Camera transform is the inverse of focus object rotation and position
                _matrix.makeRotate(nodeRotation.inverse());
                osg::Matrix node_trans = osg::Matrix::translate(-nodeCenter);
                _matrix.preMult(node_trans);
                _matrix = _matrix * translate * localRotation;
            }
        }
        else if (custom_cam->GetFixPos())
        {
            _matrix.makeLookAt(cam_pos, nodeFocusPoint, osg::Vec3(0, 0, 1));
        }
    }
    else if (_mode == RB_MODE_DRIVER)
    {
        // Create a view matrix for driver position, or center Front Looking Camrera (FLC)
        osg::Matrix localTx(0.0,
                            0.0,
                            -1.0,
                            0.0,
                            -1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            DRIVER_CENTER_OFFSET_Y,
                            -DRIVER_CENTER_OFFSET_Z,
                            DRIVER_CENTER_OFFSET_X,
                            1.0);

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
        _matrix.makeLookAt(_eye, nodeFocusPoint, nodeRotation * osg::Vec3(0, -1, 0));
    }
    else
    {
        _matrix.makeLookAt(_eye, nodeFocusPoint, up);
    }

    return true;
}
