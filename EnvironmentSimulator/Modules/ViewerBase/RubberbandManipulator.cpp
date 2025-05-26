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

RubberbandManipulator::RubberbandManipulator(unsigned int mode, osg::Vec3d origin, double& time_ref) : time_ref_(time_ref)

{
    cameraAngle_        = orbitCameraAngle;
    cameraBaseDistance_ = orbitCameraDistance;
    cameraRotation_     = orbitCameraRotation;
    track_node_         = nullptr;
    track_tx_           = nullptr;
    origin_             = origin;
    setMode(mode);
    explicitCenter_.Reset();
}

double osgGA::RubberbandManipulator::GetCameraDistance()
{
    return cameraBaseDistance_ + zoom_distance_;
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
    int index = static_cast<int>(mode_) - RB_MODE_CUSTOM;

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
    if (mode_ == RB_MODE_TOP || mode_ == RB_MODE_DRIVER || mode_ >= RB_MODE_CUSTOM)
    {
        cameraAngle_        = orbitCameraAngle;
        cameraRotation_     = orbitCameraRotation;
        cameraBaseDistance_ = orbitCameraDistance;
    }

    mode_ = mode;

    if (mode == RB_MODE_RUBBER_BAND)
    {
        cameraAngle_        = orbitCameraAngle;
        cameraBaseDistance_ = orbitCameraDistance;
    }
    else if (mode == RB_MODE_TOP)
    {
        cameraBaseDistance_ = topCameraDistance;
    }
    else if (mode >= CAMERA_MODE::RB_MODE_CUSTOM)
    {
    }
}

void RubberbandManipulator::setTrackNode(osg::ref_ptr<osg::Node> node, bool calcDistance)
{
    track_node_ = node;

    if (calcDistance)
    {
        calculateCameraDistance();
    }
    explicitCenter_.Reset();
}

const osg::Node* RubberbandManipulator::getTrackNode() const
{
    return track_node_.get();
}

void RubberbandManipulator::setCenterAndDistance(osg::Vec3 center, double distance)
{
    track_node_ = nullptr;
    track_tx_   = nullptr;
    explicitCenter_.Set(center);
    cameraBaseDistance_ = distance;
}

void RubberbandManipulator::setTrackTransform(osg::ref_ptr<osg::PositionAttitudeTransform> tx)
{
    track_tx_ = tx;
    explicitCenter_.Reset();
}

void RubberbandManipulator::calculateCameraDistance()
{
    const osg::MatrixList&    m = track_node_->getWorldMatrices();
    osg::ComputeBoundsVisitor cbv;
    track_node_->accept(cbv);
    osg::BoundingBox bb   = cbv.getBoundingBox();
    osg::Vec3        minV = bb._min * m.front();
    osg::Vec3        maxV = bb._max * m.front();
    cameraBaseDistance_   = MAX((maxV.x() - minV.x() + maxV.y() - minV.y()) / 2, orbitCameraDistance);
    zoom_distance_        = 0.0;
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
                cameraAngle_ += angleScale * (ly0 - ea.getYnormalized());
                if (cameraAngle_ > 89)
                {
                    cameraAngle_ = 89;
                }
                if (cameraAngle_ < -89)
                {
                    cameraAngle_ = -89;
                }
                cameraRotation_ += 2 * angleScale * (lx0 - ea.getXnormalized());

                lx0 = ea.getXnormalized();
                ly0 = ea.getYnormalized();
            }
            if (ea.getButtonMask() == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
            {
                zoom_distance_ -= zoomScale * GetCameraDistance() * (ry0 - ea.getYnormalized());
                if (GetCameraDistance() < 1)
                {
                    zoom_distance_ = 1.0 - cameraBaseDistance_;
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
            zoom_distance_ -= scrollScale * cameraBaseDistance_ * scroll;
            return false;
        }
        case (GUIEventAdapter::FRAME):
        {
            static double old_simulation_time = 0.0;
            static double old_system_time     = 0.0;

            double dt = minimum(time_ref_ - old_simulation_time, 0.1);
            if (dt < 1e-6)
            {
                // If simulation time is not updated, use system time
                dt = minimum(ea.getTime() - old_system_time, 0.1);
            }

            old_simulation_time = time_ref_;
            old_system_time     = ea.getTime();

            addEvent(ea);

            if (calcMovement(dt, false))
            {
                us.requestRedraw();
            }

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
            nodeCenter = (track_tx_->getPosition() + origin_) + nodeRotation * track_node_->getBound().center();
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
        else
        {
            nodeCenter = explicitCenter_.GetRef();
        }
    }
}

void RubberbandManipulator::flushEventStack()
{
    ga_t1_ = NULL;
    ga_t0_ = NULL;
}

void RubberbandManipulator::addEvent(const GUIEventAdapter& ea)
{
    ga_t1_ = ga_t0_;
    ga_t0_ = &ea;
}

void RubberbandManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    (void)matrix;
}

osg::Matrixd RubberbandManipulator::getMatrix() const
{
    return osg::Matrixd::inverse(matrix_);
}

osg::Matrixd RubberbandManipulator::getInverseMatrix() const
{
    return matrix_;
}

bool RubberbandManipulator::calcMovement(double dt, bool reset)
{
    osg::Vec3d    up(0.0, 0.0, 1.0);
    osg::Vec3d    nodeCenter;
    osg::Quat     nodeRotation;
    osg::Matrix   cameraTargetRotation;
    float         springDC;
    osg::Vec3d    cameraTargetPosition(0, 0, 0);
    osg::Vec3d    cameraToTarget(0, 0, 0);
    double        x, y, z;
    CustomCamera* custom_cam = GetCurrentCustomCamera();

    relative_pos_.set(0.0, 0.0, 0.0);

    computeNodeCenterAndRotation(nodeCenter, nodeRotation);
    osg::Matrix nodeRot;
    nodeRot.makeRotate(nodeRotation);
    osg::Vec3d nodeFocusPoint = osg::Vec3d(NODE_CENTER_OFFSET_X, NODE_CENTER_OFFSET_Y, NODE_CENTER_OFFSET_Z) + nodeCenter;

    if (mode_ == RB_MODE_TOP)
    {
        cameraRotation_ = -90;
        cameraAngle_    = 90;
        x               = -GetCameraDistance() * (cosf(cameraRotation_ * 0.0174533f) * cosf(cameraAngle_ * 0.0174533f));
        y               = -GetCameraDistance() * (sinf(cameraRotation_ * 0.0174533f) * cosf(cameraAngle_ * 0.0174533f));
        relative_pos_.set(x, y, GetCameraDistance());  // Put a small number to prevent undefined camera angle
    }
    else if (mode_ == RB_MODE_RUBBER_BAND)
    {
        relative_pos_.set(-GetCameraDistance(), 0.0, GetCameraDistance() * atan(cameraAngle_ * 0.0174533f));
    }
    else if (mode_ == RB_MODE_DRIVER)
    {
        cameraRotation_ = 0;
        cameraAngle_    = 0;
        relative_pos_.set(1.0, 0.0, 0.0);
    }
    else if (mode_ >= RB_MODE_CUSTOM)
    {
        cameraRotation_ = 0.0;
        cameraAngle_    = 0;
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
        if (mode_ == RB_MODE_FIXED)
        {
            cameraRotation_ = 0;
        }
        x = -GetCameraDistance() * (cosf(cameraRotation_ * 0.0174533f) * cosf(cameraAngle_ * 0.0174533f));
        y = -GetCameraDistance() * (sinf(cameraRotation_ * 0.0174533f) * cosf(cameraAngle_ * 0.0174533f));
        z = GetCameraDistance() * sinf(cameraAngle_ * 0.0174533f);

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
        eye_ = nodeFocusPoint + cameraTargetPosition;
        cameraVel_.set(0, 0, 0);
        cameraAcc_.set(0, 0, 0);
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
        cameraToTarget = (nodeFocusPoint + cameraTargetPosition) - eye_;

        // Update camera state
        springDC   = 2 * sqrt(springFC);
        cameraAcc_ = cameraToTarget * springFC - cameraVel_ * springDC;
        cameraVel_ += cameraAcc_ * static_cast<float>(dt);

        if (mode_ == RB_MODE_FIXED || mode_ == RB_MODE_ORBIT || mode_ == RB_MODE_TOP || mode_ == RB_MODE_DRIVER || mode_ >= RB_MODE_CUSTOM)
        {
            eye_ = nodeFocusPoint + cameraTargetPosition;
        }
        else
        {
            eye_ += cameraVel_ * static_cast<float>(dt);
        }
    }

    // Create the view matrix
    if (mode_ >= RB_MODE_CUSTOM && custom_cam && (custom_cam->GetFixPos() || custom_cam->GetFixRot()))
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
                matrix_ = translate * localRotation;
            }
            else
            {
                // Find position relative focus object
                // Camera transform is the inverse of focus object rotation and position
                matrix_.makeRotate(nodeRotation.inverse());
                osg::Matrix node_trans = osg::Matrix::translate(-nodeCenter);
                matrix_.preMult(node_trans);
                matrix_ = matrix_ * translate * localRotation;
            }
        }
        else if (custom_cam->GetFixPos())
        {
            matrix_.makeLookAt(cam_pos, nodeFocusPoint, osg::Vec3(0, 0, 1));
        }
    }
    else if (mode_ == RB_MODE_DRIVER)
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
        matrix_.makeRotate(nodeRotation.inverse());
        osg::Matrix trans;
        trans.makeTranslate(-nodeCenter);
        matrix_.preMult(trans);

        // Combine driver and camera transform
        matrix_ = matrix_ * localTx;
    }
    else if (mode_ == RB_MODE_TOP)
    {
        matrix_.makeLookAt(eye_, nodeFocusPoint, nodeRotation * osg::Vec3(0, -1, 0));
    }
    else
    {
        matrix_.makeLookAt(eye_, nodeFocusPoint, up);
    }

    return true;
}
