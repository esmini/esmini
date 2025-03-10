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

#include "TopViewManipulator.hpp"
#include <osg/Quat>
#include <osg/Notify>
#include <osg/Transform>
#include <osg/ComputeBoundsVisitor>

#include "CommonMini.hpp"

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace osg;
using namespace osgGA;

#define NODE_CENTER_OFFSET_X 0.0
#define NODE_CENTER_OFFSET_Y 0.0
#define NODE_CENTER_OFFSET_Z 0.25

const float springFC            = 16.0f;
const float orbitCameraDistance = 25.0f;
const float topCameraDistance   = 2300;
const float orbitCameraAngle    = 13.0f;
const float orbitCameraRotation = -14.0f;

TopViewManipulator::TopViewManipulator(osg::Vec3d origin)
{
    _cameraAngle    = orbitCameraAngle;
    _cameraDistance = orbitCameraDistance;
    _cameraRotation = orbitCameraRotation;
    track_node_     = nullptr;
    track_tx_       = nullptr;
    origin_         = origin;
    mousePos_.set(0.0, 0.0);
    setMode(RB_MODE_TOP);
}

TopViewManipulator::~TopViewManipulator()
{
}

void TopViewManipulator::setMode(unsigned int mode)
{
    if (mode > GetNumberOfCameraModes())
    {
        mode = GetNumberOfCameraModes() - 1;
    }

    // If leaving locked views, then reset camera rotations
    if (_mode == RB_MODE_TOP)
    {
        _cameraAngle    = orbitCameraAngle;
        _cameraRotation = orbitCameraRotation;
        //_cameraDistance = orbitCameraDistance;
        _cameraDistance = topCameraDistance;
    }

    _mode = mode;
}

void TopViewManipulator::setTrackNode(osg::ref_ptr<osg::Node> node, bool calcDistance)
{
    if (!node)
    {
        osg::notify(osg::NOTICE) << "TopViewManipulator::setTrackBB(bb):  Unable to set tracked bounding box due to null Node" << std::endl;
        return;
    }
    track_node_ = node;

    if (calcDistance)
    {
        calculateCameraDistance();
    }
}

void TopViewManipulator::setTrackTransform(osg::ref_ptr<osg::PositionAttitudeTransform> tx)
{
    if (!tx)
    {
        osg::notify(osg::NOTICE) << "TopViewManipulator::setTrackTX(tx):  Unable to set tracked transofmration node due to null Node" << std::endl;
        return;
    }
    track_tx_ = tx;
}

void TopViewManipulator::calculateCameraDistance()
{
    const osg::MatrixList&    m = track_node_->getWorldMatrices();
    osg::ComputeBoundsVisitor cbv;
    track_node_->accept(cbv);
    osg::BoundingBox bb   = cbv.getBoundingBox();
    osg::Vec3        minV = bb._min * m.front();
    osg::Vec3        maxV = bb._max * m.front();
    _cameraDistance       = MAX((maxV.x() - minV.x() + maxV.y() - minV.y()) / 2, orbitCameraDistance);
}

void TopViewManipulator::init(const GUIEventAdapter&, GUIActionAdapter& us)
{
    (void)us;
    calcMovement(0, true);
}

void TopViewManipulator::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("NodeTracker: Space", "Reset the viewing position to home");
    usage.addKeyboardMouseBinding("NodeTracker: +", "Increase camera distance");
    usage.addKeyboardMouseBinding("NodeTracker: -", "Decrease camera distance");
}

bool TopViewManipulator::handle(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
    static float lx0 = 0;
    static float ly0 = 0;
    static float ry0 = 0;
    // float        angleScale  = 30.0;
    // float        zoomScale   = 1.0;
    float scrollScale = 0.2f;

    if (ea.getEventType() & (GUIEventAdapter::MOVE | GUIEventAdapter::PUSH | GUIEventAdapter::RELEASE))
    {
        double fov      = M_PI / 180.0;
        double camera_z = cameraPos_[2] - cameraTargetPos_[2];
        double x_offset = std::tan(fov / 2) * camera_z * ea.getXnormalized() * ea.getWindowWidth() / ea.getWindowHeight();
        double y_offset = std::tan(fov / 2) * camera_z * ea.getYnormalized();
        mousePos_.set(cameraPos_[0] - x_offset, cameraPos_[1] - y_offset);
    }

    if (ea.getHandled())
        return false;

    if (ea.getEventType() & GUIEventAdapter::PUSH)
    {
        if (ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        {
            // int lpush = true;
            lx0 = ea.getXnormalized();
            ly0 = ea.getYnormalized();
            // LOG("clicked: x:%f, y:%f\n", lx0, ly0);
        }
        // else if (ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        // {
        //     // int rpush = true;
        //     // float rx0 = ea.getXnormalized();
        //     ry0 = ea.getYnormalized();
        // }
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
            if (ea.getButtonMask() == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
            {
                static bool lxy0_init = false;
                if (lxy0_init)
                {
                    float  x        = ea.getY();
                    float  y        = ea.getX();
                    double fov      = M_PI / 180.0;
                    double camera_z = cameraPos_[2] - cameraTargetPos_[2];
                    double dx       = std::tan(fov / 2) * camera_z * (ea.getXnormalized() - lx0) * ea.getWindowWidth() / ea.getWindowHeight();
                    double dy       = std::tan(fov / 2) * camera_z * (ea.getYnormalized() - ly0);

                    if (bs_.contains(osg::Vec3d(cameraPos_[0] + dx, cameraPos_[1] + dy, 0.0)))
                    {
                        cameraPos_[0] += dx;
                        cameraPos_[1] += dy;
                        cameraTargetPos_[0] += dx;
                        cameraTargetPos_[1] += dy;
                    }
                    // LOG("CAMERA: %.2f, %.2f, %.2f", cameraPos_[0], cameraPos_[1], cameraPos_[2]);
                    //  printf("       x: %.2f,        y: %.2f\n", x, y);
                    //  printf("x_offset: %.2f, y_offset: %.2f\n", _cameraXOffset, _cameraYOffset);
                    //  printf("   zoom: %.2f, y_offset: %.2f\n", zoomScale);
                }
                else
                {
                    // printf("skip\n");
                }

                // _cameraAngle += angleScale * (ly0 - ea.getYnormalized());
                // if (_cameraAngle > 89)
                // {
                //     _cameraAngle = 89;
                // }
                // if (_cameraAngle < -89)
                // {
                //     _cameraAngle = -89;
                // }
                // _cameraRotation += 2 * angleScale * (lx0 - ea.getXnormalized());

                lx0       = ea.getXnormalized();
                ly0       = ea.getYnormalized();
                lxy0_init = true;
            }
            // if (ea.getButtonMask() == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
            // {
            //     _cameraDistance -= zoomScale * _cameraDistance * (ry0 - ea.getYnormalized());
            //     if (_cameraDistance < 1)
            //     {
            //         _cameraDistance = 1;
            //     }
            //     ry0 = ea.getYnormalized();
            //     cameraPos_[2] = cameraTargetPos_[2] + _cameraDistance;
            // }
            // break;
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
            double old_cameraDistance = _cameraDistance;
            _cameraDistance -= scrollScale * _cameraDistance * (static_cast<float>(scroll));
            _cameraDistance = std::min(200000.0, std::max(800.0, _cameraDistance));
            double dx       = (mousePos_[0] - cameraPos_[0]) / old_cameraDistance * _cameraDistance;
            double dy       = (mousePos_[1] - cameraPos_[1]) / old_cameraDistance * _cameraDistance;
            // LOG("camera distance: %.2f", _cameraDistance);
            if (bs_.contains(osg::Vec3d(mousePos_[0] - dx, mousePos_[1] - dy, 0.0)))
            {
                cameraPos_[0]       = mousePos_[0] - dx;
                cameraPos_[1]       = mousePos_[1] - dy;
                cameraTargetPos_[0] = mousePos_[0] - dx;
                cameraTargetPos_[1] = mousePos_[1] - dy;
                cameraPos_[2]       = cameraTargetPos_[2] + _cameraDistance;
                // LOG("scroll camera: %.2f %.2f %.2f", cameraPos_[0], cameraPos_[1], cameraPos_[2]);
            }
            else
            {
                _cameraDistance = old_cameraDistance;
            }

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

void TopViewManipulator::computeNodeCenterAndRotation(osg::Vec3d& nodeCenter, osg::Quat& nodeRotation) const
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
    }
}

void TopViewManipulator::flushEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
}

void TopViewManipulator::addEvent(const GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
}

void TopViewManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    (void)matrix;
}

osg::Matrixd TopViewManipulator::getMatrix() const
{
    return osg::Matrixd::inverse(_matrix);
}

osg::Matrixd TopViewManipulator::getInverseMatrix() const
{
    return _matrix;
}

bool TopViewManipulator::calcMovement(double dt, bool reset)
{
    osg::Vec3d  up(0.0, 0.0, 1.0);
    osg::Vec3d  nodeCenter;
    osg::Quat   nodeRotation;
    osg::Matrix cameraTargetRotation;
    float       springDC;
    osg::Vec3d  cameraTargetPosition(0, 0, 0);
    osg::Vec3d  cameraToTarget(0, 0, 0);
    double      x, y, z;

    relative_pos_.set(0.0, 0.0, 0.0);

    computeNodeCenterAndRotation(nodeCenter, nodeRotation);
    osg::Matrix nodeRot;
    nodeRot.makeRotate(nodeRotation);
    osg::Vec3d nodeFocusPoint = osg::Vec3d(NODE_CENTER_OFFSET_X, NODE_CENTER_OFFSET_Y, NODE_CENTER_OFFSET_Z) + nodeCenter;

    if (_mode == RB_MODE_TOP)
    {
        _cameraRotation = -90;
        _cameraAngle    = 90;
        x               = -_cameraDistance * (cosf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        y               = -_cameraDistance * (sinf(_cameraRotation * 0.0174533f) * cosf(_cameraAngle * 0.0174533f));
        relative_pos_.set(x, y, _cameraDistance);  // Put a small number to prevent undefined camera angle
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
        cameraPos_       = _eye;
        cameraTargetPos_ = nodeFocusPoint;
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

        if (_mode == RB_MODE_TOP)
        {
            _eye = nodeFocusPoint + cameraTargetPosition;
        }
    }

    // Create the view matrix
    if (_mode == RB_MODE_TOP)
    {
        // LOG("camera pos: %.2f, %.2f, %.2f | target pos: %.2f, %.2f, %.2f", cameraPos_[0], cameraPos_[1], cameraPos_[2], cameraTargetPos_[0],
        // cameraTargetPos_[1], cameraTargetPos_[2]);
        _matrix.makeLookAt(cameraPos_, cameraTargetPos_, osg::Vec3(0, -1, 0));
    }

    return true;
}

void TopViewManipulator::moveCameraTo(double x, double y)
{
    cameraPos_[0]       = x + origin_[0];
    cameraPos_[1]       = y + origin_[1];
    cameraTargetPos_[0] = x + origin_[0];
    cameraTargetPos_[1] = y + origin_[1];
}

void TopViewManipulator::setCameraDistance(double d)
{
    cameraPos_[2]   = cameraTargetPos_[2] + d;
    _cameraDistance = d;
}

std::vector<double> TopViewManipulator::getCameraSettings()
{
    return {cameraPos_[0], cameraPos_[1], cameraPos_[2], cameraTargetPos_[0], cameraTargetPos_[1], cameraTargetPos_[2], _cameraDistance};
}

void TopViewManipulator::setCameraSettings(const std::vector<double>& settings)
{
    cameraPos_[0]       = settings[0];
    cameraPos_[1]       = settings[1];
    cameraPos_[2]       = settings[2];
    cameraTargetPos_[0] = settings[3];
    cameraTargetPos_[1] = settings[4];
    cameraTargetPos_[2] = settings[5];
}

void TopViewManipulator::backupCameraSettings()
{
    cameraSettings_.push_back(cameraPos_[0]);
    cameraSettings_.push_back(cameraPos_[1]);
    cameraSettings_.push_back(cameraPos_[2]);
    cameraSettings_.push_back(cameraTargetPos_[0]);
    cameraSettings_.push_back(cameraTargetPos_[1]);
    cameraSettings_.push_back(cameraTargetPos_[2]);
    cameraSettings_.push_back(_cameraDistance);
}

void TopViewManipulator::restoreCameraSettings()
{
    if (cameraSettings_.size() >= 7)
    {
        _cameraDistance = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraTargetPos_[2] = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraTargetPos_[1] = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraTargetPos_[0] = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraPos_[2] = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraPos_[1] = cameraSettings_.back();
        cameraSettings_.pop_back();
        cameraPos_[0] = cameraSettings_.back();
        cameraSettings_.pop_back();
    }
    cameraSettings_.clear();
}

void TopViewManipulator::SetBound(const osg::BoundingSphere& bs)
{
    bs_ = bs;
}
