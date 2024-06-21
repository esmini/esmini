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

#ifndef OSI_SENSOR_VIEW_HPP_
#define OSI_SENSOR_VIEW_HPP_

#ifdef _USE_OSI

#include "viewer.hpp"
#include "OSIReporter.hpp"

namespace viewer
{

    class OSIDetectedPoint
    {
    public:
        osg::ref_ptr<osg::Group>     parent_;
        osg::ref_ptr<osg::Geometry>  osi_detection_geom_;
        osg::ref_ptr<osg::Vec3Array> osi_detection_points_;
        osg::ref_ptr<osg::Vec4Array> osi_detection_color_;
        bool                         showing_;

        OSIDetectedPoint(const osg::Vec3 point, osg::ref_ptr<osg::Group> parent);
        ~OSIDetectedPoint();
        void Show()
        {
            osi_detection_geom_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
            showing_ = true;
        };
        void Hide()
        {
            osi_detection_geom_->setNodeMask(0x0);
            showing_ = false;
        };
        void Update(const osg::Vec3 point);
    };

    class OSIDetectedCar
    {
    public:
        osg::ref_ptr<osg::Group>                     parent_;
        osg::ref_ptr<osg::Group>                     car_;
        osg::ref_ptr<osg::Geode>                     osi_detection_geode_box_;
        osg::ref_ptr<osg::Geode>                     osi_detection_geode_center_;
        osg::ref_ptr<osg::PositionAttitudeTransform> osi_detection_tx_;
        osg::Vec3                                    bb_dimensions_;
        bool                                         showing_;

        OSIDetectedCar(const osg::Vec3 point, double h, double w, double l, osg::ref_ptr<osg::Group> parent);
        ~OSIDetectedCar();
        void Show();
        void Hide();
        void Update(const osg::Vec3 point);
    };

    class OSISensorDetection
    {
    public:
        osg::ref_ptr<osg::Group> parent_;
        osg::ref_ptr<osg::Group> detected_points_group_;
        osg::ref_ptr<osg::Group> detected_bb_group_;

        std::map<uint64_t, OSIDetectedPoint*> detected_points_;
        std::map<uint64_t, OSIDetectedCar*>   detected_cars_;

        OSISensorDetection(osg::ref_ptr<osg::Group> parent);
        ~OSISensorDetection();
        void SensorUpdate(osi3::SensorView* sv);
    };

}  // namespace viewer

#endif  // _USE_OSI

#endif  // OSI_SENSOR_VIEW_HPP_
