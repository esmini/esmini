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

#include "OSISensorView.hpp"
#include "RoadManager.hpp"

using namespace viewer;

#ifdef _USE_OSI

OSISensorDetection::OSISensorDetection(osg::ref_ptr<osg::Group> parent)
{
    parent_ = parent;

    detected_points_group_ = new osg::Group;
    detected_points_group_->setDataVariance(osg::Object::DYNAMIC);
    parent->addChild(detected_points_group_);
    detected_bb_group_ = new osg::Group;
    detected_bb_group_->setDataVariance(osg::Object::DYNAMIC);
    parent->addChild(detected_bb_group_);
}

OSISensorDetection::~OSISensorDetection()
{
    for (auto point : detected_points_)
    {
        delete point.second;
    }
    detected_points_.clear();

    for (auto car : detected_cars_)
    {
        delete car.second;
    }
    detected_cars_.clear();

    parent_->removeChild(detected_points_group_);
    parent_->removeChild(detected_bb_group_);
}

void OSISensorDetection::SensorUpdate(osi3::SensorView* sv)
{
    // lets find the detected objects that are still under the radar FOV
    if (sv)
    {
        bool found = false;
        for (auto point : detected_points_)
        {
            for (int i = 0; i < sv->global_ground_truth().lane_boundary_size(); ++i)
            {
                for (int j = 0; j < sv->global_ground_truth().lane_boundary()[i].boundary_line_size(); ++j)
                {
                    // lets see the lane boundary ID and check if detected points map are in the sensor view
                    std::string str_id = std::to_string(sv->global_ground_truth().lane_boundary()[i].id().value()) + std::to_string(j);
                    uint64_t    id     = std::stoul(str_id);
                    if (point.first == id)
                    {
                        found = true;
                        break;
                    }
                    if (found)
                    {
                        break;
                    }
                }
            }

            // If the point isn't in the sensor view then we hide it
            if (!found)
            {
                point.second->Hide();
            }
            // If the point was detected before, we show it again
            else if (found && !point.second->showing_)
            {
                point.second->Show();
            }
            found = false;
        }

        found = false;
        for (auto car : detected_cars_)
        {
            for (int i = 0; i < sv->global_ground_truth().moving_object_size(); i++)
            {
                // lets see the moving object ID and check if detected cars map are in the sensor view
                uint64_t id = sv->global_ground_truth().moving_object()[i].id().value();
                if (car.first == id)
                {
                    found = true;
                    break;
                }
            }

            // If the moving object isn't in the sensor view then we hide it
            if (!found)
            {
                car.second->Hide();
            }
            // If the moving object was detected before, we show it again
            else if (found && !car.second->showing_)
            {
                car.second->Show();
            }
            found = false;
        }

        // Lets check if it is needed to create new points and moving objects visuals
        double z_offset = 0.10;
        if (sv->has_global_ground_truth())
        {
            for (int i = 0; i < sv->global_ground_truth().moving_object_size(); i++)
            {
                // Get moving object position and dimension
                const osi3::Vector3d    moving_object_position  = sv->global_ground_truth().moving_object()[i].base().position();
                const osi3::Dimension3d moving_object_dimension = sv->global_ground_truth().moving_object()[i].base().dimension();

                // Get moving object id
                uint64_t id = sv->global_ground_truth().moving_object()[i].id().value();

                // If the moving object ID isn't in the cars map then we create one and added to the map
                if (detected_cars_.count(id) == 0)
                {
                    detected_cars_.emplace(id,
                                           new OSIDetectedCar(osg::Vec3(static_cast<float>(moving_object_position.x()),
                                                                        static_cast<float>(moving_object_position.y()),
                                                                        static_cast<float>(moving_object_position.z() + z_offset)),
                                                              moving_object_dimension.height() + 1.0,
                                                              moving_object_dimension.width() + 1.0,
                                                              moving_object_dimension.length() + 1.0,
                                                              detected_bb_group_));
                }
                else
                {
                    // Otherwise update the visual object
                    osg::Vec3 bb_dimension = detected_cars_.find(id)->second->bb_dimensions_;
                    detected_cars_.find(id)->second->Update(
                        osg::Vec3(static_cast<float>(moving_object_position.x()),
                                  static_cast<float>(moving_object_position.y()),
                                  static_cast<float>(moving_object_position.z()) + bb_dimension.z() + static_cast<float>(z_offset)));
                }
            }

            for (int i = 0; i < sv->global_ground_truth().lane_boundary_size(); ++i)
            {
                for (int j = 0; j < sv->global_ground_truth().lane_boundary()[i].boundary_line_size(); ++j)
                {
                    // Get line boundary id
                    std::string str_id = std::to_string(sv->global_ground_truth().lane_boundary()[i].id().value()) + std::to_string(j);
                    uint64_t    id     = std::stoul(str_id);

                    // Get line boundary position
                    const osi3::Vector3d boundary_line_position = sv->global_ground_truth().lane_boundary()[i].boundary_line()[j].position();

                    // If the lane boundary ID isn't in the points map then we create one and added to the map
                    if (detected_points_.count(id) == 0)
                    {
                        detected_points_.emplace(id,
                                                 new OSIDetectedPoint(osg::Vec3(static_cast<float>(boundary_line_position.x()),
                                                                                static_cast<float>(boundary_line_position.y()),
                                                                                static_cast<float>(boundary_line_position.z() + z_offset)),
                                                                      detected_points_group_));
                    }
                    else
                    {
                        // Otherwise update the visual object
                        detected_points_.find(id)->second->Update(osg::Vec3(static_cast<float>(boundary_line_position.x()),
                                                                            static_cast<float>(boundary_line_position.y()),
                                                                            static_cast<float>(boundary_line_position.z() + z_offset)));
                    }
                }
            }
        }
    }
}

OSIDetectedPoint::OSIDetectedPoint(const osg::Vec3 point, osg::ref_ptr<osg::Group> parent)
{
    parent_                                      = parent;
    osi_detection_geom_                          = new osg::Geometry;
    osi_detection_points_                        = new osg::Vec3Array;
    osi_detection_color_                         = new osg::Vec4Array;
    osg::ref_ptr<osg::Point> osi_detection_point = new osg::Point();

    // start point of each road mark
    osi_detection_points_->push_back(point);
    osi_detection_color_->push_back(osg::Vec4(SE_Color::Color2RBG(SE_Color::Color::GREEN)[0],
                                              SE_Color::Color2RBG(SE_Color::Color::GREEN)[1],
                                              SE_Color::Color2RBG(SE_Color::Color::GREEN)[2],
                                              1.0));

    osi_detection_point->setSize(8.0f);
    osi_detection_geom_->getOrCreateStateSet()->setAttributeAndModes(osi_detection_point, osg::StateAttribute::ON);
    osi_detection_geom_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    osi_detection_geom_->getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

    osi_detection_geom_->setVertexArray(osi_detection_points_.get());
    osi_detection_geom_->setColorArray(osi_detection_color_.get());
    osi_detection_geom_->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    osi_detection_geom_->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<int>(osi_detection_points_->size())));
    osi_detection_geom_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
    showing_ = true;

    parent_->addChild(osi_detection_geom_);
}

void OSIDetectedPoint::Update(const osg::Vec3 point)
{
    osi_detection_points_->clear();
    osi_detection_points_->push_back(point);
    osi_detection_points_->dirty();
    osi_detection_geom_->dirtyGLObjects();
    osi_detection_geom_->dirtyBound();
}

OSIDetectedPoint::~OSIDetectedPoint()
{
    parent_->removeChild(osi_detection_geom_);
}

OSIDetectedCar::OSIDetectedCar(const osg::Vec3 point, double h, double w, double l, osg::ref_ptr<osg::Group> parent)
{
    parent_ = parent;
    bb_dimensions_.set(static_cast<float>(l), static_cast<float>(w), static_cast<float>(h));

    car_                     = new osg::Group;
    osi_detection_geode_box_ = new osg::Geode;
    osi_detection_geode_box_->addDrawable(new osg::ShapeDrawable(new osg::Box()));
    osi_detection_geode_box_->getDrawable(0)->setDataVariance(osg::Object::DataVariance::DYNAMIC);

    osi_detection_tx_ = new osg::PositionAttitudeTransform;

    osg::Material* material = new osg::Material();

    // Set color of vehicle based on its index
    const float(*color)[3] = &SE_Color::Color2RBG(SE_Color::Color::GREEN);
    float b                = 1.0;  // brighness range (0,1)

    material->setAmbient(osg::Material::FRONT, osg::Vec4(b * (*color)[0], b * (*color)[1], b * (*color)[2], 1.0f));
    material->setDiffuse(osg::Material::FRONT, osg::Vec4(b * (*color)[0], b * (*color)[1], b * (*color)[2], 1.0f));

    // Set dimensions of the entity "box"
    osi_detection_tx_->setScale(bb_dimensions_);
    osi_detection_tx_->setPosition(osg::Vec3d(point.x(), point.y(), point.z()));

    // Draw only wireframe
    osg::PolygonMode* polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    osg::ref_ptr<osg::StateSet> stateset = osi_detection_geode_box_->getOrCreateStateSet();  // Get the StateSet of the group
    stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    stateset->setMode(GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    stateset->setDataVariance(osg::Object::DYNAMIC);
    osi_detection_geode_box_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);

    osi_detection_geode_center_ = new osg::Geode;
    osi_detection_geode_center_->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 0.2f)));
    osi_detection_geode_center_->getDrawable(0)->setDataVariance(osg::Object::DYNAMIC);

    osi_detection_geode_center_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
    car_->addChild(osi_detection_geode_center_);

    osi_detection_tx_->addChild(osi_detection_geode_box_);
    osi_detection_tx_->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    osi_detection_tx_->getOrCreateStateSet()->setAttribute(material);
    osi_detection_tx_->getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);
    car_->setName("BoundingBox");
    car_->addChild(osi_detection_tx_);

    parent_->addChild(car_);
}

OSIDetectedCar::~OSIDetectedCar()
{
    parent_->removeChild(car_);
}

void OSIDetectedCar::Update(const osg::Vec3 point)
{
    osi_detection_tx_->setPosition(point);
    osi_detection_tx_->dirtyBound();
    osi_detection_geode_center_->dirtyBound();
    osi_detection_geode_box_->dirtyBound();
}

void OSIDetectedCar::Show()
{
    car_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
    osi_detection_geode_box_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
    osi_detection_geode_center_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
    showing_ = true;
}

void OSIDetectedCar::Hide()
{
    car_->setNodeMask(0x0);
    osi_detection_geode_box_->setNodeMask(0x0);
    osi_detection_geode_center_->setNodeMask(0x0);
    showing_ = false;
}

#endif  // _USE_OSI