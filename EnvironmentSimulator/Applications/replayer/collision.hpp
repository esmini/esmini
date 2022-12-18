#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "CommonMini.hpp"

typedef struct
{
	int id;
	std::string name;
	viewer::EntityModel* entityModel;
	struct ObjectPositionStructDat pos;
	osg::ref_ptr<osg::Vec3Array> trajPoints;
	viewer::PolyLine* trajectory;
	float wheel_angle;
	float wheel_rotation;
	bool visible;
    OSCBoundingBox bounding_box;
	std::vector<SE_Vector> corners;
} ScenarioEntity;

void updateCorners(ScenarioEntity& entity)
{
    SE_Vector bb_center(entity.bounding_box.center_.x_, entity.bounding_box.center_.y_);
    SE_Vector bb_dim(entity.bounding_box.dimensions_.length_, entity.bounding_box.dimensions_.width_);

    SE_Vector front_right = SE_Vector(entity.pos.x, entity.pos.y) + SE_Vector(bb_center.x() + bb_dim.x() / 2.0, bb_center.y() - bb_dim.y() / 2.0).Rotate(entity.pos.h);
    SE_Vector front_left = SE_Vector(entity.pos.x, entity.pos.y) + SE_Vector(bb_center.x() + bb_dim.x() / 2.0, bb_center.y() + bb_dim.y() / 2.0).Rotate(entity.pos.h);
    SE_Vector rear_left = SE_Vector(entity.pos.x, entity.pos.y) + SE_Vector(bb_center.x() - bb_dim.x() / 2.0, bb_center.y() + bb_dim.y() / 2.0).Rotate(entity.pos.h);
    SE_Vector rear_right = SE_Vector(entity.pos.x, entity.pos.y) + SE_Vector(bb_center.x() - bb_dim.x() / 2.0, bb_center.y() - bb_dim.y() / 2.0).Rotate(entity.pos.h);

    entity.corners = {front_right, front_left, rear_left, rear_right};
}

SE_Vector calculate_normalized_axis_projection(const SE_Vector& current_SE_Vector, const SE_Vector& next_SE_Vector)
{
    const SE_Vector axis(next_SE_Vector - current_SE_Vector);
    const double magnitude = hypot(-axis.y(), axis.x());

    return SE_Vector(axis.x() / magnitude, axis.y() / magnitude);
}

void compute_projections(const std::vector<SE_Vector>& ego_corners, const std::vector<SE_Vector>& target_corners, const SE_Vector& axis_normalized, std::vector<double>& projections_a, std::vector<double>& projections_b)
{
    projections_a.reserve(ego_corners.size());
    projections_b.reserve(target_corners.size());

    for (size_t i = 0; i < ego_corners.size(); i++)
    {
        const double projection_a = axis_normalized.Dot(ego_corners[i]);
        const double projection_b = axis_normalized.Dot(target_corners[i]);

        projections_a.push_back(projection_a);
        projections_b.push_back(projection_b);
    }
}

bool is_overlapping(const std::vector<double>& projections_a, const std::vector<double>& projections_b)
{
    const double max_projection_a = *std::max_element(projections_a.begin(), projections_a.end());
    const double min_projection_a = *std::min_element(projections_a.begin(), projections_a.end());

    const double max_projection_b = *std::max_element(projections_b.begin(), projections_b.end());
    const double min_projection_b = *std::min_element(projections_b.begin(), projections_b.end());

    // Does not intersect
    if (max_projection_a < min_projection_b || max_projection_b < min_projection_a)
    {
        return false;
    }

    // Projection overlaps but may not necessarily be intersecting yet
    return true;
}

bool separating_axis_intersect(const ScenarioEntity& ego, const ScenarioEntity& target)
{
    for (size_t i = 0; i < ego.corners.size(); i++)
    {
        SE_Vector current_point = ego.corners[i];
        SE_Vector next_point(ego.corners[(i + 1) % ego.corners.size()].x(), ego.corners[(i + 1) % ego.corners.size()].y());

        SE_Vector axis_normalized = calculate_normalized_axis_projection(current_point, next_point);

        std::vector<double> projections_a;
        std::vector<double> projections_b;

        compute_projections(ego.corners, target.corners, axis_normalized, projections_a, projections_b);
        if (!is_overlapping(projections_a, projections_b))
        {
            return false;
        }

    }

    for (size_t i = 0; i < target.corners.size(); i++)
    {
        SE_Vector current_point(target.corners[i]);
        SE_Vector next_point(target.corners[(i + 1) % target.corners.size()].x(), target.corners[(i + 1) % target.corners.size()].y());

        SE_Vector axis_normalized = calculate_normalized_axis_projection(current_point, next_point);

        std::vector<double> projections_a;
        std::vector<double> projections_b;

        compute_projections(ego.corners, target.corners, axis_normalized, projections_a, projections_b);
        if (!is_overlapping(projections_a, projections_b))
        {
            return false;
        }

    }

    return true; // Intersects
}

#endif