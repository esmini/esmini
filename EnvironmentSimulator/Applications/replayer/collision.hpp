#ifndef COLLISION_HPP
#define COLLISION_HPP

struct Point
{
	float x;
	float y;
};

typedef struct 
{
	int id;
	std::string name;
	viewer::EntityModel* entityModel;
	struct ObjectPositionStruct pos;
	osg::ref_ptr<osg::Vec3Array> trajPoints;
	viewer::PolyLine* trajectory;
	float wheel_angle;
	float wheel_rotation;
	bool visible;
	std::vector<Point> corners;
} ScenarioEntity;

void updateCorners(ScenarioEntity& entity)
{
    entity.corners.clear();

    Point front_right;
    Point front_left;
    Point rear_left;
    Point rear_right;

	auto p_front_right_x = entity.pos.x + entity.entityModel->modelBB_.xMax();
	auto p_front_right_y = entity.pos.y + entity.entityModel->modelBB_.yMin();
	front_right.x = entity.pos.x + (p_front_right_x - entity.pos.x) * cos(entity.pos.h) - (p_front_right_y - entity.pos.y) * sin(entity.pos.h);
	front_right.y = entity.pos.y + (p_front_right_x - entity.pos.x) * sin(entity.pos.h) + (p_front_right_y - entity.pos.y) * cos(entity.pos.h);

	auto p_front_left_x = entity.pos.x + entity.entityModel->modelBB_.xMax();
	auto p_front_left_y = entity.pos.y + entity.entityModel->modelBB_.yMax();
	front_left.x = entity.pos.x + (p_front_left_x - entity.pos.x) * cos(entity.pos.h) - (p_front_left_y - entity.pos.y) * sin(entity.pos.h);
	front_left.y = entity.pos.y + (p_front_left_x - entity.pos.x) * sin(entity.pos.h) + (p_front_left_y - entity.pos.y) * cos(entity.pos.h);
	
	auto p_rear_left_x = entity.pos.x + entity.entityModel->modelBB_.xMin();
	auto p_rear_left_y = entity.pos.y + entity.entityModel->modelBB_.yMax();
	rear_left.x = entity.pos.x + (p_rear_left_x - entity.pos.x) * cos(entity.pos.h) - (p_rear_left_y - entity.pos.y) * sin(entity.pos.h);
	rear_left.y = entity.pos.y + (p_rear_left_x - entity.pos.x) * sin(entity.pos.h) + (p_rear_left_y - entity.pos.y) * cos(entity.pos.h);
	
	auto p_rear_right_x  = entity.pos.x + entity.entityModel->modelBB_.xMin();
	auto p_rear_right_y = entity.pos.y + entity.entityModel->modelBB_.yMin();
	rear_right.x = entity.pos.x + (p_rear_right_x - entity.pos.x) * cos(entity.pos.h) - (p_rear_right_y - entity.pos.y) * sin(entity.pos.h);
	rear_right.y = entity.pos.y + (p_rear_right_x - entity.pos.x) * sin(entity.pos.h) + (p_rear_right_y - entity.pos.y) * cos(entity.pos.h);

    entity.corners = {front_right, front_left, rear_left, rear_right};
}

double dot(const Point& axis_normalized, const Point& corner)
{
    return axis_normalized.x * corner.x + axis_normalized.y * corner.y;
}

Point calculate_normalized_axis_projection(const Point& current_point, const Point& next_point)
{
    const double axis_x = -(next_point.y - current_point.y);
    const double axis_y =   next_point.x - current_point.x;
    const double magnitude = hypot(axis_x, axis_y);

    Point axis_normalised;
    axis_normalised.x = axis_x / magnitude;
    axis_normalised.y = axis_y / magnitude;

    return axis_normalised;
}

void compute_projections(const std::vector<Point>& ego_corners, const std::vector<Point>& target_corners, const Point& axis_normalized, std::vector<double>& projections_a, std::vector<double>& projections_b)
{
    projections_a.reserve(ego_corners.size());
    projections_b.reserve(target_corners.size());

    for (size_t i = 0; i < ego_corners.size(); i++)
    {
        const double projection_a = dot(axis_normalized, ego_corners[i]);
        const double projection_b = dot(axis_normalized, target_corners[i]);

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
        Point current_point;
        current_point.x = ego.corners[i].x;
        current_point.y = ego.corners[i].y;

        Point next_point;
        next_point.x = ego.corners[(i + 1) % ego.corners.size()].x; 
        next_point.y = ego.corners[(i + 1) % ego.corners.size()].y; 

        Point axis_normalized = calculate_normalized_axis_projection(current_point, next_point);

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
        Point current_point;
        current_point.x = target.corners[i].x;
        current_point.y = target.corners[i].y;

        Point next_point;
        next_point.x = target.corners[(i + 1) % target.corners.size()].x; 
        next_point.y = target.corners[(i + 1) % target.corners.size()].y; 

        Point axis_normalized = calculate_normalized_axis_projection(current_point, next_point);

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