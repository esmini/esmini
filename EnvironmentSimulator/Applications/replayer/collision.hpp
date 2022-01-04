#ifndef COLLISION_HPP
#define COLLISION_HPP

class Point
{
public:
    Point() : x_(0), y_(0) {}
    Point(double x, double y) : x_(x), y_(y) {}

    Point operator + (Point const& p) const {
        Point res;
        res.x_ = x_ + p.x_;
        res.y_ = y_ + p.y_;
        return res;
    }

    Point operator - (Point const& p) const {
        Point res;
        res.x_ = x_ - p.x_;
        res.y_ = y_ - p.y_;
        return res;
    }

    Point Rotate(double angle) {
        Point res;
        res.x_ = x_ * cos(angle) - y_ * sin(angle);
        res.y_ = x_ * sin(angle) + y_ * cos(angle);
        return res;
    }

    double Dot(const Point& corner) const {
        return x_ * corner.x_ + y_ * corner.y_;
    }

    double x() const { return x_; }
    double y() const { return y_; }

private:
    float x_;
	float y_;
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
    OSCBoundingBox bounding_box;
	std::vector<Point> corners;
} ScenarioEntity;

void updateCorners(ScenarioEntity& entity)
{
    Point bb_center(entity.bounding_box.center_.x_, entity.bounding_box.center_.y_);
    Point bb_dim(entity.bounding_box.dimensions_.length_, entity.bounding_box.dimensions_.width_);

    Point front_right = Point(entity.pos.x, entity.pos.y) + Point(bb_center.x() + bb_dim.x() / 2.0f, bb_center.y() - bb_dim.y() / 2.0f).Rotate(entity.pos.h);
    Point front_left = Point(entity.pos.x, entity.pos.y) + Point(bb_center.x() + bb_dim.x() / 2.0f, bb_center.y() + bb_dim.y() / 2.0f).Rotate(entity.pos.h);
    Point rear_left = Point(entity.pos.x, entity.pos.y) + Point(bb_center.x() - bb_dim.x() / 2.0f, bb_center.y() + bb_dim.y() / 2.0f).Rotate(entity.pos.h);
    Point rear_right = Point(entity.pos.x, entity.pos.y) + Point(bb_center.x() - bb_dim.x() / 2.0f, bb_center.y() - bb_dim.y() / 2.0f).Rotate(entity.pos.h);

    entity.corners = {front_right, front_left, rear_left, rear_right};
}

Point calculate_normalized_axis_projection(const Point& current_point, const Point& next_point)
{
    const Point axis(next_point - current_point);
    const double magnitude = hypot(-axis.y(), axis.x());

    return Point(axis.x() / magnitude, axis.y() / magnitude);
}

void compute_projections(const std::vector<Point>& ego_corners, const std::vector<Point>& target_corners, const Point& axis_normalized, std::vector<double>& projections_a, std::vector<double>& projections_b)
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
        Point current_point = ego.corners[i];
        Point next_point(ego.corners[(i + 1) % ego.corners.size()].x(), ego.corners[(i + 1) % ego.corners.size()].y());

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
        Point current_point(target.corners[i]);
        Point next_point(target.corners[(i + 1) % target.corners.size()].x(), target.corners[(i + 1) % target.corners.size()].y());

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