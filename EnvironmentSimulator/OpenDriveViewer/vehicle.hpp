
class Vehicle
{
public:
	Vehicle(double x, double y, double h, double length);
	void Update(double dt, int acceleration, int steering);

	double posX_;
	double posY_;
	double posZ_;
	double heading_;

	double velX_;
	double velY_;
	double velAngle_;
	double velAngleRelVehicleLongAxis_;
	double speed_;
	double wheelAngle_;
	double wheelRotation_;
	double headingDot_;

	double length_;
};

