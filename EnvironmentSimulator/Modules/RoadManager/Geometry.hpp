#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "RoadManager.hpp"
#include "Polynomial.hpp"
class Geometry {
   public:
	typedef enum {
		GEOMETRY_TYPE_UNKNOWN,
		GEOMETRY_TYPE_LINE,
		GEOMETRY_TYPE_ARC,
		GEOMETRY_TYPE_SPIRAL,
		GEOMETRY_TYPE_POLY3,
		GEOMETRY_TYPE_PARAM_POLY3,
	} GeometryType;

	Geometry() : s_(0.0), x_(0.0), y_(0), hdg_(0), length_(0), type_(GeometryType::GEOMETRY_TYPE_UNKNOWN) {}
	Geometry(double s, double x, double y, double hdg, double length, GeometryType type)
		: s_(s), x_(x), y_(y), hdg_(hdg), length_(length), type_(type) {}
	virtual ~Geometry() {}

	GeometryType GetType() { return type_; }
	double GetLength() { return length_; }
	virtual double GetX() { return x_; }
	void SetX(double x) { x_ = x; }
	virtual double GetY() { return y_; }
	void SetY(double y) { y_ = y; }
	virtual double GetHdg() { return GetAngleInInterval2PI(hdg_); }
	void SetHdg(double hdg) { hdg_ = hdg; }
	double GetS() { return s_; }
	virtual double EvaluateCurvatureDS(double ds) = 0;
	virtual void Print();
	virtual void EvaluateDS(double ds, double* x, double* y, double* h);
	virtual void Save(pugi::xml_node& geometry);
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }

   protected:
	double s_;
	double x_;
	double y_;
	double hdg_;
	double length_;
	GeometryType type_;
	std::vector<UserData*> user_data_;
};

class Line : public Geometry {
   public:
	Line() {}
	Line(double s, double x, double y, double hdg, double length)
		: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_LINE) {}
	~Line(){};

	void Print();
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds) {
		(void)ds;
		return 0;
	}
	void Save(pugi::xml_node&) override;
};

class Arc : public Geometry {
   public:
	Arc() : curvature_(0.0) {}
	Arc(double s, double x, double y, double hdg, double length, double curvature)
		: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC), curvature_(curvature) {}
	~Arc() {}

	double EvaluateCurvatureDS(double ds) {
		(void)ds;
		return curvature_;
	}
	double GetRadius() { return std::fabs(1.0 / curvature_); }
	double GetCurvature() { return curvature_; }
	void Print();
	void EvaluateDS(double ds, double* x, double* y, double* h);
	void Save(pugi::xml_node&) override;

   private:
	double curvature_;
};

class Spiral : public Geometry {
   public:
	Spiral()
		: curv_start_(0.0),
		  curv_end_(0.0),
		  c_dot_(0.0),
		  x0_(0.0),
		  y0_(0.0),
		  h0_(0.0),
		  s0_(0.0),
		  arc_(0),
		  line_(0) {}
	Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);

	~Spiral(){};

	double GetCurvStart() { return curv_start_; }
	double GetCurvEnd() { return curv_end_; }
	double GetX0() { return x0_; }
	double GetY0() { return y0_; }
	double GetH0() { return h0_; }
	double GetS0() { return s0_; }
	double GetCDot() { return c_dot_; }
	void SetX0(double x0) { x0_ = x0; }
	void SetY0(double y0) { y0_ = y0; }
	void SetH0(double h0) { h0_ = h0; }
	void SetS0(double s0) { s0_ = s0; }
	void SetCDot(double c_dot) { c_dot_ = c_dot; }
	void Print();
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void SetX(double x);
	void SetY(double y);
	void SetHdg(double h);
	void Save(pugi::xml_node&) override;

	Arc* arc_;
	Line* line_;

   private:
	double curv_start_;
	double curv_end_;
	double c_dot_;
	double x0_;	 // 0 if spiral starts with curvature = 0
	double y0_;	 // 0 if spiral starts with curvature = 0
	double h0_;	 // 0 if spiral starts with curvature = 0
	double s0_;	 // 0 if spiral starts with curvature = 0
};

class Poly3 : public Geometry {
   public:
	Poly3() : umax_(0.0) {}
	Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d);
	~Poly3(){};

	void SetUMax(double umax) { umax_ = umax; }
	double GetUMax() { return umax_; }
	void Print();
	Polynomial GetPoly3() { return poly3_; }
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void Save(pugi::xml_node&) override;

	Polynomial poly3_;

   private:
	double umax_;
	void EvaluateDSLocal(double ds, double& u, double& v);
};

class ParamPoly3 : public Geometry {
   public:
	enum PRangeType { P_RANGE_UNKNOWN, P_RANGE_NORMALIZED, P_RANGE_ARC_LENGTH };

	ParamPoly3() {}
	ParamPoly3(double s,
			   double x,
			   double y,
			   double hdg,
			   double length,
			   double aU,
			   double bU,
			   double cU,
			   double dU,
			   double aV,
			   double bV,
			   double cV,
			   double dV,
			   PRangeType p_range)
		: Geometry(s, x, y, hdg, length, GeometryType::GEOMETRY_TYPE_PARAM_POLY3), p_range_(p_range) {
		poly3U_.Set(aU, bU, cU, dU, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
		poly3V_.Set(aV, bV, cV, dV, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
		calcS2PMap(p_range);
	}
	~ParamPoly3(){};

	void Print();
	Polynomial GetPoly3U() { return poly3U_; }
	Polynomial GetPoly3V() { return poly3V_; }
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void calcS2PMap(PRangeType p_range);
	double s2p_map_[PARAMPOLY3_STEPS + 1][2];
	double S2P(double s);
	void Save(pugi::xml_node&) override;

	PRangeType p_range_;
	Polynomial poly3U_;
	Polynomial poly3V_;
};

#endif
