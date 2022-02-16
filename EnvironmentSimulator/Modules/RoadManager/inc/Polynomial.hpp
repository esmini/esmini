#pragma once
class Polynomial {
   public:
	Polynomial() : a_(0), b_(0), c_(0), d_(0), p_scale_(1.0) {}
	Polynomial(double a, double b, double c, double d, double p_scale = 1)
		: a_(a), b_(b), c_(c), d_(d), p_scale_(p_scale) {}
	void Set(double a, double b, double c, double d, double p_scale = 1);
	void SetA(double a) { a_ = a; }
	void SetB(double b) { b_ = b; }
	void SetC(double c) { c_ = c; }
	void SetD(double d) { d_ = d; }
	double GetA() { return a_; }
	double GetB() { return b_; }
	double GetC() { return c_; }
	double GetD() { return d_; }
	double GetPscale() { return p_scale_; }
	double Evaluate(double s);
	double EvaluatePrim(double s);
	double EvaluatePrimPrim(double s);

   private:
	double a_;
	double b_;
	double c_;
	double d_;
	double p_scale_;
};
