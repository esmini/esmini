#include "Polynomial.hpp"
double Polynomial::Evaluate(double p) {
	p *= p_scale_;

	return (a_ + p * b_ + p * p * c_ + p * p * p * d_);
}

double Polynomial::EvaluatePrim(double p) {
	p *= p_scale_;

	return (b_ + 2 * p * c_ + 3 * p * p * d_);
}

double Polynomial::EvaluatePrimPrim(double p) {
	p *= p_scale_;

	return (2 * c_ + 6 * p * d_);
}

void Polynomial::Set(double a, double b, double c, double d, double p_scale) {
	a_ = a;
	b_ = b;
	c_ = c;
	d_ = d;
	p_scale_ = p_scale;
}
