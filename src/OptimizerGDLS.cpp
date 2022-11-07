#include "OptimizerGDLS.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerGDLS::OptimizerGDLS() :
	alphaInit(1e-4),
	gamma(0.5),
	tol(1e-3),
	iterMax(100),
	iter(0),
	debug(false)
{
	
}

OptimizerGDLS::OptimizerGDLS(int n) :
	alphaInit(1.0),
	gamma(0.5),
	tol(1e-3),
	iter(0),
	debug(false)
{
	this->iterMax = 10 * n;
}

OptimizerGDLS::~OptimizerGDLS()
{
	
}

VectorXd OptimizerGDLS::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	int n = xInit.rows();
	VectorXd x = xInit;
	VectorXd g(n);
	iter = 0;
	while (iter < iterMax) {
		double f = objective->evalObjective(x, g);
		double alpha = alphaInit;
		VectorXd dx;
		for (int i = 1; i <= iterMax; i++) {
			dx = -alpha * g;
			double fnew = objective->evalObjective(x + dx);
			if (fnew < f) {
				break;
			}
			alpha *= gamma;
		}

		if (debug) {
			double e = 1e-7;
			VectorXd g_(n);
			for (int i = 0; i < n; ++i) {
				VectorXd x_ = x;
				x_(i) += e;
				double f_ = objective->evalObjective(x_);
				g_(i) = (f_ - f) / e;
			}
			cout << "g: " << (g_ - g).norm() << endl;
		}

		x += dx;
		iter++;
		if (dx.norm() < tol) {
			break;
		}
	}
	return x;
}
