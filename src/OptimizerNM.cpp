#include "OptimizerNM.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerNM::OptimizerNM() :
	tol(1e-3),
	iterMax(100),
	iter(0),
	debug(false)
{
	
}

OptimizerNM::OptimizerNM(int n) :
	tol(1e-3),
	iter(0),
	debug(false)
{
	this->iterMax = 10 * n;
}

OptimizerNM::~OptimizerNM()
{
	
}

VectorXd OptimizerNM::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	int n = xInit.rows();
	VectorXd x = xInit;
	VectorXd g(n);
	MatrixXd h(n, n);
	iter = 0;
	while (iter < iterMax) {
		objective->evalObjective(x, g, h);

		if (debug) {
			double e = 1e-7;
			MatrixXd h_(n, n);
			for (int i = 0; i < n; ++i) {
				VectorXd x_ = x;
				x_(i) += e;
				VectorXd g_(n);
				objective->evalObjective(x_, g_);
				h_.col(i) = (g_ - g) / e;
			}
			cout << "H: " << (h_ - h).norm() << endl;
		}

		VectorXd dx = (-h).ldlt().solve(g);
		x += dx;
		iter++;
		if (dx.norm() < tol) {
			break;
		}
	}
	return x;
}
