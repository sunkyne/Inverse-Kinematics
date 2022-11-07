#include "OptimizerGD.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerGD::OptimizerGD() :
	alpha(1.0),
	tol(1e-6),
	iterMax(100),
	iter(0)
{
	
}

OptimizerGD::~OptimizerGD()
{
	
}

VectorXd OptimizerGD::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	int n = xInit.rows();
	VectorXd x = xInit;
	VectorXd g(n);
	iter = 1;
	while (iter <= iterMax) {
		objective->evalObjective(x, g);
		VectorXd dx = -alpha * g;
		x += dx;
		if (dx.norm() < tol) {
			break;
		}
		iter++;
	}
	return x;
}
