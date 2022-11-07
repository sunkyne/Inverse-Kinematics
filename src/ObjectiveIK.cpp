#include "ObjectiveIK.h"
#include <cmath>

using namespace std;
using namespace Eigen;

ObjectiveIK::ObjectiveIK() :
	wTar(1e3),
	wReg(1e0),
	pTarget(1.0, 1.0)
{
	
}

ObjectiveIK::~ObjectiveIK()
{
	
}

double ObjectiveIK::evalObjective(const Eigen::VectorXd& x)
{
	Vector2d pDelta = IK.pos0(x, links) - pTarget;
	return (1.0 / 2.0) * wTar * pDelta.dot(pDelta) + (1.0 / 2.0) * wReg * x.transpose() * x;
}

double ObjectiveIK::evalObjective(const Eigen::VectorXd& x, Eigen::VectorXd& g)
{
	Vector2d pDelta = IK.pos0(x, links) - pTarget;
	g = wTar * pDelta.transpose() * IK.pos1(x, links, nlinks) + wReg * x.transpose();
	return evalObjective(x);
}

double ObjectiveIK::evalObjective(const Eigen::VectorXd& x, Eigen::VectorXd& g, Eigen::MatrixXd& H)
{
	Vector2d pDelta = IK.pos0(x, links) - pTarget;
	MatrixXd I(nlinks, nlinks);
	I.setIdentity();
	MatrixXd p2 = IK.pos2(x, links, nlinks);
	MatrixXd prod(nlinks, nlinks);
	for (int i = 0; i < nlinks; i++) {
		for (int j = 0; j < nlinks; j++) {
			prod(i, j) = pDelta.transpose() * p2.block<2, 1>(i * 2, j);
		}
	}
	H = wTar * (IK.pos1(x, links, nlinks).transpose() * IK.pos1(x, links, nlinks) + prod) + wReg * I;
	return evalObjective(x, g);
}
