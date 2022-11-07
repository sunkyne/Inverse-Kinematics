#include "InverseKin.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "GLSL.h"

using namespace std;
using namespace Eigen;

InverseKin::InverseKin() :
	r(1.0, 0.0, 1.0)
{
	
}

InverseKin::~InverseKin()
{
	
}

Eigen::Vector2d InverseKin::pos0(Eigen::VectorXd angle, shared_ptr<Link> link) {
	Vector3d p;
	pos0Helper(angle, link, p);
	return p.head<2>();
}

Eigen::Vector3d InverseKin::pos0Helper(Eigen::VectorXd angle, shared_ptr<Link> link, Vector3d& p) {
	if (link != NULL) {
		Matrix3d T = Matrix3d::Identity();
		T(0, 2) = link->getPosition().x();
		T(1, 2) = link->getPosition().y();
		Vector3d x = T * R0(angle(link->getDepth(), 0)) * pos0Helper(angle, link->getChild(), p);
		p = x;
		return x;
	}
	else {
		return r;
	}
}

Eigen::MatrixXd InverseKin::pos1(Eigen::VectorXd angle, shared_ptr<Link> link, int nlink) {
	MatrixXd p1(2, nlink);
	for (int i = 0; i < nlink; i++) {
		Vector3d p;
		pos1Helper(angle, link, p, i);
		p1.block<2, 1>(0, i) = p.head<2>();
	}
	return p1;
}

Eigen::Vector3d InverseKin::pos1Helper(Eigen::VectorXd angle, shared_ptr<Link> link, Vector3d& p, int i) {
	if (link != NULL) {
		Matrix3d T = Matrix3d::Identity();
		T(0, 2) = link->getPosition().x();
		T(1, 2) = link->getPosition().y();
		Vector3d x;
		if (link->getDepth() == i) {
			x = T * R1(angle(link->getDepth())) * pos1Helper(angle, link->getChild(), p, i);
		}
		else {
			x = T * R0(angle(link->getDepth())) * pos1Helper(angle, link->getChild(), p, i);
		}
		p = x;
		return x;
	}
	else {
		return r;
	}
}

Eigen::MatrixXd InverseKin::pos2(Eigen::VectorXd angle, shared_ptr<Link> link, int nlink) {
	MatrixXd p2(2*nlink, nlink);
	for (int i = 0; i < nlink; i++) {
		for (int j = 0; j < nlink; j++) {
			Vector3d p;
			pos2Helper(angle, link, p, i, j);
			p2.block<2, 1>(i*2, j) = p.head<2>();
		}
	}
	return p2;
}

Eigen::Vector3d InverseKin::pos2Helper(Eigen::VectorXd angle, shared_ptr<Link> link, Vector3d& p, int i, int j) {
	if (link != NULL) {
		Matrix3d T = Matrix3d::Identity();
		T(0, 2) = link->getPosition().x();
		T(1, 2) = link->getPosition().y();
		Vector3d x;
		if (link->getDepth() == i && link->getDepth() == j) {
			x = T * R2(angle(link->getDepth())) * pos2Helper(angle, link->getChild(), p, i, j);
		}
		else if (link->getDepth() == i || link->getDepth() == j) {
			x = T * R1(angle(link->getDepth())) * pos2Helper(angle, link->getChild(), p, i, j);
		}
		else {
			x = T * R0(angle(link->getDepth())) * pos2Helper(angle, link->getChild(), p, i, j);
		}
		p = x;
		return x;
	}
	else {
		return r;
	}
}

Eigen::Matrix3d InverseKin::R0(double angle) {
	Matrix3d R;
	R << cos(angle), -sin(angle), 0,
		sin(angle), cos(angle), 0,
		0, 0, 1;
	return R;
}
Eigen::Matrix3d InverseKin::R1(double angle) {
	Matrix3d R;
	R << -sin(angle), -cos(angle), 0,
		cos(angle), -sin(angle), 0,
		0, 0, 0;
	return R;
}
Eigen::Matrix3d InverseKin::R2(double angle) {
	Matrix3d R;
	R << -cos(angle), sin(angle), 0,
		-sin(angle), -cos(angle), 0,
		0, 0, 0;
	return R;
}