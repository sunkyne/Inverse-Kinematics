#pragma once
#ifndef Inverse_Kin_H
#define Inverse_Kin_H

#define GLEW_STATIC
#include <GL/glew.h>

#include <memory>
#include <Eigen/Dense>

#include "Link.h"

class InverseKin
{
public:
	InverseKin();
	virtual ~InverseKin();
	Eigen::Vector2d pos0(Eigen::VectorXd angle, std::shared_ptr<Link> link);
	Eigen::MatrixXd pos1(Eigen::VectorXd angle, std::shared_ptr<Link> link, int nlink);
	Eigen::MatrixXd pos2(Eigen::VectorXd angle, std::shared_ptr<Link> link, int nlink);
	Eigen::Vector3d pos0Helper(Eigen::VectorXd angle, std::shared_ptr<Link> link, Eigen::Vector3d& p);
	Eigen::Vector3d pos1Helper(Eigen::VectorXd angle, std::shared_ptr<Link> link, Eigen::Vector3d& p, int i);
	Eigen::Vector3d pos2Helper(Eigen::VectorXd angle, std::shared_ptr<Link> link, Eigen::Vector3d& p, int i, int j);
	Eigen::Matrix3d R0(double angle);
	Eigen::Matrix3d R1(double angle);
	Eigen::Matrix3d R2(double angle);
	
private:
	Eigen::Vector3d r;

};

#endif
