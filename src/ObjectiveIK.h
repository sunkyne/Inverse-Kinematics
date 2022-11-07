#pragma once
#ifndef OBJECTIVE_IK_H
#define OBJECTIVE_IK_H

#include "Objective.h"
#include "InverseKin.h"
#include "Link.h"
#include <iostream>
#include <memory>

class ObjectiveIK : public Objective
{
public:
	ObjectiveIK();
	virtual ~ObjectiveIK();
	virtual double evalObjective(const Eigen::VectorXd& x);
	virtual double evalObjective(const Eigen::VectorXd& x, Eigen::VectorXd& g);
	virtual double evalObjective(const Eigen::VectorXd& x, Eigen::VectorXd& g, Eigen::MatrixXd& H);

	void setTarget(Eigen::Vector2d pTarget) { this->pTarget = pTarget; }
	void setLink(std::shared_ptr<Link> links) { this->links = links; }
	void setNLink(int nlinks) { this->nlinks = nlinks; }

private:
	float wTar;
	float wReg;
	InverseKin IK;
	Eigen::Vector2d pTarget;
	std::shared_ptr<Link> links;
	int nlinks;
};

#endif
