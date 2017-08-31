#ifndef ARM_H
#define ARM_H

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "jacobian.h"

class Arm {

private:
	float l1;
	float l2;
	float l3;
	float l4;

	Eigen::Vector3f r1;
	Eigen::Vector3f r2;
	Eigen::Vector3f r3; 
	Eigen::Vector3f r4;

	Eigen::Matrix3f matrix1;
	Eigen::Matrix3f matrix2;
	Eigen::Matrix3f matrix3;
	Eigen::Matrix3f matrix4;

	Eigen::Vector3f p1;
	Eigen::Vector3f p2;
	Eigen::Vector3f p3;
	Eigen::Vector3f p4;

	Eigen::Vector3f v1;
	Eigen::Vector3f v2;
	Eigen::Vector3f v3;
	Eigen::Vector3f v4;

public:
	//contructor for the arm
	float error;
	Arm();
	void drawArm();
	std::vector<float> getLengthVector();
	std::vector<Eigen::Vector3f> getRotationVector();
	std::vector<Eigen::MatrixXf> getRotationMatrices();
	void updateRotations(Eigen::VectorXf dr);
	Eigen::MatrixXf crossProductMatrix(Eigen::Vector3f p);
	Eigen::Vector3f getEndEffector();
};

#endif