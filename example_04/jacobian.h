#ifndef JACOBIAN_H
#define JACOBIAN_H

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "Eigen/Dense"
#include <vector>

#define PI 3.14159265



class Jacobian {
public:
	Jacobian();
	static Eigen::MatrixXf rodriguez(Eigen::Vector3f ri);
	static Eigen::Vector3f endEffector(std::vector<float> lengths, std::vector<Eigen::Vector3f> rotations);
	static Eigen::MatrixXf jacobianVectors(std::vector<float> lengths, std::vector<Eigen::Vector3f> rotations);
	static Eigen::MatrixXf jacobianMatrices(std::vector<float> lengths, std::vector<Eigen::MatrixXf> rotations);
	static Eigen::MatrixXf pseudoInverse(Eigen::MatrixXf jacobian);
};

#endif