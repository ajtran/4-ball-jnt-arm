#include "Jacobian.h"
#include <iostream>
using namespace Eigen;
using namespace std;

Jacobian::Jacobian() {};

MatrixXf Jacobian::rodriguez(Vector3f ri) {
	Vector3f rhat = ri.normalized();
	MatrixXf r(3,3);
	r << 0, -rhat(2), rhat(1),
		 rhat(2), 0, -rhat(0),
		 -rhat(1), rhat(0), 0;
	float theta = pow(ri.dot(ri), 0.5) * PI/180;
	MatrixXf I = MatrixXf::Identity(3,3);
	return r * sin(theta) + I + (r * r) * (1 - cos(theta));
}

Vector3f Jacobian::endEffector(vector<float> lengths, vector<Vector3f> rotations) {
	Vector3f pn = Vector3f::Zero(3);
	MatrixXf rodn = MatrixXf::Identity(3,3);
	for (int i = 0; i < lengths.size(); i++) {
		rodn *= rodriguez(rotations[i]);
		Vector3f vn(0, lengths[i], 0);
		pn += rodn * vn;
	}
	return pn;
}

MatrixXf Jacobian::jacobianVectors(vector<float> lengths, vector<Vector3f> rotations) {
	MatrixXf jacobian = MatrixXf::Zero(3, 3 * lengths.size());
	MatrixXf jacobianPart;

	Matrix3f rodn = rodriguez(rotations[lengths.size() - 1]);

	Vector3f pn(0, 0, 0);
	Vector4f pn4x1(pn(0), pn(1), pn(2), 1);

	MatrixXf rodi0 = MatrixXf::Identity(3, 3);

	for (int i = 0; i < lengths.size(); i++) {

		MatrixXf rodi = rodriguez(rotations[i]);

		Vector3f vi(0, lengths[i], 0);
		Vector3f pi = rodi * vi;

		
		MatrixXf x = MatrixXf::Identity(4,4);
		x << rodi(0, 0), rodi(0, 1), rodi(0, 2), pi(0),
			 rodi(1, 0), rodi(1, 1), rodi(1, 2), pi(1),
  			 rodi(2, 0), rodi(2, 1), rodi(2, 2), pi(2),
			 0, 0, 0, 1;

		for (int j = i + 1; j < lengths.size(); j++) {
			MatrixXf rodj = rodriguez(rotations[j]);

			Vector3f vj(0, lengths[j], 0);
			Vector3f pj = rodj * vj;

			MatrixXf xj = MatrixXf::Identity(4, 4);
			xj << rodj(0, 0), rodj(0, 1), rodj(0, 2), pj(0),
				  rodj(1, 0), rodj(1, 1), rodj(1, 2), pj(1),
	  			  rodj(2, 0), rodj(2, 1), rodj(2, 2), pj(2),
	  			  0, 0, 0, 1;
			
			x = x * xj;
		}

		MatrixXf product = 1 * x * pn4x1;
		MatrixXf cross(3,3);
		cross << 0, -product(2), product(1),
				 product(2), 0, -product(0),
				 -product(1), product(0), 0;

		jacobianPart = (-1 * rodi0) * cross;

		rodi0 = rodi0 * rodi;

		jacobian(0, 0 + 3 * i) = jacobianPart(0, 0);
		jacobian(0, 1 + 3 * i) = jacobianPart(0, 1);
		jacobian(0, 2 + 3 * i) = jacobianPart(0, 2);
		jacobian(1, 0 + 3 * i) = jacobianPart(1, 0);
		jacobian(1, 1 + 3 * i) = jacobianPart(1, 1);
		jacobian(1, 2 + 3 * i) = jacobianPart(1, 2);
		jacobian(2, 0 + 3 * i) = jacobianPart(2, 0);
		jacobian(2, 1 + 3 * i) = jacobianPart(2, 1);
		jacobian(2, 2 + 3 * i) = jacobianPart(2, 2);
	}
	return jacobian;
}

MatrixXf Jacobian::jacobianMatrices(vector<float> lengths, vector<MatrixXf> rotations) {
	MatrixXf jacobian = MatrixXf::Zero(3, 3 * lengths.size());

	MatrixXf jacobianPart;

	Vector3f pn(0, 0, 0);
	Vector4f pn4x1(pn(0), pn(1), pn(2), 1);

	MatrixXf rodi0 = MatrixXf::Identity(3, 3);

	for (int i = 0; i < lengths.size(); i++) {

		MatrixXf rodi = rotations[i];

		Vector3f vi(0, lengths[i], 0);
		Vector3f pi = rodi * vi;
		
		MatrixXf x = MatrixXf::Identity(4,4);
		x << rodi(0, 0), rodi(0, 1), rodi(0, 2), pi(0),
			 rodi(1, 0), rodi(1, 1), rodi(1, 2), pi(1),
  			 rodi(2, 0), rodi(2, 1), rodi(2, 2), pi(2),
  			 0, 0, 0, 1;

		for (int j = i + 1; j < lengths.size(); j++) {
			MatrixXf rodj = rotations[j];

			Vector3f vj(0, lengths[j], 0);
			Vector3f pj = rodj * vj;

			MatrixXf xj = MatrixXf::Identity(4, 4);

			xj << rodj(0, 0), rodj(0, 1), rodj(0, 2), pj(0),
				  rodj(1, 0), rodj(1, 1), rodj(1, 2), pj(1),
	  			  rodj(2, 0), rodj(2, 1), rodj(2, 2), pj(2),
	  			  0, 0, 0, 1;

			x = x * xj;
		}

		MatrixXf product = 1 * x * pn4x1;
		MatrixXf cross(3,3);
		cross << 0, -product(2), product(1),
				 product(2), 0, -product(0),
				 -product(1), product(0), 0;

		jacobianPart = (-1 * rodi0) * cross;

		rodi0 = rodi0 * rodi;

		jacobian(0, 0 + 3 * i) = jacobianPart(0, 0);
		jacobian(0, 1 + 3 * i) = jacobianPart(0, 1);
		jacobian(0, 2 + 3 * i) = jacobianPart(0, 2);
		jacobian(1, 0 + 3 * i) = jacobianPart(1, 0);
		jacobian(1, 1 + 3 * i) = jacobianPart(1, 1);
		jacobian(1, 2 + 3 * i) = jacobianPart(1, 2);
		jacobian(2, 0 + 3 * i) = jacobianPart(2, 0);
		jacobian(2, 1 + 3 * i) = jacobianPart(2, 1);
		jacobian(2, 2 + 3 * i) = jacobianPart(2, 2);
	}

	// cout << "new jacoby\n" << jacobian << endl;
	return jacobian;
}

MatrixXf Jacobian::pseudoInverse(MatrixXf jacobian) {
	JacobiSVD<MatrixXf> svd(jacobian, ComputeFullU | ComputeFullV);
	Vector3f svds = svd.singularValues();
	float err = 0.01;

	MatrixXf diagonals = MatrixXf::Zero(jacobian.cols(), jacobian.rows());
	for (int i = 0; i < svds.rows(); i++) {
		if ((svds(i) == 0) || (abs(svds(i)) <= err)) {
			diagonals(i, i) = 0;
		}
		else {
			diagonals(i, i) = 1/svds(i);
		}
	}	
	return svd.matrixV() * diagonals * svd.matrixU().transpose();
}

