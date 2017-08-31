#include "arm.h"

using namespace std;
using namespace Eigen;

Arm::Arm() {
    l1 = 0.4;
    l2 = 0.3;
    l3 = 0.2;
    l4 = 0.2;

    r1 = Vector3f::Zero();
    r2 = Vector3f::Zero();
    r3 = Vector3f::Zero();
    r4 = Vector3f::Zero();

    matrix1 = MatrixXf::Identity(3,3);
    matrix2 = MatrixXf::Identity(3,3);
    matrix3 = MatrixXf::Identity(3,3);
    matrix4 = MatrixXf::Identity(3,3);

    v1 << 0, l1, 0;
    v2 << 0, l2, 0;
    v3 << 0, l3, 0;
    v4 << 0, l4, 0;
    p1 = matrix1 * v1;
    p2 = matrix2 * v2;
    p3 = matrix3 * v3;
    p4 = matrix4 * v4;

    error = numeric_limits<float>::max();
}

MatrixXf Arm::crossProductMatrix(Vector3f p) {
    MatrixXf cpm(3,3);
    cpm << 0, -p(2), p(1),
           p(2), 0, -p(0),
           -p(1), p(0), 0;
    return cpm;
}

void Arm::updateRotations(VectorXf dr) {
    Vector3f dr1(dr(0), dr(1), dr(2));
    Vector3f dr2(dr(3), dr(4), dr(5));
    Vector3f dr3(dr(6), dr(7), dr(8));
    Vector3f dr4(dr(9), dr(10), dr(11));

    matrix1 = Jacobian::rodriguez(dr1) * matrix1;
    matrix2 = Jacobian::rodriguez(dr2) * matrix2;
    matrix3 = Jacobian::rodriguez(dr3) * matrix3;
    matrix4 = Jacobian::rodriguez(dr4) * matrix4;

    r1 += dr1;
    r2 += dr2;
    r3 += dr3;
    r4 += dr4;
    // r1 = Jacobian::rodriguez(dr1) * r1;
    // r1 << rodr1(2,1), rodr1(0,2), rodr1(1,0);
    // r2 = Jacobian::rodriguez(dr2) * r2;
    // r2 << rodr2(2,1), rodr2(0,2), rodr2(1,0);
    // r3 = Jacobian::rodriguez(dr3) * r3;
    // r3 << rodr3(2,1), rodr3(0,2), rodr3(1,0);
    // r4 = Jacobian::rodriguez(dr4) * r4;
    // r4 << rodr4(2,1), rodr4(0,2), rodr4(1,0);
    // cout << "r1\n" << r1 << endl;
    // cout << "r2\n" << r2 << endl;
    // cout << "r3\n" << r3 << endl;
    // cout << "r4\n" << r4 << endl;
    // p1 = matrix1 * p1;
    // p2 = matrix2 * p2;
    // p3 = matrix3 * p3;
    // p4 = matrix4 * p4;
}

void Arm::drawArm() {
    GLUquadric* quad;
    quad = gluNewQuadric();

    Vector3f vtemp;
    Vector3f ptemp;
    // base balljoint
    glPushMatrix();
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
    glColor3f(0.8f, 1.0f, 1.0f);
    gluSphere(quad, .07, 32, 20);
    glPopMatrix();

    // inbetween ball
    for (float i = 0.01; i < l1; i+= .02) {
        glPushMatrix();
        glColor3f(1.0f, 1.0f, 1.0f);
        vtemp << 0, i, 0;
        ptemp = matrix1 * vtemp;
        glTranslatef(ptemp(0), ptemp(1), ptemp(2));
        gluSphere(quad, .02, 32, 20);
        glPopMatrix();
    }

    // // second ball
    glPushMatrix();
    glColor3f(0.2f, 0.8f, 0.8f);
    Vector3f point1 = matrix1 * v1;
    glTranslatef(point1(0), point1(1), point1(2));
    gluSphere(quad, .07, 32, 20);
    glPopMatrix();

    // inbetween ball
    for (float i = 0.01; i < l2; i+= .02) {
        glPushMatrix();
        glColor3f(1.0f,1.0f, 1.0f);
        vtemp << 0, i, 0;
        ptemp = point1 + matrix1 * matrix2 * vtemp;
        glTranslatef(ptemp(0), ptemp(1), ptemp(2));
        gluSphere(quad, .02, 32, 20);
        glPopMatrix();
    }

    // third ball
    glPushMatrix();
    glColor3f(0.0f, 0.7f, 0.9f);
    Vector3f point2 = point1 + matrix1 * matrix2 * v2;
    glTranslatef(point2(0), point2(1), point2(2));
    gluSphere(quad, .07, 32, 20);
    glPopMatrix();

    // inbetween ball
    for (float i = 0.01; i < l3; i+= .02) {
        glPushMatrix();
        glColor3f(1.0f, 1.0f, 1.0f);
        vtemp << 0, i, 0;
        ptemp = point2 + matrix1 * matrix2 * matrix3 * vtemp;
        glTranslatef(ptemp(0), ptemp(1), ptemp(2));
        gluSphere(quad, .02, 32, 20);
        glPopMatrix();
    }

    // fourth ball
    glPushMatrix();
    glColor3f(0.2f, 0.4f, 0.8f);
    Vector3f length3(0, l3, 0);
    Vector3f point3 = point2 + matrix1 * matrix2 * matrix3 * v3;
    glTranslatef(point3(0), point3(1), point3(2));
    gluSphere(quad, .07, 32, 20);
    glPopMatrix();

    // inbetween ball
    for (float i = 0.01; i < l4; i+= .02) {
        glPushMatrix();
        glColor3f(1.0f, 1.0f, 1.0f);
        vtemp << 0, i, 0;
        ptemp = point3 + matrix1 * matrix2 * matrix3 * matrix4 * vtemp;
        glTranslatef(ptemp(0), ptemp(1), ptemp(2));
        gluSphere(quad, .02, 32, 20);
        glPopMatrix();
    }

    // end effector ball
    glPushMatrix();
    glColor3f(0.0, 0.0f, 1.0f);
    Vector3f point4 = point3 + matrix1 * matrix2 * matrix3 * matrix4 * v4;
    glTranslatef(point4(0), point4(1), point4(2));
    gluSphere(quad, .04, 32, 20);
    glPopMatrix();

    gluDeleteQuadric(quad);
}

vector<float> Arm::getLengthVector() {
    vector<float> lengths;
    lengths.push_back(l1);
    lengths.push_back(l2);
    lengths.push_back(l3);
    lengths.push_back(l4);
    return lengths;
}

vector<Vector3f> Arm::getRotationVector() {
    vector<Vector3f> rotations;
    rotations.push_back(r1);
    rotations.push_back(r2);
    rotations.push_back(r3);
    rotations.push_back(r4);
    return rotations;
}

vector<MatrixXf> Arm::getRotationMatrices() {
    vector<MatrixXf> rotations;
    rotations.push_back(matrix1);
    rotations.push_back(matrix2);
    rotations.push_back(matrix3);
    rotations.push_back(matrix4);
    return rotations;
}

Vector3f Arm::getEndEffector() {
    // Vector3f pn = Vector3f::Zero(3)
    Vector3f v1(0, l1, 0);
    Vector3f v2(0, l2, 0);
    Vector3f v3(0, l3, 0);
    Vector3f v4(0, l4, 0);
    return matrix1 * v1 + 
           matrix1 * matrix2 * v2 + 
           matrix1 * matrix2 * matrix3 * v3 + 
           matrix1 * matrix2 * matrix3 * matrix4 * v4;
}

// Vector3f Arm::getEndEffector() {
//     return rodriguez * l4
// }