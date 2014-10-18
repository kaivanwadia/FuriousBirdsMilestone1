#include "vectormath.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

const Matrix3d VectorMath::crossProductMatrix(const Eigen::Vector3d &v)
{
    Matrix3d result;
    result << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0;
    return result;
}

const Matrix3d VectorMath::rotationMatrix(const Vector3d &axisAngle)
{
    double theta = axisAngle.norm();
    Vector3d thetahat = axisAngle/theta;

    if(theta == 0)
        thetahat.setZero();

    Matrix3d result;
    result.setIdentity();
    result = cos(theta)*result + sin(theta)*crossProductMatrix(thetahat) + (1-cos(theta))*thetahat*thetahat.transpose();
    return result;
}

double VectorMath::randomUnitIntervalReal()
{
    return double(rand())/double(RAND_MAX);
}

const Vector3d VectorMath::axisAngle(const Matrix3d &rotationMatrix)
{
    Matrix3d RminusI;
    RminusI.setIdentity();
    RminusI = rotationMatrix - RminusI;

    std::cout<<"\nRot Matrix: \n"<<rotationMatrix<<std::endl;
    JacobiSVD<MatrixXd> svd(RminusI, ComputeFullV);
    assert(fabs(svd.singularValues()[2]) < 1e-8);
    Vector3d axis = svd.matrixV().col(2);
    Vector3d testAxis = perpToAxis(axis);
    Vector3d resultAxis = rotationMatrix*testAxis;
    double theta = atan2(testAxis.cross(resultAxis).dot(axis), testAxis.dot(resultAxis));
    return theta*axis;
}

const Vector3d VectorMath::perpToAxis(const Vector3d &v)
{
    int mincoord = 0;
    double minval = 0;
    for(int i=0; i<3; i++)
    {
        if(fabs(v[i]) < minval)
        {
            mincoord = i;
            minval = fabs(v[i]);
        }
    }
    Vector3d other(0,0,0);
    other[mincoord] = 1.0;
    Vector3d result = v.cross(other);
    result.normalize();
    return result;
}
