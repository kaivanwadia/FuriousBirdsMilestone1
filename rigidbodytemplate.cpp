#include "rigidbodytemplate.h"
#include "mesh.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

RigidBodyTemplate::RigidBodyTemplate(std::string &meshFilename)
{
    mesh_ = new Mesh(meshFilename);
    calculateVolume();
    calculateCenterOfMass();
    translateRigidBodyToOrigin();
    calculateSmallestRadius();
    scaleRigidBody();
    calculateVolume();
    calculateInertiaTensorMatrix();
    computeEigenVectorsandValues();
}

void RigidBodyTemplate::calculateVolume()
{
    vol = 0;
    for (int i = 0; i < mesh_->getNumFaces(); i++)
    {
        double faceArea = 0;
        Vector3d pi = mesh_->getVert(mesh_->getFace(i)[0]);
        Vector3d pj = mesh_->getVert(mesh_->getFace(i)[1]);
        Vector3d pk = mesh_->getVert(mesh_->getFace(i)[2]);
        double norm = mesh_->getFaceArea(i) * 2;
        Vector3d temp = ((pi+pj+pk)/6)*norm;
        faceArea = temp.dot(mesh_->getFaceNormal(i));
        vol += faceArea;
    }
    vol = vol/3;
}

void RigidBodyTemplate::calculateCenterOfMass()
{
    centerOfMass.setZero();
    for (int i = 0; i < mesh_->getNumFaces(); i++)
    {
        Vector3d pi = mesh_->getVert(mesh_->getFace(i)[0]);
        Vector3d pj = mesh_->getVert(mesh_->getFace(i)[1]);
        Vector3d pk = mesh_->getVert(mesh_->getFace(i)[2]);
        Vector3d faceCenter = ((mesh_->getFaceNormal(i) * (mesh_->getFaceArea(i) * 2))/2);
        faceCenter = ((faceCenter.array()*(pi.array() * pi.array() + pj.array() * pj.array() + pk.array() * pk.array() + pi.array() * pj.array() + pi.array() * pk.array() + pj.array() * pk.array())).matrix())/12;
        centerOfMass += faceCenter;
    }
    centerOfMass = centerOfMass/vol;
}

void RigidBodyTemplate::translateRigidBodyToOrigin()
{
    mesh_->translate((-1)*centerOfMass);
    centerOfMass.setZero();
}

void RigidBodyTemplate::calculateSmallestRadius()
{
    double maxDistance = 0;
    double dist = 0;
    for (int i = 0; i < mesh_->getNumVerts(); i++)
    {
        dist = mesh_->getVert(i).norm();
        if (dist > maxDistance)
            maxDistance = dist;
    }
    radius = maxDistance;
}

void RigidBodyTemplate::scaleRigidBody()
{
    mesh_->scale(1/radius);
}

void RigidBodyTemplate::calculateInertiaTensorMatrix()
{
    inertiaTensor.setZero();
    Vector3d xyzSquared;
    Vector3d yzXZxyVector;
    xyzSquared.setZero();
    yzXZxyVector.setZero();
    for (int i = 0; i < mesh_->getNumFaces(); i++)
    {
        Vector3d pi = mesh_->getVert(mesh_->getFace(i)[0]);
        Vector3d pj = mesh_->getVert(mesh_->getFace(i)[1]);
        Vector3d pk = mesh_->getVert(mesh_->getFace(i)[2]);
        double a = pi[0];
        double b = pi[1];
        double c = pi[2];
        double l = pj[0];
        double m = pj[1];
        double n = pj[2];
        double u = pk[0];
        double v = pk[1];
        double w = pk[2];
        double term1 = a*(2*b*(3*c+n+w) + 2*c*(m+v) + 2*m*n + m*w + n*v + 2*v*w);
        double term2 = b*(2*c*(l+u) + l*(2*n + w) + u*(n+2*w));
        double term3 = 2*c*l*m + c*l*v + c*m*u + 2*c*u*v + 6*l*m*n;
        double term4 = 2*(l*n*v + m*n*u + n*u*v);
        double term5 = 2*w*(l*(m+v) + u*(m+3*v));
        double faceIntegral = (term1 + term2 + term3 + term4 + term5)/120;
        yzXZxyVector += 2*mesh_->getFaceArea(i)*faceIntegral*mesh_->getFaceNormal(i);
        Vector3d temp = (2.0/3.0)*mesh_->getFaceNormal(i)*mesh_->getFaceArea(i);
        xyzSquared += (temp.array()*((1.0/20.0)*( pj.array().pow(3) + pj.array().pow(2)*(pk.array() + pi.array())
                                             + pj.array()*(pk.array().pow(2) + pk.array()*pi.array() + pi.array().pow(2))
                                             + (pk.array() + pi.array())*(pk.array().pow(2) + pi.array().pow(2)) ))).matrix();


    }
    inertiaTensor(0,0) = xyzSquared[1] + xyzSquared[2];
    inertiaTensor(0,1) = -yzXZxyVector[2];
    inertiaTensor(0,2) = -yzXZxyVector[1];
    inertiaTensor(1,0) = -yzXZxyVector[2];
    inertiaTensor(1,1) = xyzSquared[0] + xyzSquared[2];
    inertiaTensor(1,2) = -yzXZxyVector[0];
    inertiaTensor(2,0) = -yzXZxyVector[1];
    inertiaTensor(2,1) = -yzXZxyVector[0];
    inertiaTensor(2,2) = xyzSquared[0] + xyzSquared[1];
}

void RigidBodyTemplate::computeEigenVectorsandValues()
{
    eigenVectors.setZero();
    eigenValues.setZero();
    EigenSolver<Matrix3d> es(inertiaTensor,true);
    Matrix3d tempEigenVectors = es.eigenvectors().real();
    Vector3d tempEigenValues = es.eigenvalues().real();
    if (tempEigenValues[0] <= tempEigenValues[1] && tempEigenValues[0] <= tempEigenValues[2])
    {
        eigenVectors.col(0) = tempEigenVectors.col(0);
        eigenValues[0] = tempEigenValues[0];
        if (tempEigenValues[1] <= tempEigenValues[2])
        {
            eigenVectors.col(1) = tempEigenVectors.col(1);
            eigenValues[1] = tempEigenValues[1];
            eigenVectors.col(2) = tempEigenVectors.col(2);
            eigenValues[2] = tempEigenValues[2];
        }
        else
        {
            eigenVectors.col(1) = tempEigenVectors.col(2);
            eigenValues[1] = tempEigenValues[2];
            eigenVectors.col(2) = tempEigenVectors.col(1);
            eigenValues[2] = tempEigenValues[1];
        }
    }
    else if (tempEigenValues[1] <= tempEigenValues[0] && tempEigenValues[1] <= tempEigenValues[2])
    {
        eigenVectors.col(0) = tempEigenVectors.col(1);
        eigenValues[0] = tempEigenValues[1];
        if (tempEigenValues[0] <= tempEigenValues[2])
        {
            eigenVectors.col(1) = tempEigenVectors.col(0);
            eigenValues[1] = tempEigenValues[0];
            eigenVectors.col(2) = tempEigenVectors.col(2);
            eigenValues[2] = tempEigenValues[2];
        }
        else
        {
            eigenVectors.col(1) = tempEigenVectors.col(2);
            eigenValues[1] = tempEigenValues[2];
            eigenVectors.col(2) = tempEigenVectors.col(0);
            eigenValues[2] = tempEigenValues[0];
        }
    }
    else if (tempEigenValues[2] <= tempEigenValues[0] && tempEigenValues[2] <= tempEigenValues[1])
    {
        eigenVectors.col(0) = tempEigenVectors.col(2);
        eigenValues[0] = tempEigenValues[2];
        if (tempEigenValues[0] <= tempEigenValues[1])
        {
            eigenVectors.col(1) = tempEigenVectors.col(0);
            eigenValues[1] = tempEigenValues[0];
            eigenVectors.col(2) = tempEigenVectors.col(1);
            eigenValues[2] = tempEigenValues[1];
        }
        else
        {
            eigenVectors.col(1) = tempEigenVectors.col(1);
            eigenValues[1] = tempEigenValues[1];
            eigenVectors.col(2) = tempEigenVectors.col(0);
            eigenValues[2] = tempEigenValues[0];
        }
    }
}

RigidBodyTemplate::~RigidBodyTemplate()
{
    delete mesh_;
}


