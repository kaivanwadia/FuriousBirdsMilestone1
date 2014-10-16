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
    cout<<"Mesh name: "<<meshFilename<<endl;
    calculateCenterOfMass();
    translateRigidBodyToOrigin();
    calculateSmallestRadius();
    scaleRigidBody();
    calculateVolume();
    cout<<"Volume : "<<vol<<endl;
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

}

RigidBodyTemplate::~RigidBodyTemplate()
{
    delete mesh_;
}


