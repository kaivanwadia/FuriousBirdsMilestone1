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
    cout<<"Volume : "<<vol<<endl;
    calculateCenterOfMass();
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
        double norm = ((pj-pi).cross(pk-pi)).norm();
        Vector3d temp = ((pi+pj+pk)/6)*norm;
        faceArea = temp.dot(mesh_->getFaceNormal(i));
        vol += faceArea;
    }
    vol = vol/3;
}

void RigidBodyTemplate::calculateCenterOfMass()
{

}

RigidBodyTemplate::~RigidBodyTemplate()
{
    delete mesh_;
}


