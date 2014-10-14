#include "rigidbodytemplate.h"
#include "mesh.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

RigidBodyTemplate::RigidBodyTemplate(std::string &meshFilename)
{
    m_ = new Mesh(meshFilename);
}

RigidBodyTemplate::~RigidBodyTemplate()
{
    delete m_;
}
