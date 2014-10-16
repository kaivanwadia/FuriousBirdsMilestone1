#ifndef RIGIDBODYTEMPLATE_H
#define RIGIDBODYTEMPLATE_H

#include <string>
#include <Eigen/Core>

class Mesh;

class RigidBodyTemplate
{
public:
    RigidBodyTemplate(std::string &meshFilename);
    ~RigidBodyTemplate();

    double vol;
    Eigen::Vector3d centerOfMass;

    const Mesh &getMesh() const {return *mesh_;}

private:
    RigidBodyTemplate(const RigidBodyTemplate &other);
    RigidBodyTemplate &operator=(const RigidBodyTemplate &other);
    void calculateVolume();
    void calculateCenterOfMass();

    Mesh *mesh_;
};

#endif // RIGIDBODYTEMPLATE_H
