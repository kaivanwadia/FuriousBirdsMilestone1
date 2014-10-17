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
    double radius;
    Eigen::Vector3d centerOfMass;
    Eigen::Matrix3d inertiaTensor;
    Eigen::Matrix3d eigenVectors;
    Eigen::Vector3d eigenValues;

    const Mesh &getMesh() const {return *mesh_;}

private:
    RigidBodyTemplate(const RigidBodyTemplate &other);
    RigidBodyTemplate &operator=(const RigidBodyTemplate &other);
    void calculateVolume();
    void calculateCenterOfMass();
    void translateRigidBodyToOrigin();
    void calculateSmallestRadius();
    void scaleRigidBody();
    void calculateInertiaTensorMatrix();
    void computeEigenVectorsandValues();

    Mesh *mesh_;
};

#endif // RIGIDBODYTEMPLATE_H
