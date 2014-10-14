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

    const Mesh &getMesh() const {return *m_;}

private:
    RigidBodyTemplate(const RigidBodyTemplate &other);
    RigidBodyTemplate &operator=(const RigidBodyTemplate &other);

    Mesh *m_;
};

#endif // RIGIDBODYTEMPLATE_H
