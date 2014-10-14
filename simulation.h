#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>
#include "simparameters.h"
#include <QGLWidget>

class RigidBodyTemplate;
class RigidBodyInstance;

typedef Eigen::Triplet<double> Tr;

class SimParameters;

class Simulation
{
public:
    Simulation(const SimParameters &params);
    ~Simulation();

    void takeSimulationStep();
    void initializeGL();

    void renderFloor();
    void renderObjects();
    void clearScene();
    void addRigidBody(Eigen::Vector3d pos, Eigen::Vector3d lookdir);

private:
    void loadFloorTexture();
    void loadRigidBodies();

    const SimParameters &params_;
    QMutex renderLock_;

    double time_;
    GLuint floorTex_;

    std::vector<RigidBodyTemplate *> templates_;
    std::vector<RigidBodyInstance *> bodies_;
};

#endif // SIMULATION_H
