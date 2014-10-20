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
class Mesh;

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

    Eigen::Matrix3d computeTMatrix(Eigen::Vector3d vec);
    Eigen::Matrix3d computeD1ofOmega(Eigen::Vector3d omega, Eigen::Vector3d firstTerm);
    Eigen::Matrix3d computeD2ofOmega(Eigen::Vector3d omega, Eigen::Vector3d secondTerm);
    Eigen::Matrix3d computeBMatrix(Eigen::Vector3d thetaI, Eigen::Vector3d omega);

    Eigen::Vector3d computeDiffVwrtC(Eigen::Vector3d cOfMass, Eigen::Vector3d theta, Mesh mesh, double density, double vol);
    Eigen::Vector3d computeDiffVwrtTheta(Eigen::Vector3d cOfMass, Eigen::Vector3d theta, Mesh mesh);


private:
    void loadFloorTexture();
    void loadRigidBodies();

    const SimParameters &params_;
    QMutex renderLock_;

    double time_;
    GLuint floorTex_;
    std::vector<RigidBodyTemplate *> templates_;
    std::vector<RigidBodyInstance *> bodies_;
    std::vector<RigidBodyInstance *> existingBodies_;
    bool first;
};

#endif // SIMULATION_H
