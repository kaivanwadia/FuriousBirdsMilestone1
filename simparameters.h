#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    const static int F_GRAVITY = 1;
    const static int F_FLOOR   = 2;

    const static int R_SPHERE  = 0;
    const static int R_2BY4    = 1;
    const static int R_BUNNY   = 3;
    const static int R_CUSTOM  = 2;

    bool simRunning;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;

    int activeForces;
    double gravityG;
    double floorStiffness;
    bool gameMode;

    double bodyDensity;
    int launchBody;
    double launchVel;
    bool randomLaunchOrientation;

    bool randomLaunchAngVel;
    double randomLaunchVelMagnitude;
};

#endif // SIMPARAMETERS_H
