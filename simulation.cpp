#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include <iostream>
#include <Eigen/Geometry>
#include <QDebug>
#include "SOIL.h"
#include "rigidbodytemplate.h"
#include "rigidbodyinstance.h"
#include "vectormath.h"
#include <Eigen/Dense>
#include "mesh.h"

const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0), floorTex_(0)
{
    loadRigidBodies();
}

Simulation::~Simulation()
{
    clearScene();
    for(vector<RigidBodyTemplate *>::iterator it = templates_.begin(); it != templates_.end(); ++it)
    {
        delete *it;
    }
}

void Simulation::initializeGL()
{
    loadFloorTexture();
}

void Simulation::loadRigidBodies()
{
    const int numobjs = 4;
    string objNames[numobjs] = {"resources/sphere.obj", "resources/2by4.obj", "resources/bunny.obj", "resources/custom.obj"};
    for(int i=0; i<numobjs; i++)
    {
        templates_.push_back(new RigidBodyTemplate(objNames[i]));
    }
}

void Simulation::loadFloorTexture()
{
    floorTex_ = SOIL_load_OGL_texture("resources/grid.jpg", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_INVERT_Y |  SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT | SOIL_FLAG_MIPMAPS);
    if(floorTex_ != 0)
    {
        glBindTexture(GL_TEXTURE_2D, floorTex_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    }
}

void Simulation::renderFloor()
{    
    glColor3f(1.0, 1.0, 1.0);
    if(floorTex_)
    {
        glBindTexture(GL_TEXTURE_2D, floorTex_);
        glEnable(GL_TEXTURE_2D);
    }
    else
        glColor3f(0.5, 0.5, 0.5);

    double texsize = 5.0;
    double gridsize = 1000.0;

    double texmax = gridsize/texsize;

    glBegin(GL_QUADS);
    {
        glTexCoord2f(texmax, texmax);
        glNormal3f(0, 0, 1.0);
        glVertex3f(gridsize,gridsize, 0);

        glTexCoord2f(texmax, -texmax);
        glNormal3f(0, 0, 1.0);
        glVertex3f(gridsize,-gridsize, 0);

        glTexCoord2f(-texmax, -texmax);
        glNormal3f(0, 0, 1.0);
        glVertex3f(-gridsize,-gridsize, 0);

        glTexCoord2f(-texmax, texmax);
        glNormal3f(0, 0, 1.0);
        glVertex3f(-gridsize,gridsize, 0);
    }
    glDisable(GL_TEXTURE_2D);
    glEnd();
}

void Simulation::renderObjects()
{
    renderLock_.lock();
    {
        for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            (*it)->render();
        }
    }
    renderLock_.unlock();
}

void Simulation::takeSimulationStep()
{
    time_ += params_.timeStep;

    for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        Vector3d z(0,0,1);
        (*it)->theta = time_*z;
    }
}

void Simulation::clearScene()
{
    renderLock_.lock();
    {
        for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
            delete *it;
        bodies_.clear();
    }
    renderLock_.unlock();
}

void Simulation::addRigidBody(Vector3d pos, Vector3d lookdir)
{
    renderLock_.lock();
    {
        Vector3d orient(0,0,0);
        Vector3d velocity, vForAngularW, angularV;
        vForAngularW.setZero();
        if (params_.randomLaunchOrientation)
        {
            orient[0] = VectorMath::randomUnitIntervalReal();
            orient[1] = VectorMath::randomUnitIntervalReal();
            orient[2] = VectorMath::randomUnitIntervalReal();
            double norm = orient.norm();
            orient = orient/norm;
        }
        if (params_.randomLaunchAngVel)
        {
            vForAngularW[0] = VectorMath::randomUnitIntervalReal();
            vForAngularW[1] = VectorMath::randomUnitIntervalReal();
            vForAngularW[2] = VectorMath::randomUnitIntervalReal();
            double norm = vForAngularW.norm();
            vForAngularW = vForAngularW/norm;
            vForAngularW = vForAngularW*params_.randomLaunchVelMagnitude;
            double orientnorm = orient.norm();
            Matrix3d Id;
            Id.setIdentity();
            Eigen::Matrix3d Rtheta = cos(orientnorm)*Id + sin(orientnorm)*VectorMath::crossProductMatrix(orient) + (1-cos(orientnorm))*orient*orient.transpose();
            angularV = Rtheta*vForAngularW;
            cout <<"Angular v:\n"<<angularV<<endl;
        }
        velocity = ((lookdir - pos)/(lookdir - pos).norm())*params_.launchVel;
//        RigidBodyInstance *newbody = new RigidBodyInstance(*templates_[params_.launchBody], pos, orient, params_.bodyDensity);
        RigidBodyInstance *newbody = new RigidBodyInstance(*templates_[params_.launchBody], pos, orient, params_.bodyDensity, velocity, angularV);
        bodies_.push_back(newbody);
    }
    renderLock_.unlock();
}
