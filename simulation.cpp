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

    renderLock_.lock();
    {
        for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            Vector3d newCOfMass, newTheta, newVelocity, newOmega;
            newCOfMass.setZero();
            newTheta.setZero();
            newVelocity.setZero();
            newOmega.setZero();
            cout<<"\nOld C : \n"<<(*it)->c<<endl;
            cout<<"\nOld C vel : \n"<<(*it)->cvel<<endl;
            cout<<"\nOld theta : \n"<<(*it)->theta<<endl;
            cout<<"\nOld Omega : \n"<<(*it)->w<<endl;
            newCOfMass = (*it)->c + params_.timeStep*(*it)->cvel;
            newTheta = VectorMath::axisAngle(VectorMath::rotationMatrix(params_.timeStep*(*it)->w)*VectorMath::rotationMatrix((*it)->theta));
            newVelocity = (*it)->cvel - (params_.timeStep/((*it)->density * (*it)->getTemplate().vol))*computeDiffVwrtC(newCOfMass, newTheta);
            newOmega = (*it)->w;
            Vector3d constant = params_.timeStep*((*it)->density)*((*it)->w.transpose())*VectorMath::rotationMatrix(-1*(*it)->theta).transpose()
                    *((*it)->getTemplate().inertiaTensor)*VectorMath::rotationMatrix(-1*(*it)->theta)*computeD2ofOmega((*it)->w, newTheta);
//            constant += add potential stuff (should be negative)
            cout<<"\nNEWTON METHOD BEGINS : "<<endl;
            int i;
            for(i = 0; i<params_.NewtonMaxIters; i++)
            {
                Vector3d fOfOmega;
                fOfOmega.setZero();
                cout<<"\nPart 1 :\n"<<params_.timeStep*((*it)->density)*(newOmega.transpose())*VectorMath::rotationMatrix(-1*newTheta).transpose()
                      *((*it)->getTemplate().inertiaTensor)*VectorMath::rotationMatrix(-1*newTheta)*computeD1ofOmega(newOmega, newTheta)<<endl;
                fOfOmega = constant.transpose() + params_.timeStep*((*it)->density)*(newOmega.transpose())*VectorMath::rotationMatrix(-1*newTheta).transpose()
                        *((*it)->getTemplate().inertiaTensor)*VectorMath::rotationMatrix(-1*newTheta)*computeD1ofOmega(newOmega, newTheta);
                cout<<"F Of Omega Temp: \n"<<fOfOmega<<endl;
                fOfOmega += params_.timeStep*((*it)->density)*(newOmega.transpose())*VectorMath::rotationMatrix(-1*newTheta).transpose()
                        *((*it)->getTemplate().inertiaTensor)*computeBMatrix(-1*newTheta, newOmega);
                cout<<"F Of Omega Final: \n"<<fOfOmega<<endl;
                if (fOfOmega.norm() < params_.NewtonTolerance)
                {
                    break;
                }
                Matrix3d gradF = params_.timeStep*((*it)->density)*VectorMath::rotationMatrix(-1*newTheta)*((*it)->getTemplate().inertiaTensor)
                        *VectorMath::rotationMatrix(-1*newTheta)*computeD1ofOmega((*it)->w, newTheta);
                cout<<"Grad F : "<<gradF<<endl;
                VectorXd deltaOmega = -1*gradF.inverse()*fOfOmega;
                cout<<"Delta Omega : "<<deltaOmega<<endl;
                newOmega += deltaOmega;
                cout<<"Temp new Omega:\n"<<newOmega<<endl;
            }
            (*it)->c = newCOfMass;
            (*it)->theta = newTheta;
            (*it)->cvel = newVelocity;
            (*it)->w = newOmega;
            cout<<"\n NEWTON METHOD ENDS"<<endl;
            if (i>0)
            {
                cout<<"\nNewton Iters: "<<i<<endl;
            }
            cout<<"\nNew C : \n"<<newCOfMass<<endl;
            cout<<"\nNew C vel : \n"<<newTheta<<endl;
            cout<<"\nNew theta : \n"<<newVelocity<<endl;
            cout<<"\nNew Omega : \n"<<(*it)->w<<endl;
        }
    }
    renderLock_.unlock();
}

Matrix3d Simulation::computeBMatrix(Vector3d thetaI, Vector3d omega)
{
    Matrix3d bMatrix;
    bMatrix = -1*VectorMath::rotationMatrix(thetaI)*VectorMath::crossProductMatrix(omega)*computeTMatrix(thetaI);
    return bMatrix;
}

Matrix3d Simulation::computeD2ofOmega(Vector3d omega, Vector3d secondTerm)
{
    Matrix3d d2Omega;
    d2Omega = (1/params_.timeStep)*computeTMatrix(-params_.timeStep*omega).inverse()*computeTMatrix(-1*secondTerm);
    return d2Omega;
}

Matrix3d Simulation::computeD1ofOmega(Vector3d omega, Vector3d firstTerm)
{
    Matrix3d d1Omega;
    d1Omega = (-1/params_.timeStep)*computeTMatrix(params_.timeStep*omega).inverse()*computeTMatrix(-1*firstTerm);
    return d1Omega;
}

Matrix3d Simulation::computeTMatrix(Vector3d vec)
{
    Matrix3d tMatrix;
    if (vec.norm() <= 0)
    {
        tMatrix.setIdentity();
        return tMatrix;
    }
    tMatrix.setZero();
    Matrix3d identity;
    identity.setIdentity();
    tMatrix = (vec*vec.transpose() + (VectorMath::rotationMatrix(-1*vec) - identity)*VectorMath::crossProductMatrix(vec));
    tMatrix = tMatrix/vec.squaredNorm();
    return tMatrix;
}

Vector3d Simulation::computeDiffVwrtC(Vector3d cOfM, Vector3d theta)
{
    Vector3d gradV;
    gradV.setZero();
    return gradV;
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
        }
        velocity = lookdir*params_.launchVel;
//        RigidBodyInstance *newbody = new RigidBodyInstance(*templates_[params_.launchBody], pos, orient, params_.bodyDensity);
        RigidBodyInstance *newbody = new RigidBodyInstance(*templates_[params_.launchBody], pos, orient, params_.bodyDensity, velocity, angularV);
        bodies_.push_back(newbody);
    }
    renderLock_.unlock();
}
