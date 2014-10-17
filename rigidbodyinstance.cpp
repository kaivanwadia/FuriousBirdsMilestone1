#include "rigidbodyinstance.h"
#include "vectormath.h"
#include <QGLWidget>
#include "rigidbodytemplate.h"
#include "mesh.h"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;
using namespace std;

RigidBodyInstance::RigidBodyInstance(const RigidBodyTemplate &rbtemplate, const Eigen::Vector3d &c, const Eigen::Vector3d &theta, double density) : c(c), theta(theta), density(density), rbtemplate_(rbtemplate)
{
    cvel.setZero();
    w.setZero();
}

RigidBodyInstance::RigidBodyInstance(const RigidBodyTemplate &rbtemplate, const Vector3d &c, const Vector3d &theta, double density, Vector3d cvel, Vector3d w) : c(c), theta(theta), density(density), rbtemplate_(rbtemplate), cvel(cvel), w(w)
{
}

void RigidBodyInstance::render()
{
    Matrix3d rot = VectorMath::rotationMatrix(theta);

    glShadeModel(GL_SMOOTH);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );
    Vector3d color(0.6, 0.9, 0.9);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    static vector<GLfloat> colors;
    static vector<int> indices;
    static vector<GLfloat> pos;
    static vector<GLfloat> normals;
    colors.clear();
    indices.clear();
    pos.clear();
    normals.clear();

    int nfaces = rbtemplate_.getMesh().getNumFaces();
    int npts = rbtemplate_.getMesh().getNumVerts();
    for(int i=0; i<npts; i++)
    {
        Vector3d diff = rbtemplate_.getMesh().getVert(i);
        Vector3d worldpos = c + rot*diff;
        Vector3d normal = rot*rbtemplate_.getMesh().getVertNormal(i);
        for(int j=0; j<3; j++)
        {
            pos.push_back(worldpos[j]);
            normals.push_back(normal[j]);
            colors.push_back(color[j]);
        }
    }

    glVertexPointer(3, GL_FLOAT, 0, &pos[0]);
    glNormalPointer(GL_FLOAT, 0, &normals[0]);
    glColorPointer(3, GL_FLOAT, 0, &colors[0]);

    for(int i=0; i<nfaces; i++)
    {
        Vector3i verts = rbtemplate_.getMesh().getFace(i);
        for(int j=0; j<3; j++)
            indices.push_back(verts[j]);
    }
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, &indices[0]);
    Vector3d endpos = rbtemplate_.centerOfMass + (rbtemplate_.eigenVectors.col(0))*1.1;
    endpos = c + rot*endpos;
    glBegin(GL_LINES);
    {
        glColor3f(1, 0, 0);
        glVertex3f(c[0], c[1], c[2]);
        glVertex3f(endpos[0], endpos[1], endpos[2]);
    }
    glEnd();
    endpos = rbtemplate_.centerOfMass + (rbtemplate_.eigenVectors.col(2))*1.1;
    endpos = c + rot*endpos;
    glBegin(GL_LINES);
    {
        glColor3f(0, 0, 1);
        glVertex3f(c[0], c[1], c[2]);
        glVertex3f(endpos[0], endpos[1], endpos[2]);
    }
    glEnd();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}
