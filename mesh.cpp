#include "mesh.h"
#include "fstream"
#include <vector>
#include <iostream>
#include <cassert>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

Mesh::Mesh() : numverts_(0), numfaces_(0) {};

Mesh::Mesh(const std::string &filename) : numverts_(0), numfaces_(0)
{
    vector<double> coords;
    vector<int> faces;

    ifstream ifs(filename.c_str());

    while(ifs)
    {
        string s;
        ifs >> s;
        if(s == "v")
        {
            numverts_++;
            for(int j=0; j<3; j++)
            {
                double dummy;
                ifs >> dummy;
                coords.push_back(dummy);
            }
        }
        else if(s == "f")
        {
            numfaces_++;
            for(int j=0; j<3; j++)
            {
                int face;
                ifs >> face;
                char dummy;
                ifs >> dummy;
                ifs >> dummy;
                int fnorm;
                ifs >> fnorm;
                faces.push_back(face-1);
            }
        }
    }

    assert((int)coords.size() == 3*numverts_);
    assert((int)faces.size() == 3*numfaces_);

    for(size_t i = 0; i<coords.size(); i+=3)
        verts_.push_back(Vector3d(coords[i], coords[i+1], coords[i+2]));

    for(size_t i =0; i < faces.size(); i+=3)
    {
        faces_.push_back(Vector3i(faces[i],faces[i+1],faces[i+2]));
    }

    computeNormals();
}

const Vector3d Mesh::getVert(int idx) const
{
    return verts_[idx];
}

const Vector3i Mesh::getFace(int idx) const
{
    return faces_[idx];
}

const Vector3d Mesh::getVertNormal(int idx) const
{
    return vertNormals_[idx];
}

const Vector3d Mesh::getFaceNormal(int idx) const
{
    return faceNormals_[idx];
}

double Mesh::getFaceArea(int idx) const
{
    return faceAreas_[idx];
}

void Mesh::computeNormals()
{
    faceNormals_.clear();
    vertNormals_.clear();
    faceAreas_.clear();

    vector<double> verttotalarea;

    Vector3d zero(0,0,0);

    for(size_t i=0; i < verts_.size(); i++)
    {
        vertNormals_.push_back(zero);
        verttotalarea.push_back(0);
    }

    for(size_t i=0; i < faces_.size(); i++)
    {
        Vector3i fverts = faces_[i];
        Vector3d pts[3];
        for(int j=0; j<3; j++)
            pts[j] = verts_[fverts[j]];

        Vector3d normal = (pts[1]-pts[0]).cross(pts[2]-pts[0]);
        double norm = normal.norm();
        faceAreas_.push_back(norm/2.0);
        faceNormals_.push_back(normal/norm);
        for(int j=0; j<3; j++)
        {
            vertNormals_[fverts[j]] += normal;
            verttotalarea[fverts[j]] += norm;
        }
    }

    for(size_t i=0; i<verts_.size(); i++)
        vertNormals_[i] /= verttotalarea[i];
}

void Mesh::translate(const Vector3d &vec)
{
    for(size_t i=0; i<verts_.size(); i++)
        verts_[i] += vec;
}

void Mesh::scale(double s)
{
    for(size_t i=0; i<verts_.size(); i++)
        verts_[i] *= s;
    for(size_t i=0; i<faces_.size(); i++)
        faceAreas_[i] *= s*s;
}
