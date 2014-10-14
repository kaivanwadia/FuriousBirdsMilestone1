#ifndef MESH_H
#define MESH_H

#include <string>
#include <Eigen/Core>
#include <vector>

class Mesh
{
public:
    Mesh();
    Mesh(const std::string &filename);

    int getNumVerts() const {return numverts_;}
    int getNumFaces() const {return numfaces_;}

    const Eigen::Vector3d getVert(int idx) const;
    const Eigen::Vector3d getVertNormal(int idx) const;
    const Eigen::Vector3d getFaceNormal(int idx) const;
    const Eigen::Vector3i getFace(int idx) const;
    double getFaceArea(int idx) const;

    void translate(const Eigen::Vector3d &vec);
    void scale(double s);

private:
    void computeNormals();

    int numverts_;
    int numfaces_;
    std::vector<Eigen::Vector3d> verts_;
    std::vector<Eigen::Vector3i> faces_;
    std::vector<Eigen::Vector3d> faceNormals_;
    std::vector<Eigen::Vector3d> vertNormals_;
    std::vector<double> faceAreas_;
};

#endif // MESH_H
