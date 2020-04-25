#include "g2oEdgeSE3Self.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o 
{
using namespace std;
using namespace Eigen;

//G2O_REGISTER_TYPE_GROUP(expmap2);
//G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP2, EdgeSE3ProjectXYZRight2);
//G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP2, EdgeSE3ProjectXYZRight);
//EdgeSE3ProjectXYZ2

Vector2 project2d(const Vector3& v)  {
  Vector2 res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

void EdgeSE3ProjectXYZ2::computeError() {
    EdgeSE3ProjectXYZ::computeError();
    //cout << _error << endl;
}


void EdgeSE3ProjectXYZ2::linearizeOplus() {
    EdgeSE3ProjectXYZ::linearizeOplus();
    //cout << _jacobianOplusXj <<endl;
}


void EdgeSE3ProjectXYZRight2::computeError() {
    const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
    const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
    Vector2 obs(_measurement);
    SE3Quat nPrTPw = mPrTPl*v1->estimate();
    _error = obs - cam_project(nPrTPw.map(v2->estimate()));
    //cout << _error <<endl;
}


void EdgeSE3ProjectXYZRight2::linearizeOplus() {
    VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());
    VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
    Vector3 xyz = vi->estimate();
    Vector3 xyz_cl = T.map(xyz);
    Vector3 xyz_cr = mPrTPl.map(xyz_cl);
    
    Vector3 xyz_trans = xyz_cr;

    number_t x = xyz_trans[0];
    number_t y = xyz_trans[1];
    number_t z = xyz_trans[2];
    number_t z_2 = z * z;

    Matrix<number_t, 2, 3> tmp;
    tmp(0, 0) = fx;
    tmp(0, 1) = 0;
    tmp(0, 2) = -x / z * fx;

    tmp(1, 0) = 0;
    tmp(1, 1) = fy;
    tmp(1, 2) = -y / z * fy;

    _jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();
    
    Matrix<number_t, 3, 6> tmp2;
    tmp2.topLeftCorner(3,3) <<  0,      xyz_cl(2),  -xyz_cl(1), 
                            -xyz_cl(2),     0,      xyz_cl(0),
                            xyz_cl(1),  -xyz_cl(0),     0;
    tmp2.topRightCorner(3,3) = Matrix<number_t, 3, 3>::Identity();
    _jacobianOplusXj = -1./z * tmp * mPrTPl.rotation().toRotationMatrix() * tmp2;
    //cout << xyz_cl<<endl;
    //cout << _jacobianOplusXj <<endl;
}

Vector2 EdgeSE3ProjectXYZRight::cam_project(const Vector3& trans_xyz) const {
    Vector2 proj = project2d(trans_xyz);
    Vector2 res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
}


bool EdgeSE3ProjectXYZRight::write(std::ostream& os) const {

    for (int i = 0; i < 2; i++) {
        os << measurement()[i] << " ";
    }

    for (int i = 0; i < 2; i++)
        for (int j = i; j < 2; j++) {
            os << " " << information()(i, j);
        }
    return os.good();
}


bool EdgeSE3ProjectXYZRight::read(std::istream& is) {
    for (int i = 0; i < 2; i++) {
        is >> _measurement[i];
    }
    for (int i = 0; i < 2; i++)
        for (int j = i; j < 2; j++) {
            is >> information()(i, j);
            if (i != j)
                information()(j, i) = information()(i, j);
        }
    return true;
}


void EdgeSE3ProjectXYZRight::linearizeOplus() {
    VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());
    VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
    Vector3 xyz = vi->estimate();
    Vector3 xyz_cl = T.map(xyz);
    Vector3 xyz_cr = mPrTPl.map(xyz_cl);
    
    Vector3 xyz_trans = xyz_cr;

    number_t x = xyz_trans[0];
    number_t y = xyz_trans[1];
    number_t z = xyz_trans[2];
    number_t z_2 = z * z;

    Matrix<number_t, 2, 3> tmp;
    tmp(0, 0) = fx;
    tmp(0, 1) = 0;
    tmp(0, 2) = -x / z * fx;

    tmp(1, 0) = 0;
    tmp(1, 1) = fy;
    tmp(1, 2) = -y / z * fy;

    _jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();
    
    Matrix<number_t, 3, 6> tmp2;
    tmp2.topLeftCorner(3,3) <<  0,      xyz_cl(2),  -xyz_cl(1), 
                            -xyz_cl(2),     0,      xyz_cl(0),
                            xyz_cl(1),  -xyz_cl(0),     0;
    tmp2.topRightCorner(3,3) = Matrix<number_t, 3, 3>::Identity();
    _jacobianOplusXj = -1./z * tmp * mPrTPl.rotation().toRotationMatrix() * tmp2;

}
/*
*/



}
