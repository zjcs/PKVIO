#ifndef __G2OEDGETSE3SELF_H__
#define __G2OEDGETSE3SELF_H__

#include "g2o/types/sba/types_six_dof_expmap.h"


namespace g2o 
{ 
class G2O_TYPES_SBA_API EdgeSE3ProjectXYZ2 : public EdgeSE3ProjectXYZ {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void computeError();
  virtual void linearizeOplus();
};
   
// Projection using focal_length in x and y directions
class G2O_TYPES_SBA_API EdgeSE3ProjectXYZRight2 : public EdgeSE3ProjectXYZ {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZRight2(const SE3Quat& nPrTPl):EdgeSE3ProjectXYZ(), mPrTPl(nPrTPl){}
  
  void computeError();

  bool isDepthPositive() {
    const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
    const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
    return ((mPrTPl*v1->estimate()).map(v2->estimate()))(2) > 0.0;
  }

  virtual void linearizeOplus();

  SE3Quat  mPrTPl;
}; 

// Projection using focal_length in x and y directions
class G2O_TYPES_SBA_API EdgeSE3ProjectXYZRight : public BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZRight(const SE3Quat& nPrTPl):BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexSE3Expmap>(), mPrTPl(nPrTPl){}
  
  bool read(std::istream &is);

  bool write(std::ostream &os) const;


  void computeError() {
    const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
    const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
    Vector2 obs(_measurement);
    SE3Quat nPrTPw = mPrTPl*v1->estimate();
    _error = obs - cam_project(nPrTPw.map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
    const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
    return ((mPrTPl*v1->estimate()).map(v2->estimate()))(2) > 0.0;
  }

  virtual void linearizeOplus();

  Vector2 cam_project(const Vector3 &trans_xyz) const;

  number_t fx, fy, cx, cy;
  SE3Quat  mPrTPl;
}; 
/*
*/

}
#endif
