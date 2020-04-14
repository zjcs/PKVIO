#include "Tools.h"


namespace PKVIO{
namespace Tools{
    
bool triangulation(const cv::Point2f& Pl2D, const cv::Point2f& Pr2D, const float fx, const float& nBaseline, float& nDepthInLeftView)
{
    float nErrorParallax = -(Pr2D.x - Pl2D.x);  // parallax is negative, converted to positive here.
    //bool  bSuccess = nErrorParallax > 0.5;
    if(nErrorParallax < 0.5)   // filter the far kpt
        return false;
    nDepthInLeftView = fx * nBaseline / nErrorParallax;
    return true;
}

bool triangulation(const cv::Point2f& Pl2D, const cv::Point2f& Pr2D, const cv::Matx44f& nPrTPl, cv::Vec3f& Pl3D)
{
    cv::Vec4f PlxyL(Pl2D.x, Pl2D.y, 0, 1);
    cv::Vec4f PlxyR = nPrTPl * PlxyL;
    cv::Vec3f zAxis = Type::cvtMatx44fToZAxis(nPrTPl);
    
    float p1 = (zAxis(0)-zAxis(2)*Pr2D.x);
    float p2 = (zAxis(1)-zAxis(2)*Pr2D.y);
    cout << "["<< p1 << "," << p2<<"] ";
    if(std::abs(p1)<1e-1 || std::abs(p2)<1e-1){
        return false;
    }
    
    float d0 =  (Pr2D.x*PlxyR(2)-PlxyR(0))/p1;
    float d1 =  (Pr2D.y*PlxyR(2)-PlxyR(1))/p2;
    
    float z = (d0+d1)/2;
    Pl3D = cv::Vec3f(Pl2D.x, Pl2D.y, z);
    
    cv::Vec3f PlR = Type::project(nPrTPl, Pl3D);
    cv::Vec2f e = cv::Vec2f(PlR(0),PlR(1))/PlR(2) - cv::Vec2f(Pr2D.x, Pr2D.y);
    float de = e.dot(e);
    cout << "->e|de:" << e <<"|" << de <<";"<<endl;
    return true;
}

}
}
