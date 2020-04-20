#include "Tools.h"
//#include <opencv2/core.hpp>


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
    //cout << "["<< p1 << "," << p2<<"] ";
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
    //cout << "->e|de:" << e <<"|" << de <<";"<<endl;
    return true;
}

bool triangulation(const cv::Matx33f& Pl, const cv::Matx44f& Tl, const cv::Point2f& Ptl,
                   const cv::Matx33f& Pr, const cv::Matx44f& Tr, const cv::Point2f& Ptr, cv::Vec3f& Ptw)
{
    bool blog = false;
    if(blog) cout << Pl << endl << Tl << endl << Pr << endl << Tr<<endl;
    cv::Matx44f A;
    
    cv::Vec3f NPl = Pl.inv()*cv::Vec3f(Ptl.x, Ptl.y, 1);
    if(blog) cout << NPl << endl;
    cv::Matx14f ARow0 = NPl(0)*Tl.row(2) - Tl.row(0);
    cv::Matx14f ARow1  = NPl(1)*Tl.row(2) - Tl.row(1);
    
    cv::Vec3f NPr = Pr.inv()*cv::Vec3f(Ptr.x, Ptr.y, 1);
    if(blog) cout << NPr << endl;
    cv::Matx14f ARow2 = NPr(0)*Tr.row(2) - Tr.row(0);
    cv::Matx14f ARow3 = NPr(1)*Tr.row(2) - Tr.row(1);
    
    for(int nIdx=0;nIdx<4;++nIdx){
        A(0, nIdx) = ARow0(nIdx);
        A(1, nIdx) = ARow1(nIdx);
        A(2, nIdx) = ARow2(nIdx);
        A(3, nIdx) = ARow3(nIdx);
    }
    if(blog) cout << A << endl;
    
    cv::Mat w,u,vt;
    cv::SVD::compute(cv::Mat(A),w,u,vt,cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
    
    cv::Vec4f Ptw4D = vt.row(3);
    if(blog) cout << Ptw4D << endl;
    Ptw4D/=Ptw4D(3);
    if(blog) cout << Ptw4D << endl;
    Ptw = cv::Vec3f(Ptw4D(0),Ptw4D(1),Ptw4D(2));
    //return true;
    return Ptw(2)<=10e3 && Ptw(2)>=0.5e3; // depth range [0.5,10]m;
}

bool triangulation(const cv::Matx33f& Pl, const cv::Point2f& Ptl,const cv::Matx33f& Pr, const cv::Point2f& Ptr, const cv::Matx44f& PrTPl, cv::Vec3f& PtInCl){
    cv::Vec3f NPl = Pl.inv()*cv::Vec3f(Ptl.x, Ptl.y, 1);
    cv::Vec3f NPr = Pr.inv()*cv::Vec3f(Ptr.x, Ptr.y, 1);
    //cout << "PrTPl:" <<endl << PrTPl <<endl;
    cv::Matx33f R = PrTPl.get_minor<3,3>(0,0);
    //cv::Matx31f m = PrTPl.get_minor<3,1>(0,3);
    cv::Vec3f   t = (cv::Mat)PrTPl.get_minor<3,1>(0,3);
    //cout << "R:" << endl << R <<endl;
    //cout << "m:" << endl << m <<endl;
    //cout << "t:" << endl << t.t() <<endl;
    cv::Vec3f b = NPr - R*NPl;
    //cout << "b:" << b.t() <<endl;
    float depth = ((b.t()*b).inv()*(b.t()*t))(0);
    PtInCl = NPl*depth;
    //cout << "PtInCl:" << PtInCl <<endl;
    return depth<=10e3 && depth>=0.1e3; // depth range [0.5,10]m;
}

bool triangulation(const cv::Matx33f& Pl, const cv::Point2f& Ptl,const cv::Matx33f& Pr, const cv::Point2f& Ptr, const cv::Matx44f& PrTPl, const cv::Matx44f& PlTPtw, cv::Vec3f& Ptw){
    bool b = triangulation(Pl,Ptl,Pr,Ptr,PrTPl,Ptw);
    if(b){ Ptw = Type::project(PlTPtw.inv(), Ptw); }
    return b;
}

}
}
