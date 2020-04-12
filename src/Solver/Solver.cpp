#include "Solver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <eigen3/Eigen/StdVector>

#include "Converter.h"
#include <opencv2/calib3d.hpp>
/*
*/

namespace PKVIO {
namespace Solver {
Solver::Solver() 
{
    initialization();
}

void Solver::initialization() 
{
    
}

void Solver::initCamerPoses(const std::map< PKVIO::Type::TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose) 
{
    
}

void Solver::initMapPoints(const std::map< PKVIO::Type::TpMapPointID, PKVIO::Type::TpPtrMapPoint3D >& nMapMapPointID2MapPoint3D) 
{
    
}

void Solver::addMeasurement(const TpVisualMeasurement& nVisualMeasurement) 
{
}

void Solver::solve(std::map< Type::TpFrameID, TpPtrCameraPose>& nMapFrameID2CameraPose,
                   std::map<Type::TpMapPointID, Type::TpPtrMapPoint3D >& nMapMapPointID2MapPoint3D,
                   const TpVecVisualMeasurement& nVecVisualMeasurement, const TpPtrCameraStereo nPtrCameraStereo) 
{
    int     nIterations = 10;
    bool    bRobust     = true;
    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    bool    bStopFlag   = false;
    bool*   pbStopFlag  = &bStopFlag;
    vector<bool> vbNotIncludedMP;
    //vbNotIncludedMP.resize(vpMP.size());
    
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> nPtrlinearSolver 
        = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(nPtrlinearSolver))
    );
    optimizer.setAlgorithm(solver);
    
    unsigned int nGraphNodeID = 0;
    for(auto Iter=nMapFrameID2CameraPose.begin(),EndIter=nMapFrameID2CameraPose.end();Iter!=EndIter;++Iter){
        const TpFrameID nFrameID        = Iter->first;
        TpPtrCameraPose nPtrCameraPose  = Iter->second;
        
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(cv::Mat(nPtrCameraPose->getMatx44f())));
        vSE3->setFixed(nFrameID==0);
        vSE3->setId(nFrameID);
        optimizer.addVertex(vSE3);
        assert(nFrameID>=0);
        nGraphNodeID = std::max(nGraphNodeID, (unsigned int)nFrameID);
    }
    
    for(auto Iter=nMapMapPointID2MapPoint3D.begin(),EndIter=nMapMapPointID2MapPoint3D.end();Iter!=EndIter;++Iter){
        const TpMapPointID nMapPointID = Iter->first;
        TpMapPoint3D nMapPoint3D = *(Iter->second.get());
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(nMapPoint3D));
        const int id = nMapPointID+nGraphNodeID+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
    }
    
    for(auto Iter=nVecVisualMeasurement.begin(),EndIter=nVecVisualMeasurement.end();Iter!=EndIter;++Iter){
        const TpVisualMeasurement& nVisualMeasurement = *Iter;
        const TpKeyPoint&          nKeyPoint = nVisualMeasurement.mKeyPoint;
        // undistor;
        TpKeyPoint nKeyPointUndistor = nKeyPoint;
        
        Eigen::Matrix<double,2,1> obs;
        obs << nKeyPointUndistor.pt.x, nKeyPointUndistor.pt.y;
        

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nVisualMeasurement.mMapPointID+nGraphNodeID+1)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nVisualMeasurement.mFrameID)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

        if(bRobust)
        {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
        }

        e->fx = nPtrCameraStereo->getInnerParam().getfx();
        e->fy = nPtrCameraStereo->getInnerParam().getfy();
        e->cx = nPtrCameraStereo->getInnerParam().getcx();
        e->cy = nPtrCameraStereo->getInnerParam().getcy();

        optimizer.addEdge(e);
    }
    
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);
    
    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    
    // Recover optimized data
    
    //Keyframes
    for(auto Iter=nMapFrameID2CameraPose.begin(),EndIter=nMapFrameID2CameraPose.end();Iter!=EndIter;++Iter){
        const TpFrameID nFrameID        = Iter->first;
        TpPtrCameraPose nPtrCameraPose  = Iter->second;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(nFrameID));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        nPtrCameraPose->setMatx44f(cv::Matx44f(Converter::toCvMat(SE3quat)));
    }

    //Points
    for(auto Iter=nMapMapPointID2MapPoint3D.begin(),EndIter=nMapMapPointID2MapPoint3D.end();Iter!=EndIter;++Iter){
        const TpMapPointID nMapPointID = Iter->first;
        TpMapPoint3D nMapPoint3D = *(Iter->second.get());
        TpPtrMapPoint3D nPtrMapPoint3D = Iter->second;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(nMapPointID+nGraphNodeID+1));
        *nPtrMapPoint3D = Converter::toCvPoint3f(vPoint->estimate());
    }
    
}


void Solver::solveBySimulator(cv::Mat nInnerMat, TpPtrCameraStereo nPtrCameraStereo) 
{
    std::map< Type::TpFrameID, TpPtrCameraPose> nMapFrameID2CameraPose;
    std::map<Type::TpMapPointID, Type::TpPtrMapPoint3D > nMapMapPointID2MapPoint3D;
    TpVecVisualMeasurement nVecVisualMeasurement;
    //TpPtrCameraStereo nPtrCameraStereo;
    
    {
        typedef cv::Vec3d TpDataPt;
        static int nSzIteration = 1; ++nSzIteration;
        std::vector<cv::Vec3d> nTranslation = {TpDataPt(0,0,0), TpDataPt(100,0,0), TpDataPt(100, 100, 0), TpDataPt(0, 100, 0)};
        std::vector<cv::Vec3d> nVecMapPt3D = {
            TpDataPt(0,0,100),TpDataPt(100,0,100),TpDataPt(100,100,100),TpDataPt(0,100,100),
            TpDataPt(20,20,200),TpDataPt(80,20,200),TpDataPt(80,80,200),TpDataPt(20,80,200),
        };
        
        nMapMapPointID2MapPoint3D.clear();
        nVecVisualMeasurement.clear();
        nMapFrameID2CameraPose.clear();
        for(int nIdxMp=0,nSzMp=nVecMapPt3D.size();nIdxMp<nSzMp;++nIdxMp)
            nMapMapPointID2MapPoint3D[nIdxMp] = std::make_shared<cv::Point3f>(cv::Point3f(nVecMapPt3D[nIdxMp]));
        
        //cv::Mat nInnerMat = mPtrDataset->getPtrCamera()->CameraLeft().getInnerParam().getInnerMat64F();
        for(int nIdxIter=0;nIdxIter<nSzIteration;++nIdxIter){
            cv::Vec3d nTranslat = nTranslation[nIdxIter%(int)nTranslation.size()];
            cv::Matx44f nCamera = Type::cvtR33T31ToMatx44f(cv::Mat(cv::Matx33f::eye()), cv::Mat(cv::Matx31f(nTranslat)));
            std::vector<cv::Vec2d> nVecVm;
            cv::projectPoints(nVecMapPt3D, TpDataPt(), nTranslat, nInnerMat, cv::Mat(), nVecVm);
            nMapFrameID2CameraPose[nIdxIter] = std::make_shared<TpCameraPose>(TpCameraPose(nCamera));

            for(int nIdxMp=0,nSzMp=nVecMapPt3D.size();nIdxMp<nSzMp;++nIdxMp){
                TpVisualMeasurement nVm;
                nVm.mFrameID = nIdxIter;
                nVm.mKeyPoint = cv::KeyPoint(cv::Point2f(nVecVm[nIdxMp]), 1);
                nVm.mMapPointID = nIdxMp;
                nVecVisualMeasurement.push_back(nVm);
            }
        }
    }   
    
    solve(nMapFrameID2CameraPose, nMapMapPointID2MapPoint3D, nVecVisualMeasurement, nPtrCameraStereo);
}

}
}

