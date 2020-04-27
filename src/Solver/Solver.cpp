#include "Solver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <eigen3/Eigen/StdVector>

#include "Converter.h"
#include <opencv2/calib3d.hpp>

#include "../DebugManager/DebugManager.h"
#include "../Tools/Timer.h"

#include "g2oEdgeSE3Self.h"

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
    Timer::Timer nTimerSolver("G2OSolver", true);
    cout << "Solver: Camera|MpPnt|VisualMeasure: " << nMapFrameID2CameraPose.size()
         << "|" <<nMapMapPointID2MapPoint3D.size() << "|" <<nVecVisualMeasurement.size() <<endl;
    //cout << "nPrTPl:" << endl << nPtrCameraStereo->getTranslateCvtPtLViewToRView().get() << endl;
        // log: #1. output the co-vis kpt number among frames
        //      #2. try to use optical flow to track KeyPoint
        //      #3. try to use sliding window(m Frames + n Key Frames) to optimize pose.
        //      #4. try to erase mappoint without enough co-vis or depth 
        //      #5. try to fix the first key frame's camera pose of input keyframes.
        //      #6. try to test the performance of inverse depth
    std::map<TpMapPointID, TpVecFrameID> nCountMp;
    for(int nIdxVm=0,nSzVm=nVecVisualMeasurement.size();nIdxVm<nSzVm;++nIdxVm){
        const TpVisualMeasurement& vm = nVecVisualMeasurement[nIdxVm];
        nCountMp[vm.mMapPointID].push_back(vm.mFrameID);
    }
    
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
#if 0
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(nPtrlinearSolver))
    );
#endif
    optimizer.setAlgorithm(solver);
    
    unsigned int nGraphNodeID = 0;
    for(auto Iter=nMapFrameID2CameraPose.begin(),EndIter=nMapFrameID2CameraPose.end();Iter!=EndIter;++Iter){
        const TpFrameID nFrameID        = Iter->first;
        TpPtrCameraPose nPtrCameraPose  = Iter->second;
        
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(cv::Mat(nPtrCameraPose->getMatx44f())));
        vSE3->setFixed(nFrameID==1 && nMapFrameID2CameraPose.size()>1);
        vSE3->setId(nFrameID);
        optimizer.addVertex(vSE3);
        assert(nFrameID>=0);
        nGraphNodeID = std::max(nGraphNodeID, (unsigned int)nFrameID);
        
        if(DebugManager::DebugControl().mBoolLogSolverVtxEdgInfo)
            cout << "Frame ID-Value: " << nFrameID << " - " << nPtrCameraPose->getPosition().t() << endl;
    }
    
    for(auto Iter=nMapMapPointID2MapPoint3D.begin(),EndIter=nMapMapPointID2MapPoint3D.end();Iter!=EndIter;++Iter){
        const TpMapPointID nMapPointID = Iter->first;
        TpMapPoint3D nMapPoint3D = *(Iter->second.get());
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(nMapPoint3D));
        const int id = nMapPointID+nGraphNodeID+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        vPoint->setFixed(DebugManager::DebugControl().mBoolMapPointFixed);
        optimizer.addVertex(vPoint);
        
        if(DebugManager::DebugControl().mBoolLogSolverVtxEdgInfo){
            auto& nVecCoVisFrame = nCountMp[nMapPointID];
            cout << "MpPnt ID-Value-Frame-Measure: " << nMapPointID << " - " << nMapPoint3D << " - " 
                 << TpSetFrameID(nVecCoVisFrame.begin(), nVecCoVisFrame.end()).size() << " - " << nVecCoVisFrame.size()
                 << endl;
        }
    }
    
    for(auto Iter=nVecVisualMeasurement.begin(),EndIter=nVecVisualMeasurement.end();Iter!=EndIter;++Iter){
        const TpVisualMeasurement& nVisualMeasurement = *Iter;
        if(!DebugManager::DebugControl().mBoolAddStereoMatchInSolver && nVisualMeasurement.mBoolRight)
            continue;
        
        const TpKeyPoint&          nKeyPoint = nVisualMeasurement.mKeyPoint;
        // undistor;
        TpKeyPoint nKeyPointUndistor = nKeyPoint;
        
        Eigen::Matrix<double,2,1> obs;
        obs << nKeyPointUndistor.pt.x, nKeyPointUndistor.pt.y;

        g2o::EdgeSE3ProjectXYZ* e = [&]()-> g2o::EdgeSE3ProjectXYZ*{
            if(nVisualMeasurement.mBoolRight){
                return  new g2o::EdgeSE3ProjectXYZRight2(Converter::toSE3Quat(cv::Mat(nPtrCameraStereo->getTranslateCvtPtLViewToRView().get())));
            }else{
                return new g2o::EdgeSE3ProjectXYZRight2(Converter::toSE3Quat(cv::Mat(cv::Matx44f::eye())));            
                //: new g2o::EdgeSE3ProjectXYZ();
                //return new g2o::EdgeSE3ProjectXYZ2();
            }
        }();
        
        e->fx = nPtrCameraStereo->getInnerParam().getfx();
        e->fy = nPtrCameraStereo->getInnerParam().getfy();
        e->cx = nPtrCameraStereo->getInnerParam().getcx();
        e->cy = nPtrCameraStereo->getInnerParam().getcy();

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
        
        if(DebugManager::DebugControl().mBoolLogSolverVtxEdgInfo){
            e->computeError();
            auto err = e->error();
            cout << "Measu ID-FrmID-MpID-Value-Error: " << nKeyPoint.class_id << (nVisualMeasurement.mBoolRight?"(R)":"L") << " - " <<nVisualMeasurement.mFrameID <<"-" << nVisualMeasurement.mMapPointID<<"-" << nKeyPointUndistor.pt <<"-"<<err.transpose()<< endl;
        }

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

    if(!DebugManager::DebugControl().mBoolMapPointFixed && DebugManager::DebugControl().mBoolUpdateMapPoint){
        //Points
        for(auto Iter=nMapMapPointID2MapPoint3D.begin(),EndIter=nMapMapPointID2MapPoint3D.end();Iter!=EndIter;++Iter){
            const TpMapPointID nMapPointID = Iter->first;
            TpMapPoint3D nMapPoint3D = *(Iter->second.get());
            TpPtrMapPoint3D nPtrMapPoint3D = Iter->second;
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(nMapPointID+nGraphNodeID+1));
            *nPtrMapPoint3D = Converter::toCvPoint3f(vPoint->estimate());
        }
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

