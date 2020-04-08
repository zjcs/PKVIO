#include "Solver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h""
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <eigen3/Eigen/StdVector>

#include "Converter.h"
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
                   const TpVecVisualMeasurement& nVecVisualMeasurement) 
{
    return;
    // need read camera calib and undistor keypoint function;
    throw;
    
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
        throw;
        
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

        /*
        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        */
        throw;

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
    
    /*

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
    */

}

}
}

