#include "../include/pcm_graph.h"

using namespace std;

//Constructor.
ConnectionGraph::ConnectionGraph(int nAgentNum){
    this->m_nAgentNum = nAgentNum;
    this->m_nGamma = 5;
}



void ConnectionGraph::AddConnection(MapConnection * pConnection){
    //Adjust the number of agents.

    this->m_gConnections.push_back(pConnection);
}



void ConnectionGraph::ClearGraph(){
    this->m_gConnections.clear();
}

void ConnectionGraph::ComputeGraph(){

    string aFilePath = "/home/kyrie/Documents/DataSet/Connections/connection.txt";

    ofstream fOutFile(aFilePath);

    


    int nConnectionSize = this->m_gConnections.size();

    vector<MapConnection *> gResultConnections;
    gResultConnections.reserve(nConnectionSize);

    //Split with the connected agents.
    vector<vector<MapConnection *>> gSplitedConnections;
    int nTotalSet = this->m_nAgentNum * this->m_nAgentNum;
    gSplitedConnections.reserve(nTotalSet);
    for (int i=0;i<nTotalSet;i++){
        vector<MapConnection *> gLocalConnections;
        gLocalConnections.reserve(nConnectionSize);
        gSplitedConnections.push_back(gLocalConnections);
    }

    for (auto pConnection : this->m_gConnections){
        int nClientID1 = pConnection->m_pKeyFrame1->m_nClientID;
        int nClientID2 = pConnection->m_pKeyFrame2->m_nClientID; 
        gSplitedConnections[(nClientID1-1) * this->m_nAgentNum + nClientID2-1].push_back(pConnection);
    }

    // this->m_gConnections = gConnections;
    for (int i = 0; i < this->m_nAgentNum-1; i++)
    {
        for (int j = i; j < this->m_nAgentNum; j++)
        {

            int nAgent1 = i+1;
            int nAgent2 = j+1;
            vector<MapConnection *> gLocalConnections;
            vector<MapConnection *> gLocalConnections1 = gSplitedConnections[(nAgent1-1)*this->m_nAgentNum+nAgent2-1];
            vector<MapConnection *> gLocalConnections2 = gSplitedConnections[(nAgent2-1)*this->m_nAgentNum+nAgent1-1];

            gLocalConnections.insert(gLocalConnections.end(), gLocalConnections1.begin(), gLocalConnections1.end());
            gLocalConnections.insert(gLocalConnections.end(), gLocalConnections2.begin(), gLocalConnections2.end());
            int nLocalConnectionSize = gLocalConnections.size();

            if (nLocalConnectionSize < 20){
                for (int i=0;i<nLocalConnectionSize;i++){
                    MapConnection * pConnection = gLocalConnections[i];
                    gResultConnections.push_back(pConnection);
                }
                continue;
            }

            // cout << "Enter" << endl;
            // cout << "nLocal size is: " << nLocalConnectionSize << endl;
            // for (int i=0;i<gLocalConnections.size();i++){
            //     MapConnection * pConnection = gLocalConnections[i];
                

            //     cout << pConnection->m_mTransform_12.matrix() << endl;
                
            // }

            double ** pGraph;
            pGraph = new double*[nLocalConnectionSize];    //注意，int*[10]表示一个有10个元素的指针数组
            for (int ii = 0; ii < nLocalConnectionSize; ii++)
            {
                pGraph[ii] = new double[nLocalConnectionSize];
            }


            for (int ii = 0; ii < nLocalConnectionSize; ii++)
            {
                for (int jj = 0; jj < nLocalConnectionSize; jj++)
                {
                    pGraph[ii][jj] = 0;
                           
                }
            }

            cout << "Start generate map" << endl;
            for (int ii=0; ii<nLocalConnectionSize;ii++){
                for (int jj=ii; jj<nLocalConnectionSize;jj++){

                    
                    //Self connected.
                    if (ii==jj){
                        pGraph[ii][jj] = 1;
                        continue;        
                    } 

                    //Check the consistency.
                    MapConnection * pConnection_1 = gLocalConnections[ii];
                    MapConnection * pConnection_2 = gLocalConnections[jj];

                    ServerKeyFrame * pKeyFrame_i = pConnection_1->m_pKeyFrame1;
                    ServerKeyFrame * pKeyFrame_k = pConnection_1->m_pKeyFrame2;
                    Sophus::SE3 mRelativePose_ik = pConnection_1->m_mTransform_12;

                    ServerKeyFrame * pKeyFrame_j, * pKeyFrame_l;
                    Sophus::SE3 mRelativePose_jl;
                    if (pConnection_2->m_pKeyFrame1->m_nClientID == pKeyFrame_i->m_nClientID &&
                        pConnection_2->m_pKeyFrame2->m_nClientID == pKeyFrame_k->m_nClientID){
                        //Consistent.
                        //pConnection2 is pConnection_jl
                        pKeyFrame_j = pConnection_2->m_pKeyFrame1;
                        pKeyFrame_l = pConnection_2->m_pKeyFrame2;
                        mRelativePose_jl = pConnection_2->m_mTransform_12;
                    } else if ( pConnection_2->m_pKeyFrame1->m_nClientID == pKeyFrame_k->m_nClientID &&
                                pConnection_2->m_pKeyFrame2->m_nClientID == pKeyFrame_i->m_nClientID){
                        //Consistent.
                        //pConnection2 is pConnection_lj
                        pKeyFrame_l = pConnection_2->m_pKeyFrame1;
                        pKeyFrame_j = pConnection_2->m_pKeyFrame2;
                        mRelativePose_jl = pConnection_2->m_mTransform_12.inverse();
                    } else{
                        continue;
                    }

                    Eigen::Matrix3d mRotation_wi_i, mRotation_wi_j, mRotation_wi_l, mRotation_wi_k;
                    Eigen::Vector3d mTranslation_wi_i, mTranslation_wi_j, mTranslation_wi_l, mTranslation_wi_k;

                    pKeyFrame_i->GetVIOPose(mTranslation_wi_i, mRotation_wi_i);
                    pKeyFrame_j->GetVIOPose(mTranslation_wi_j, mRotation_wi_j);
                    pKeyFrame_l->GetVIOPose(mTranslation_wi_l, mRotation_wi_l);
                    pKeyFrame_k->GetVIOPose(mTranslation_wi_k, mRotation_wi_k);

                    Eigen::Matrix3d mRotationEulerYaw_wi_i, mRotationEulerPitch_wi_i, mRotationEulerRoll_wi_i;
                    Eigen::Matrix3d mRotationEulerYaw_wi_j, mRotationEulerPitch_wi_j, mRotationEulerRoll_wi_j;
                    Eigen::Matrix3d mRotationEulerYaw_wi_l, mRotationEulerPitch_wi_l, mRotationEulerRoll_wi_l;
                    Eigen::Matrix3d mRotationEulerYaw_wi_k, mRotationEulerPitch_wi_k, mRotationEulerRoll_wi_k;

                    ServerUtility::DecomposeEuler(mRotation_wi_i, mRotationEulerYaw_wi_i, mRotationEulerPitch_wi_i, mRotationEulerRoll_wi_i);
                    ServerUtility::DecomposeEuler(mRotation_wi_i, mRotationEulerYaw_wi_j, mRotationEulerPitch_wi_j, mRotationEulerRoll_wi_j);
                    ServerUtility::DecomposeEuler(mRotation_wi_i, mRotationEulerYaw_wi_l, mRotationEulerPitch_wi_l, mRotationEulerRoll_wi_l);
                    ServerUtility::DecomposeEuler(mRotation_wi_i, mRotationEulerYaw_wi_k, mRotationEulerPitch_wi_k, mRotationEulerRoll_wi_k);


                    //Compute the covariance of the measurement.
                    Eigen::Matrix4d mCov_ij = pKeyFrame_j->DeterminePropChainCov(pKeyFrame_i);
                    Eigen::Matrix4d mCov_lk = pKeyFrame_k->DeterminePropChainCov(pKeyFrame_l);

                    Sophus::SE3 mVIOPose_wi(mRotation_wi_i, mTranslation_wi_i);
                    Sophus::SE3 mVIOPose_wj(mRotation_wi_j, mTranslation_wi_j);
                    Sophus::SE3 mVIOPose_wl(mRotation_wi_l, mTranslation_wi_l);
                    Sophus::SE3 mVIOPose_wk(mRotation_wi_k, mTranslation_wi_k);

                    Sophus::SE3 mRelativePose_ij = mVIOPose_wi.inverse()  * mVIOPose_wj;
                    Sophus::SE3 mRelativePose_lk = mVIOPose_wl.inverse()  * mVIOPose_wk;

                    Eigen::Matrix4d mJacobianCov_ij = Eigen::Matrix4d::Identity();
                    mJacobianCov_ij.block<3 , 3>(1 , 1) = mRelativePose_ik.inverse().rotation_matrix();
                    Eigen::Vector3d mC_ij(0.0 , 0.0 , 0.0);
                    Eigen::Matrix3d mA_ij = mRelativePose_ik.inverse().rotation_matrix() *  mRotationEulerRoll_wi_i.transpose() *  mRotationEulerPitch_wi_i.transpose(); 
                    Eigen::Matrix3d mB_ij = mRotationEulerPitch_wi_j * mRotationEulerRoll_wi_j;
                    Eigen::Vector3d mT1_ij = mRelativePose_jl.rotation_matrix() * mRelativePose_lk.translation();
                    Eigen::Vector3d mT2_ij = mRelativePose_jl.translation();
                    double nRelativeYaw_ij = ServerUtility::R2ypr(mRotation_wi_j)(0) - ServerUtility::R2ypr(mRotation_wi_i)(0); 

                    mC_ij += ServerUtility::ComputeJacobian(mA_ij, mB_ij, mT1_ij, nRelativeYaw_ij);
                    mC_ij += ServerUtility::ComputeJacobian(mA_ij, mB_ij, mT2_ij, nRelativeYaw_ij);
                    mJacobianCov_ij.block<3 , 1>(1 , 0) = mC_ij;



                    Eigen::Matrix4d mJacobianCov_lk = Eigen::Matrix4d::Identity();
                    mJacobianCov_lk.block<3 , 3>(1 , 1) = mRelativePose_ik.inverse().rotation_matrix() * mRelativePose_ij.rotation_matrix() * mRelativePose_jl.rotation_matrix();


                    Sophus::SE3 mErrorPose = mRelativePose_ik.inverse() * mRelativePose_ij * mRelativePose_jl * mRelativePose_lk; 

                    //Compute Error
                    double nEulerError = 0.0;
                    double nEuler_ki = -ServerUtility::R2ypr(mRelativePose_ik.rotation_matrix()).x();
                    double nEuler_ij = ServerUtility::R2ypr(mRelativePose_ij.rotation_matrix()).x();
                    double nEuler_jl = ServerUtility::R2ypr(mRelativePose_jl.rotation_matrix()).x();
                    double nEuler_lk = ServerUtility::R2ypr(mRelativePose_lk.rotation_matrix()).x();
                    nEulerError = nEuler_ki + nEuler_ij + nEuler_jl + nEuler_lk;


                    Eigen::Vector3d mErrorTranslation = mErrorPose.translation();
                    Eigen::Matrix3d mErrorRotation = mErrorPose.rotation_matrix();

                    Eigen::Vector3d mErrorEulerAngle = ServerUtility::R2ypr(mErrorRotation);
                    double nErrorYaw = mErrorEulerAngle.x(); 

                    Eigen::Vector4d mErrorVetor;
                    mErrorVetor(0) = nErrorYaw;
                    mErrorVetor(1) = mErrorTranslation(0);
                    mErrorVetor(2) = mErrorTranslation(1);
                    mErrorVetor(3) = mErrorTranslation(2);
                    
                    // cout << "Error Rotation: " << endl << mErrorRotation << endl;
                    // cout << "Error euler is: " << endl << ServerUtility::R2ypr(mErrorRotation) << endl;
                    // cout << "mErrorVector is: " << endl << mErrorVetor << endl;
                    // cout << "nEuler error is: " << nEulerError << endl; 


                    //Generate the covariance matrix.
                    double nUnitYawCov = 1;
                    double nUnitTranslationCov = 0.01;
                    double nScale_ij = abs((pKeyFrame_j->m_nLocalIndex - pKeyFrame_i->m_nLocalIndex)/4)+1;
                    double nRelativeYawCov_ij = nUnitYawCov * nScale_ij * nScale_ij;
                    double nTranslationCovValue_ij = nUnitTranslationCov * nScale_ij * nScale_ij;
                    
                    double nScale_lk = abs((pKeyFrame_l->m_nLocalIndex - pKeyFrame_k->m_nLocalIndex)/4)+1;
                    double nRelativeYawCov_lk = nUnitYawCov * nScale_lk * nScale_lk;
                    double nTranslationCovValue_lk = nUnitTranslationCov * nScale_lk * nScale_lk;

                    Eigen::Matrix4d mCov = Eigen::Matrix4d::Identity();
                    mCov(0 , 0) = nRelativeYawCov_ij + nRelativeYawCov_lk;
                    mCov(1 , 1) = nTranslationCovValue_ij + nTranslationCovValue_lk;
                    mCov(2 , 2) = nTranslationCovValue_ij + nTranslationCovValue_lk;
                    mCov(3 , 3) = nTranslationCovValue_ij + nTranslationCovValue_lk;
                    


                    double nError1 = mErrorVetor.transpose() * mCov.inverse()  * mErrorVetor; 


                    mCov = mJacobianCov_ij * mCov_ij * mJacobianCov_ij.transpose() + mJacobianCov_lk *  mCov_lk * mJacobianCov_lk.transpose();
                    // cout << "Cov det is: " << mCov.determinant() << endl;
                    // // mCov = mCov.inverse();


                    // cout << "nError1 is: " << nError1 << endl;

                    double nError2 = mErrorVetor.transpose() * mCov.inverse()  * mErrorVetor; 

                    // cout << "nError2 is: " << nError2 << endl;

                    // cout << "mCov_ij is: " << endl << mCov_ij << endl;

                    // cout << "mCov_lk is: " << endl << mCov_lk << endl;

                    double nError = mErrorVetor.transpose() * mCov.inverse()  * mErrorVetor; 



                    if (nError <= this->m_nGamma){
                        pGraph[ii][jj] = 1;
                        pGraph[jj][ii] = 1;
                    }

                }
            }


            cout << "Finish generate map" << endl;

            Eigen::MatrixXi mConsistencyGraph(nLocalConnectionSize, nLocalConnectionSize);
            mConsistencyGraph *=0;
            cout << "Local connection size: " << nLocalConnectionSize << endl;
            for (int ii=0; ii<nLocalConnectionSize;ii++){
                for (int jj=0; jj<nLocalConnectionSize;jj++){
                    if (pGraph[ii][jj] >0.9){
                        mConsistencyGraph(ii , jj) = 1;
                    }else{
                        mConsistencyGraph(ii , jj) = 0;
                    }
                }
            }

            PCM::PattabiramanMaxCliqueSolverHeuristic solver;
            GraphConsistencyEvaluator ce(mConsistencyGraph);

            PCM::PCMSolver<PCM::PattabiramanMaxCliqueSolverHeuristic,
                             GraphConsistencyEvaluator>
            pcm(solver, ce);
            pcm.add_measurements(nLocalConnectionSize);

            auto consistent_set = pcm.solve_pcm();


            for (int i=0;i<nLocalConnectionSize;i++){
                MapConnection * pConnection = gLocalConnections[i];
                if (consistent_set[i]){
                    gResultConnections.push_back(pConnection);
                }
            }

            // for (auto pConnection : gLocalConnections){
            //     gResultConnections.push_back(pConnection);
            // }


            // cout << "Debug0" << endl;
            //Debug.
            //Save the graph
           

            fOutFile << "From " << i+1 << " to " << j+1 << endl;
            
            fOutFile << "Connection size: " << gLocalConnections.size() << endl;

            fOutFile << endl;


            //Save connections.
            fOutFile << "Connections: " << endl;
            for (int i=0;i<gLocalConnections.size();i++){
                MapConnection * pConnection = gLocalConnections[i];

                fOutFile << "From " << pConnection->m_pKeyFrame2->m_nClientID << "," << pConnection->m_pKeyFrame2->m_nGlobalIndex << " to " << pConnection->m_pKeyFrame1->m_nClientID << "," << pConnection->m_pKeyFrame1->m_nGlobalIndex << ": " << endl;
                // cout << "Debug1" << endl;
                fOutFile << pConnection->m_mTransform_12.matrix() << endl;
                // cout << "Debug2" << endl;
                fOutFile << endl;
            }




            for (int ii=0;ii<nLocalConnectionSize;ii++){
                for (int jj=0;jj<nLocalConnectionSize;jj++){
                    fOutFile << pGraph[ii][jj] << " ";
                }
                fOutFile << endl;
            }

            fOutFile << "Connection condition: " << endl;
            for (auto bConnected : consistent_set){
                fOutFile << bConnected << " ";
            }

            //Free space.
            for(int ii = 0; ii < nLocalConnectionSize; ii++)
            {
                delete[] pGraph[ii];
            }
            delete[] pGraph;
        }
    }

    this->m_gConnections = gResultConnections;
}


    
void ConnectionGraph::SaveGraph(string aFilePath){
    
    ofstream fOutFile(aFilePath);

    int nIntraNum = 0;
    int nInterNum = 0;

    int gConnectionSize[5][5];

    for (int i=0;i<5;i++){
        for (int j=0;j<5;j++){
            gConnectionSize[i][j] = 0;
        }
    }




    for (int i=0;i<this->m_gConnections.size();i++){
        MapConnection * pConnection = this->m_gConnections[i];
        int nClientID1 = pConnection->m_pKeyFrame1->m_nClientID;
        int nClientID2 = pConnection->m_pKeyFrame2->m_nClientID;

        gConnectionSize[nClientID1-1][nClientID2-1] +=1;
        if (pConnection->m_pKeyFrame1->m_nClientID != 
            pConnection->m_pKeyFrame2->m_nClientID){
            nInterNum++;
        }else{
            nIntraNum++;
        }
    }




    fOutFile << "Connection size: " << this->m_gConnections.size() << endl;
    fOutFile << "Intra Connection size: " << nIntraNum << endl;
    fOutFile << "Inter Connection size: " << nInterNum << endl;
    fOutFile << endl;

    for (int i=0;i<this->m_nAgentNum;i++){
        for (int j=0;j<this->m_nAgentNum;j++){
            fOutFile << "Connection from " << i+1 << " to " << j+1  << " size: " << gConnectionSize[i][j] << endl;        
        }
    }


    //Save connections.
    fOutFile << "Connections: " << endl;
    for (int i=0;i<this->m_gConnections.size();i++){
        MapConnection * pConnection = this->m_gConnections[i];
        fOutFile << "From " << pConnection->m_pKeyFrame2->m_nClientID << "," << pConnection->m_pKeyFrame2->m_nGlobalIndex << " to " << pConnection->m_pKeyFrame1->m_nClientID << "," << pConnection->m_pKeyFrame1->m_nGlobalIndex << ": " << endl;
        fOutFile << pConnection->m_mTransform_12.matrix() << endl;
        fOutFile << endl;
    }
}
