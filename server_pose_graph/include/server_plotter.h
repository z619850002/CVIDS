#ifndef SERVER_PLOTTER_H_
#define SERVER_PLOTTER_H_



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.h>

#include <opencv2/core/core.hpp>

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <set>
#include <unordered_map>
#include <map>
#include <iterator>
#include<algorithm>
#include <mutex>
#include<pangolin/pangolin.h>

#include "server_keyframe.h"

using namespace std;

class ServerPlotter
{
public:
	ServerPlotter(){
    	float fps = 10;
    	this->m_nT = 1e3/fps;


	    this->m_nImageWidth = 640;
	    this->m_nImageHeight = 480;

	    this->m_nViewpointX = 0;
	    this->m_nViewpointY = -0.7;
	    this->m_nViewpointZ = -1.8;
	    this->m_nViewpointF = 500;


	    //Drawer settings
	    this->mKeyFrameSize = 0.2;
	    this->mKeyFrameLineWidth = 1;
	    this->mGraphLineWidth = 0.9;
	    this->mPointSize = 5;
	    this->mCameraSize = 0.08;
	    this->mCameraLineWidth = 1.5;

        this->m_nShowLoop = 0;

        this->m_bSaveMesh = false;

	}


public:
    bool GetSaveMesh(){
        this->m_mKeyFrameMutex.lock();
        bool bSave = this->m_bSaveMesh;
        this->m_mKeyFrameMutex.unlock();
        return bSave;
    }


    bool SetSaveMesh(bool bSave){
        this->m_mKeyFrameMutex.lock();
        this->m_bSaveMesh = bSave;
        this->m_mKeyFrameMutex.unlock();
        return true;
    }


    void Run(){
    	pangolin::CreateWindowAndBind("MapMerger: Map Viewer",1024,768);
	    // 3D Mouse handler requires depth testing to be enabled
	    glEnable(GL_DEPTH_TEST);

	    // Issue specific OpenGl we might need
	    glEnable (GL_BLEND);
	    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	    // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
	    
    	// Define Camera Render Object (for view / scene browsing)
	    pangolin::OpenGlRenderState s_cam(
	                pangolin::ProjectionMatrix(1024,768,this->m_nViewpointF,this->m_nViewpointF,512,389,0.1,1000),
	                pangolin::ModelViewLookAt(this->m_nViewpointX,this->m_nViewpointY,this->m_nViewpointZ, 0,0,0,0.0,-1.0, 0.0)
	                );

	    // Add named OpenGL viewport to window and provide 3D Handler
	    pangolin::View& d_cam = pangolin::CreateDisplay()
	            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
	            .SetHandler(new pangolin::Handler3D(s_cam));

	    pangolin::OpenGlMatrix Twc;
	    Twc.SetIdentity();

        cv::namedWindow("Frames");


        this->m_mCurrentImageMutex.lock();
        for (int i=0;i<4;i++){
            cv::Mat mImage = cv::Mat::zeros(360,480,CV_8UC3);
            this->m_gCurrentImages.push_back(mImage);
        }
        this->m_mLoopClosureImage = cv::Mat::zeros(360,960,CV_8UC3);
        this->m_mCurrentImageMutex.unlock();

	    

	    while(1)
	    {
            if (this->m_nShowLoop > 0){
                this->m_nShowLoop -= 1;   
            }

	        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	        // this->m_pMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

	        //Follow the camera.
	        s_cam.Follow(Twc);
	        
	        d_cam.Activate(s_cam);
	        glClearColor(1.0f,1.0f,1.0f,1.0f);
	        
			       
            // this->m_pMapDrawer->DrawMapPoints();
	    
            this->DrawKeyFrames();

            //Draw current frames.
            cv::Mat mImage = this->GenerateCurrentFrames();
            cv::imshow("Frames", mImage);

            // cv::imshow("Frame0", this->m_gCurrentImages[0]);
            // cv::imshow("Frame1", this->m_gCurrentImages[1]);
            // cv::imshow("Frame2", this->m_gCurrentImages[2]);
            // cv::imshow("Frame3", this->m_gCurrentImages[3]);

	        pangolin::FinishFrame();

	        if (cv::waitKey(this->m_nT) == 's'){
                string aSavePath = "/home/kyrie/Documents/DataSet/CoVins/Trajectory/Euroc/";
                ofstream fOutFile1(aSavePath + "pose1.txt");
                ofstream fOutFile2(aSavePath + "pose2.txt");
                ofstream fOutFile3(aSavePath + "pose3.txt");
                ofstream fOutFile4(aSavePath + "pose4.txt");

                fOutFile1 << fixed;
                fOutFile2 << fixed;
                fOutFile3 << fixed;
                fOutFile4 << fixed;
                // fOutFile << "Test!" << endl;


                //Copy the pose and the timestamp

                this->m_mKeyFrameMutex.lock();

                vector<Sophus::SE3> gPoses1, gPoses2, gPoses3, gPoses4;
                vector<double> gTimeStamp1, gTimeStamp2, gTimeStamp3, gTimeStamp4;
                vector<Eigen::Quaterniond> gRotations1, gRotations2, gRotations3, gRotations4;
                vector<Eigen::Vector3d> gTranslations1, gTranslations2, gTranslations3, gTranslations4;

                for (ServerKeyFrame * pKeyFrame : this->m_gKeyFrames){
                    //Load Time stamp
                    double nTimeStamp = pKeyFrame->m_nTimeStamp;
                    //Load pose.
                    Eigen::Matrix3d mRotation_wi;
                    Eigen::Vector3d mTranslation_wi;


                    pKeyFrame->GetPose(mTranslation_wi, mRotation_wi);
                    Eigen::Quaterniond mQuaternion_wi(mRotation_wi);

                    
                    if (pKeyFrame->m_bHasBeenAligned){
                        //Actually the inverse pose.
                        if (pKeyFrame->m_nClientID == 1){
                            gTimeStamp1.push_back(nTimeStamp);
                            gRotations1.push_back(mQuaternion_wi);
                            gTranslations1.push_back(mTranslation_wi);

                        }else if (pKeyFrame->m_nClientID == 2){
                            gTimeStamp2.push_back(nTimeStamp);
                            gRotations2.push_back(mQuaternion_wi);
                            gTranslations2.push_back(mTranslation_wi);

                        }else if (pKeyFrame->m_nClientID == 3){
                            
                            gTimeStamp3.push_back(nTimeStamp);
                            gRotations3.push_back(mQuaternion_wi);
                            gTranslations3.push_back(mTranslation_wi);

                        }else if (pKeyFrame->m_nClientID == 4){
                            
                            gTimeStamp4.push_back(nTimeStamp);
                            gRotations4.push_back(mQuaternion_wi);
                            gTranslations4.push_back(mTranslation_wi);

                        }
                    }
                    
                }

                this->m_bSaveMesh = true;

                this->m_mKeyFrameMutex.unlock();

                //Save the trajectory.
                for (int i=0;i<gTimeStamp1.size();i++){
                    Eigen::Vector3d mTranslation_wi = gTranslations1[i];
                    Eigen::Quaterniond mRotation_wi = gRotations1[i];

                    fOutFile1   << setprecision(6) << gTimeStamp1[i] << " " 
                                << setprecision(9) << mTranslation_wi(0) << " " << mTranslation_wi(1) << " " << mTranslation_wi(2) << " "
                                << mRotation_wi.x() << " " << mRotation_wi.y() << " " << mRotation_wi.z() << " " << mRotation_wi.w() << endl; 
                }



                //Save the trajectory.
                for (int i=0;i<gTimeStamp2.size();i++){
                    Eigen::Vector3d mTranslation_wi = gTranslations2[i];
                    Eigen::Quaterniond mRotation_wi = gRotations2[i];

                    fOutFile2   << setprecision(6) << gTimeStamp2[i] << " " 
                                << setprecision(9) << mTranslation_wi(0) << " " << mTranslation_wi(1) << " " << mTranslation_wi(2) << " "
                                << mRotation_wi.x() << " " << mRotation_wi.y() << " " << mRotation_wi.z() << " " << mRotation_wi.w() << endl; 
                }


                //Save the trajectory.
                for (int i=0;i<gTimeStamp3.size();i++){
                    Eigen::Vector3d mTranslation_wi = gTranslations3[i];
                    Eigen::Quaterniond mRotation_wi = gRotations3[i];

                    fOutFile3   << setprecision(6) << gTimeStamp3[i] << " " 
                                << setprecision(9) << mTranslation_wi(0) << " " << mTranslation_wi(1) << " " << mTranslation_wi(2) << " "
                                << mRotation_wi.x() << " " << mRotation_wi.y() << " " << mRotation_wi.z() << " " << mRotation_wi.w() << endl; 
                }


                //Save the trajectory.
                for (int i=0;i<gTimeStamp4.size();i++){
                    Eigen::Vector3d mTranslation_wi = gTranslations4[i];
                    Eigen::Quaterniond mRotation_wi = gRotations4[i];

                    fOutFile4   << setprecision(6) << gTimeStamp4[i] << " " 
                                << setprecision(9) << mTranslation_wi(0) << " " << mTranslation_wi(1) << " " << mTranslation_wi(2) << " "
                                << mRotation_wi.x() << " " << mRotation_wi.y() << " " << mRotation_wi.z() << " " << mRotation_wi.w() << endl; 
                }


                fOutFile1.close();
                fOutFile2.close();
                fOutFile3.close();
                fOutFile4.close();
            }  


            //timestamp x y z  qz qx qy qw 

            //Save the trajectory here.
	    }

	}


    void DrawKeyFrames(){
    	const float &w = mKeyFrameSize;
	    const float h = w*0.75;
	    const float z = w*0.6;

        this->m_mKeyFrameMutex.lock();

        vector<ServerKeyFrame *> gDrawedKeyFrames;

        vector<Sophus::SE3> gPoses1, gPoses2, gPoses3, gPoses4;
        for (ServerKeyFrame * pKeyFrame : this->m_gKeyFrames){
            
            //Load pose.
            Eigen::Matrix3d mRotation_wc;
            Eigen::Vector3d mTranslation_wc;
            pKeyFrame->GetCameraPose(mTranslation_wc, mRotation_wc);
            //Actually the inverse pose.
            Sophus::SE3 mPose = Sophus::SE3(mRotation_wc, mTranslation_wc);            
            

            if (pKeyFrame->m_nClientID == 1){
                gPoses1.push_back(mPose);
            }else if (pKeyFrame->m_nClientID == 2){
                gPoses2.push_back(mPose);
            }else if (pKeyFrame->m_nClientID == 3){
                gPoses3.push_back(mPose);
            }else if (pKeyFrame->m_nClientID == 4){
                gPoses4.push_back(mPose);
            }
        }

        this->m_mKeyFrameMutex.unlock();





        for(int i=gPoses1.size()-1; i>=0; i--)
        {
			Sophus::SE3 mPose = gPoses1[i];            
			// mPose = mPose.inverse();

            Eigen::MatrixXd mTwc = mPose.matrix();

            glPushMatrix();
            // cout << "Twc is " << endl << Twc << endl;

	        std::vector<GLfloat > gTwc = {	mTwc(0,0),mTwc(1,0), mTwc(2,0), mTwc(3,0),
				    						mTwc(0,1),mTwc(1,1), mTwc(2,1), mTwc(3,1), 
				    						mTwc(0,2),mTwc(1,2), mTwc(2,2), mTwc(3,2),
				    						mTwc(0,3),mTwc(1,3), mTwc(2,3), mTwc(3,3)};

			
	        glMultMatrixf(gTwc.data());


            glLineWidth(mKeyFrameLineWidth);
            // if (nKeyFrameError < nAverageError){
            // 	glColor3f(1.0f,0.0f,1.0f);
            // }else{
            glColor3f(0.5f,0.0f,1.0f);
            // }
            

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            // if (i < this->gP - 10){
            //     i-=10;
            // }
        }




        for(int i=gPoses2.size()-1; i>=0; i--)
        {
            Sophus::SE3 mPose = gPoses2[i];            
            // mPose = mPose.inverse();

            Eigen::MatrixXd mTwc = mPose.matrix();

            glPushMatrix();
            // cout << "Twc is " << endl << Twc << endl;

            std::vector<GLfloat > gTwc = {  mTwc(0,0),mTwc(1,0), mTwc(2,0), mTwc(3,0),
                                            mTwc(0,1),mTwc(1,1), mTwc(2,1), mTwc(3,1), 
                                            mTwc(0,2),mTwc(1,2), mTwc(2,2), mTwc(3,2),
                                            mTwc(0,3),mTwc(1,3), mTwc(2,3), mTwc(3,3)};

            
            glMultMatrixf(gTwc.data());


            glLineWidth(mKeyFrameLineWidth);
            // if (nKeyFrameError < nAverageError){
            //  glColor3f(1.0f,0.0f,1.0f);
            // }else{
            glColor3f(1.0f,0.0f,1.0f);
            // }
            

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            // if (i < this->gP - 10){
            //     i-=10;
            // }
        }




        for(int i=gPoses3.size()-1; i>=0; i--)
        {
            Sophus::SE3 mPose = gPoses3[i];            
            // mPose = mPose.inverse();

            Eigen::MatrixXd mTwc = mPose.matrix();

            glPushMatrix();
            // cout << "Twc is " << endl << Twc << endl;

            std::vector<GLfloat > gTwc = {  mTwc(0,0),mTwc(1,0), mTwc(2,0), mTwc(3,0),
                                            mTwc(0,1),mTwc(1,1), mTwc(2,1), mTwc(3,1), 
                                            mTwc(0,2),mTwc(1,2), mTwc(2,2), mTwc(3,2),
                                            mTwc(0,3),mTwc(1,3), mTwc(2,3), mTwc(3,3)};

            
            glMultMatrixf(gTwc.data());


            glLineWidth(mKeyFrameLineWidth);
            // if (nKeyFrameError < nAverageError){
            //  glColor3f(1.0f,0.0f,1.0f);
            // }else{
            glColor3f(0.5f,1.0f,1.0f);
            // }
            

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            // if (i < this->gP - 10){
            //     i-=10;
            // }
        }




        for(int i=gPoses4.size()-1; i>=0; i--)
        {
            Sophus::SE3 mPose = gPoses4[i];            
            // mPose = mPose.inverse();

            Eigen::MatrixXd mTwc = mPose.matrix();

            glPushMatrix();
            // cout << "Twc is " << endl << Twc << endl;

            std::vector<GLfloat > gTwc = {  mTwc(0,0),mTwc(1,0), mTwc(2,0), mTwc(3,0),
                                            mTwc(0,1),mTwc(1,1), mTwc(2,1), mTwc(3,1), 
                                            mTwc(0,2),mTwc(1,2), mTwc(2,2), mTwc(3,2),
                                            mTwc(0,3),mTwc(1,3), mTwc(2,3), mTwc(3,3)};

            
            glMultMatrixf(gTwc.data());


            glLineWidth(mKeyFrameLineWidth);
            // if (nKeyFrameError < nAverageError){
            //  glColor3f(1.0f,0.0f,1.0f);
            // }else{
            glColor3f(0.5f,0.0f,0.0f);
            // }
            

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            // if (i < this->gP - 10){
            //     i-=10;
            // }
        }


        // for(int i=1; i< gDrawedKeyFrames.size(); i++)
        // {

        //     ServerKeyFrame * pKeyFrame = gDrawedKeyFrames[i];


        //     ServerKeyFrame * pPreviousKeyFrame = gDrawedKeyFrames[i-1];
        //     if (pPreviousKeyFrame != nullptr){
        //     	glLineWidth(mKeyFrameLineWidth);
	       //      glColor3f(1.0f,0.0f,1.0f);
	       //      glBegin(GL_LINES);

        //         Eigen::Matrix3d mRotation_wi;
        //         Eigen::Vector3d mTranslation_wi;
        //         pKeyFrame->GetPose(mTranslation_wi, mRotation_wi);

        //         Sophus::SE3 mPose = Sophus::SE3(mRotation_wi, mTranslation_wi); 


	       //      Eigen::Vector3d iStartPosition = mPose.inverse().translation();

        //         Eigen::Matrix3d mPreviousRotation_wi;
        //         Eigen::Vector3d mPreviousTranslation_wi;

        //         pPreviousKeyFrame->GetPose(mPreviousTranslation_wi, mPreviousRotation_wi);
        //         Sophus::SE3 mPreviousPose = Sophus::SE3(mPreviousRotation_wi, mPreviousTranslation_wi);

	       //      Eigen::Vector3d iEndPosition = mPreviousPose.inverse().translation();
	       //      glVertex3f(iStartPosition[0],iStartPosition[1],iStartPosition[2]);
	       //      glVertex3f(iEndPosition[0],iEndPosition[1],iEndPosition[2]);
	       //      glEnd();		
        //     }
			

        // }


	    

    }



    void AddKeyFrame(ServerKeyFrame * pKeyFrame){
    	this->m_mKeyFrameMutex.lock();
    	this->m_gKeyFrames.push_back(pKeyFrame);
        this->UpdateCurrentImage(pKeyFrame->m_mImage, pKeyFrame->m_nClientID);
    	this->m_mKeyFrameMutex.unlock();
    }


    cv::Mat GenerateCurrentFrames(){
        this->m_mCurrentImageMutex.lock();
        cv::Mat mImage = cv::Mat::zeros(360*2,960,CV_8UC3);

        cv::Rect iRect1(0, 0, 480, 360);
        cv::Mat mDst1 = mImage(iRect1);

        cv::Rect iRect2(480, 0, 480, 360);
        cv::Mat mDst2 = mImage(iRect2);

        cv::Rect iRect3(0, 360, 480, 360);
        cv::Mat mDst3 = mImage(iRect3);

        cv::Rect iRect4(480, 360, 480, 360);
        cv::Mat mDst4 = mImage(iRect4);



// 
        cv::Rect iRectLoopClosure(320, 300, 320, 120);
        // cv::Mat mDst5 = mImage(iRectLoopClosure);


        // 将指定拷贝至目标图像
        this->m_gCurrentImages[0].copyTo(mImage(iRect1));
        this->m_gCurrentImages[1].copyTo(mImage(iRect2));
        this->m_gCurrentImages[2].copyTo(mImage(iRect3));
        this->m_gCurrentImages[3].copyTo(mImage(iRect4));
        

        if (this->m_nShowLoop >0){
            this->m_mLoopClosureImage.copyTo(mImage(iRectLoopClosure));
        }



        this->m_mCurrentImageMutex.unlock();
        return mImage;
    }



    void UpdateCurrentImage(cv::Mat mImage, int nClientID){
        this->m_mCurrentImageMutex.lock();
        cout << "Update:" << endl;
        cout << "nClientID is: " << nClientID << endl;
        cv::Mat mResizedImage = mImage.clone();
        cv::resize( mResizedImage, 
                    mResizedImage, 
                    cv::Size(480, 360)); 
        //Convert to colorful image.
        if (mResizedImage.channels() == 1){
            cv::cvtColor(mResizedImage, mResizedImage, cv::COLOR_GRAY2BGR); 

        }
        
        // cv::imshow("Resized image", mResizedImage);
        
        if (nClientID > this->m_gCurrentImages.size()){
            cerr << "Wrong client ID in visualization!!" << endl;
            return;
        }
        this->m_gCurrentImages[nClientID-1] = mResizedImage;
        this->m_mCurrentImageMutex.unlock();
    }


    void UpdateLoopClosureImage(cv::Mat mImage){
        this->m_mCurrentImageMutex.lock();
        this->m_mLoopClosureImage = mImage.clone();
        cv::resize( m_mLoopClosureImage,
                    m_mLoopClosureImage,
                    cv::Size(960/3, 360/3));
        if (m_mLoopClosureImage.channels() == 1){
            cv::cvtColor(m_mLoopClosureImage, m_mLoopClosureImage, cv::COLOR_GRAY2BGR); 
        }
        this->m_nShowLoop = 100;
        this->m_mCurrentImageMutex.unlock();
    }


    bool m_bSaveMesh;


    vector<cv::Mat> m_gCurrentImages;
    cv::Mat m_mLoopClosureImage;
    mutex m_mCurrentImageMutex;



    mutex m_mKeyFrameMutex;
    vector<ServerKeyFrame *> m_gKeyFrames;

    // 1/fps in ms
    double m_nT;
    float m_nImageWidth, m_nImageHeight;

    float m_nViewpointX, m_nViewpointY, m_nViewpointZ, m_nViewpointF;


    int m_nShowLoop;


    //Drawer setttings.
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

};





#endif