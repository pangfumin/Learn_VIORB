/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <boost/concept_check.hpp>

#include<opencv2/core/core.hpp>

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
void LoadIMUData(const string &strIMUDataFilePath, 
		 std::vector<ORB_SLAM2::IMUData>& vimuData);

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
                        "path_to_image0_folder  path_to_image1_folder path_to_times_file path_to_imu_file"  << endl;
        return 1;
    }
    
    
     
    // Retrieve paths to images
    vector<string> vstrImage0Filenames;
    vector<string> vstrImage1Filenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[5]), vstrImage0Filenames, vTimestamps);
    LoadImages(string(argv[4]), string(argv[5]), vstrImage1Filenames, vTimestamps);

    
    int nImages = vstrImage0Filenames.size();
    
    
    std::vector<ORB_SLAM2::IMUData> vimuData;
    LoadIMUData(string(argv[6]),vimuData);
    
    
    std::cout<<"Get Image: "<<nImages<<" "<<"Get IMU： "<< vimuData.size()<<std::endl;

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
    
    ORB_SLAM2::ConfigParam config(argv[2]);

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);



    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    int imuStart = 0;
    int imuCnt = imuStart;

    // Main loop
    cv::Mat im0, im1;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im0 = cv::imread(vstrImage0Filenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        im1 = cv::imread(vstrImage1Filenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im0.empty() || im1.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImage0Filenames[ni] << endl;
            return 1;
        }
        
        // get IMU data
        std::vector<ORB_SLAM2::IMUData> imuReadingBatch;
        while(vimuData.at(imuCnt)._t < tframe)
	{
	  imuReadingBatch.push_back(vimuData.at(imuCnt));
	  imuCnt ++;
	 // std::cout<<vimuData.at(imuCnt)._g.transpose()<<std::endl;
	  
	}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        cv::Mat imLeftRect, imRightRect;
        cv::remap(im0,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(im1,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        // Pass the image to the SLAM system
        SLAM.TrackStereoVI(imLeftRect,imRightRect, imuReadingBatch, tframe);
	
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
     SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMUData(const string &strIMUDataFilePath, std::vector<ORB_SLAM2::IMUData>& vimuData)
{
    ifstream fIMUdata;
    fIMUdata.open(strIMUDataFilePath.c_str());

    if (!fIMUdata.good()) {
      std::cout<<"Can not read: "<<strIMUDataFilePath<<std::endl;
      return ;
    }
    
    std::string line;
    int cnt = 0;
     while (std::getline(fIMUdata, line))
     {
        std::stringstream  ss;
        ORB_SLAM2::IMUData Imu;
        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        ss<<s;
        double t;
        ss>>t;

        Imu._t = t/1e9;

        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          std::stringstream  ssg;
          ssg<<s;
          double g_ele;
          ssg>>g_ele;
          Imu._g[j] = g_ele;

        }
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          std::stringstream  ssa;
          ssa<<s;
          double a_ele;
          ssa>> a_ele;
          Imu._a[j] = a_ele;

        }
        //std::cout<< Imu._a.transpose()<<std::endl;
        vimuData.push_back(Imu);
	
    }
}

