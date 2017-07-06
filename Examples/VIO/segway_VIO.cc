


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <boost/concept_check.hpp>

#include<opencv2/core/core.hpp>

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"

#include<System.h>
#include "../DataSetUtils/segwayDatasetReader.hpp"
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder" << endl;
        return 1;
    }


    std::string dataSetPath = string(argv[3]);

    segwayRobotDatasetReader dataReader(dataSetPath);
    okvis::CameraMeasurementDeque cameraMeasDeque = dataReader.getCameraMeasDeque();
    okvis::GyroMeasurementDeque gyroMeasDeque = dataReader.getGyroMeasDeque();
    okvis::AccelMeasurementDeque accelMeasDeque = dataReader.getAccelMeasDeue();
     

    
    int nImages = cameraMeasDeque.size();
    
    
    std::vector<ORB_SLAM2::IMUData> vimuData;

    
    std::cout<<"Get Image: "<<nImages<<" "<<"Get IMU： "<< vimuData.size()<<std::endl;

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    
    ORB_SLAM2::ConfigParam config(argv[2]);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    int imuStart = 0;
    int imuCnt = imuStart;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(cameraMeasDeque[ni].measurement.fileName,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = cameraMeasDeque[ni].timeStamp;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  cameraMeasDeque[ni].measurement.fileName << endl;
            return 1;
        }
        
        // get IMU data
        std::vector<ORB_SLAM2::IMUData> imuReadingBatch;

        while(gyroMeasDeque.at(imuCnt).timeStamp < tframe)
	{
        Eigen::Vector3d g = gyroMeasDeque.at(imuCnt).measurement.data_;
        Eigen::Vector3d a = accelMeasDeque.at(imuCnt).measurement.data_;
        ORB_SLAM2::IMUData imuData;
        imuData._t = gyroMeasDeque.at(imuCnt).timeStamp;
        imuData._a = a;
        imuData._g = g;
	  imuReadingBatch.push_back(imuData);
	  imuCnt ++;
	 // std::cout<<vimuData.at(imuCnt)._g.transpose()<<std::endl;
	  
	}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonoVI(im, imuReadingBatch, tframe);
	
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
            T = cameraMeasDeque[ni+1].timeStamp-tframe;
        else if(ni>0)
            T = tframe-cameraMeasDeque[ni-1].timeStamp;

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
