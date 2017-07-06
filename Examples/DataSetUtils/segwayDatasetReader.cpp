#include "segwayDatasetReader.hpp"
#include <iostream>

segwayRobotDatasetReader::segwayRobotDatasetReader(std::string  datasetPath):
        datasetPath_(datasetPath){


    loadFisheyeList();
    loadGyroReadings();
    loadAccelReadings();


    time_overlap_start_end_ = getAllMeasOverlapTime();
    interpolateRawData();


}


bool segwayRobotDatasetReader::loadFisheyeList(){
    std::string fisheyeListPath = datasetPath_ + "/fisheye_timestamps.txt";

    std::ifstream ifs(fisheyeListPath);
    if(!ifs.is_open()){
        std::cerr<< "Failed to open fisheye list file: " << fisheyeListPath<<std::endl;
        return false;
    }

    double lastTimestamp = -1;
    std::string oneLine;
    okvis::CameraMeasurement camMeas;
    while(!ifs.eof()){
        std::getline(ifs, oneLine);

        std::stringstream stream(oneLine);
        std::string s;
        std::getline(stream, s, ' ');
        if(s.empty()) break;
        camMeas.measurement.fileName =datasetPath_ + "/" + s;


       // cv::Mat img = cv::imread(camMeas.measurement.fileName);
        //if (img.empty()) continue;
        std::getline(stream, s, ' ');


        double t1 = std::stod(s)*1e-6;

        if(std::abs(t1 - lastTimestamp) < 1e-3 || lastTimestamp > t1 ){
            continue;
        }

        lastTimestamp = t1;
        //std::cout<<t1<<std::endl;
        camMeas.timeStamp = t1;

        CameraMeasDeque_.push_back(camMeas);

    }
    ifs.close();




    std::cout<<"load fisheye data: "<<CameraMeasDeque_.size()<<std::endl;
    return true;
}
bool segwayRobotDatasetReader::loadGyroReadings(){
    std::string gyroListPath = datasetPath_ + "/gyro.txt";

    std::ifstream ifs(gyroListPath);
    if(!ifs.is_open()){
        std::cerr<< "Failed to open fisheye list file: " << gyroListPath<<std::endl;
        return false;
    }
    std::string oneLine;
    okvis::GyroMeasurement gyroMeas;

    double lastTimestamp = -1;
    while(!ifs.eof()){
        std::getline(ifs, oneLine);

        std::stringstream stream(oneLine);
        std::string s;
        std::getline(stream, s, ',');
        //std::cout<<"t: "<<s<<std::endl;
        if(s.empty()) break;
        double t1 = std::stof(s)*1e-6;

        if(std::abs(t1- lastTimestamp)< 1e-3 || lastTimestamp > t1) continue;

        lastTimestamp = t1;

        gyroMeas.timeStamp = (t1);

        std::getline(stream, s, ','); //skip one


        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            //std::cout<<s<<std::endl;
            gyr[j] = std::stof(s);
        }
        gyroMeas.measurement.data_ = gyr;


        GyroMeasDeque_.push_back(gyroMeas);

    }
    ifs.close();

    std::cout<<"load gyro data: "<<GyroMeasDeque_.size()<<std::endl;
    return true;


}

bool segwayRobotDatasetReader::loadAccelReadings() {
    std::string accelListPath = datasetPath_ + "/accel.txt";

    std::ifstream ifs(accelListPath);
    if(!ifs.is_open()){
        std::cerr<< "Failed to open fisheye list file: " << accelListPath<<std::endl;
        return false;
    }
    std::string oneLine;
    okvis::AccelMeasurement accelMeas;

    double lastTimestamp = -1;
    while(!ifs.eof()){
        std::getline(ifs, oneLine);

        std::stringstream stream(oneLine);
        std::string s;
        std::getline(stream, s, ',');
        //std::cout<<"t: "<<s<<std::endl;
        if(s.empty()) break;
        double t1 = std::stof(s)*1e-6;

        if(std::abs(t1 - lastTimestamp)<1e-3 || lastTimestamp > t1){
            continue;
        }

        lastTimestamp = t1;
        accelMeas.timeStamp = (t1);

        std::getline(stream, s, ','); //skip one


        Eigen::Vector3d accel;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            //std::cout<<s<<std::endl;
            accel[j] = std::stof(s);
        }
        accelMeas.measurement.data_ = accel;


        AccelMeasDeque_.push_back(accelMeas);

    }
    ifs.close();

    std::cout<<"load accel data: "<<AccelMeasDeque_.size()<<std::endl;
    return true;


}



std::pair<double,double> segwayRobotDatasetReader::getAllMeasOverlapTime(){

    std::pair<double,double> time_start_end;




    std::vector<double> start_times ,end_times;
    start_times.push_back(CameraMeasDeque_.front().timeStamp);
    start_times.push_back(GyroMeasDeque_.front().timeStamp);
    start_times.push_back(AccelMeasDeque_.front().timeStamp);


    end_times.push_back(CameraMeasDeque_.back().timeStamp);
    end_times.push_back(GyroMeasDeque_.back().timeStamp);
    end_times.push_back(AccelMeasDeque_.back().timeStamp);

    double s = getMax(start_times);
    double e = getMin(end_times);





    std::cout<<"s: "<<s <<" e: "<<e<<std::endl;
    std::cout<<"CameraMeasDeque_: "<<CameraMeasDeque_.front().timeStamp<<std::endl;
    std::cout<<"GyroMeasDeque_: "<<GyroMeasDeque_.front().timeStamp<<std::endl;
    std::cout<<"AccelMeasDeque_: "<<AccelMeasDeque_.front().timeStamp<<std::endl;


    std::cout<<"CameraMeasDeque_: "<<CameraMeasDeque_.back().timeStamp<<std::endl;
    std::cout<<"GyroMeasDeque_: "<<GyroMeasDeque_.back().timeStamp<<std::endl;
    std::cout<<"AccelMeasDeque_: "<<AccelMeasDeque_.back().timeStamp<<std::endl;




    time_start_end = std::make_pair(s,e);



    return time_start_end;


}

void segwayRobotDatasetReader::interpolateRawData(){

    double overlap_start = time_overlap_start_end_.first;
    double overlap_end = time_overlap_start_end_.second;

    okvis::CameraMeasurementDeque::const_iterator citr = CameraMeasDeque_.begin();
    okvis::GyroMeasurementDeque::const_iterator gitr = GyroMeasDeque_.begin();

    okvis::AccelMeasurementDeque::const_iterator aitr = AccelMeasDeque_.begin();


    // Find Start fiducial
    while(citr->timeStamp < overlap_start) citr++;


    // cursor to onestep forward fiducial
    while(!( gitr->timeStamp <= citr->timeStamp && (gitr+1)->timeStamp > citr->timeStamp)) gitr++;


    std::cout<<"Interpolate ..."<<std::endl;
    // interpolate gyro and priorEstPose
    while(citr->timeStamp < overlap_end){

        double fiducial = citr->timeStamp;

        CropCameraMeasDeque_.push_back(*citr);


        // For gyro
        while(gitr->timeStamp < fiducial ){
            if(!(gitr->timeStamp <= fiducial && (gitr+1)->timeStamp > fiducial )){
                interp_GyroMeasDeque_.push_back(*gitr);
                //std::cout<<gitr->measurement.data_.transpose()<<std::endl;

            }else{
                double t0 = gitr->timeStamp;
                double t1 = (gitr+1)->timeStamp;
                Eigen::Vector3d g0 = gitr->measurement.data_;
                Eigen::Vector3d g1 = (gitr+1)->measurement.data_;

                double interp_t = fiducial;
                Eigen::Vector3d interp_v = interpolate<Eigen::Vector3d>(t0,t1,interp_t,g0,g1);

                okvis::GyroMeasurement interp_g;
                interp_g.timeStamp = fiducial;
                interp_g.measurement.data_ = interp_v;

                interp_GyroMeasDeque_.push_back(interp_g);
                //std::cout<<"*"<<interp_g.measurement.data_.transpose()<<std::endl;

            }
            gitr++;
        }






        citr++;
    }



    std::cout<<"Align gyro and accel ..."<<interp_GyroMeasDeque_.front().timeStamp<<" "<<interp_GyroMeasDeque_.back().timeStamp<<std::endl;
    //For align gyro and accel
    okvis::GyroMeasurementDeque::const_iterator inter_gitr = interp_GyroMeasDeque_.begin();


    while(!( aitr->timeStamp <= inter_gitr->timeStamp && (aitr+1)->timeStamp > inter_gitr->timeStamp)) {


        aitr++;
    }


    while(inter_gitr != interp_GyroMeasDeque_.end()){
        double fiducial = inter_gitr->timeStamp;
        while(true){
            if(!( aitr->timeStamp <= fiducial && (aitr+1)->timeStamp > fiducial)){
                if(aitr+1 == AccelMeasDeque_.end())
                    break;
                aitr++;
            }else{

                double t0 = aitr->timeStamp;
                double t1 = (aitr+1)->timeStamp;
                Eigen::Vector3d a0 = aitr->measurement.data_;
                Eigen::Vector3d a1 = (aitr+1)->measurement.data_;

                double interp_t = fiducial;


                okvis::AccelMeasurement inter_accel;
                inter_accel.timeStamp = fiducial;
                inter_accel.measurement.data_ = interpolate(t0,t1,interp_t,a0,a1);

                interp_AccelMeasDeque_.push_back(inter_accel);

                //std::cout<<"*"<<inter_accel.measurement.data_.transpose()<<std::endl;

                break;


            }
        }

        inter_gitr++;


    }


    std::cout<<"crop_camera: "<<CropCameraMeasDeque_.size()<<" "<<CropCameraMeasDeque_.front().timeStamp
             <<" "<<CropCameraMeasDeque_.back().timeStamp<<std::endl;
    std::cout<<"interp_gyro: "<<interp_GyroMeasDeque_.size()<<" "<<interp_GyroMeasDeque_.front().timeStamp
             <<" "<<interp_GyroMeasDeque_.back().timeStamp<<std::endl;


}
