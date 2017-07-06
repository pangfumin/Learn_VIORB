#ifndef _SEGWAYROBOTDATASETREADER_h_
#define _SEGWAYROBOTDATASETREADER_h_


#include <fstream>
#include <istream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
namespace okvis{
    typedef std::deque<Eigen::Vector3d> LinearVelocityDeque;


    template<class MEASUREMENT_T>
    struct Measurement {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timeStamp;      ///< Measurement timestamp
        MEASUREMENT_T measurement;  ///< Actual measurement.
        int sensorId = -1;          ///< Sensor ID. E.g. camera index in a multicamera setup

        /// \brief Default constructor.
        Measurement()
                : timeStamp(0.0) {
        }
        /**
         * @brief Constructor
         * @param timeStamp_ Measurement timestamp.
         * @param measurement_ Actual measurement.
         * @param sensorId Sensor ID (optional).
         */
        Measurement(const double& timeStamp_, const MEASUREMENT_T& measurement_,
                    int sensorId = -1)
                : timeStamp(timeStamp_),
                  measurement(measurement_),
                  sensorId(sensorId) {
        }

        Measurement(const Measurement& rhs):timeStamp(rhs.timeStamp), measurement(rhs.measurement), sensorId(rhs.sensorId)
        {

        }
        Measurement& operator=(const Measurement& rhs)
        {
            timeStamp = rhs.timeStamp;
            measurement = rhs.measurement;
            sensorId = rhs.sensorId;
            return *this;
        }
    };

/// \brief IMU measurements. For now assume they are synchronized:
    struct ImuSensorReadings {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /// \brief Default constructor.
        ImuSensorReadings()
                : gyroscopes(),
                  accelerometers() {
        }
        /**
         * @brief Constructor.
         * @param gyroscopes_ Gyroscope measurement.
         * @param accelerometers_ Accelerometer measurement.
         */
        ImuSensorReadings(Eigen::Vector3d gyroscopes_,
                          Eigen::Vector3d accelerometers_)
                : gyroscopes(gyroscopes_),
                  accelerometers(accelerometers_) {
        }
        Eigen::Vector3d gyroscopes;     ///< Gyroscope measurement.
        Eigen::Vector3d accelerometers; ///< Accelerometer measurement.
    };

    struct GyroReading{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /// \brief Default constructor.
        GyroReading()
                : data_() {
        }

        GyroReading(Eigen::Vector3d data)
                : data_(data){
        }

        GyroReading(const GyroReading& rhs): data_(rhs.data_)
        {

        }

        GyroReading& operator=(const GyroReading& rhs)
        {
            data_=rhs.data_;
            return *this;
        }

        Eigen::Vector3d data_;     ///< Gyroscope measurement.

    };


/// \brief Camera measurement.
    struct CameraData {
        CameraData(){};
        std::string fileName;

        bool deliversKeypoints; ///< Are the keypoints delivered too?
        std::string idInSource; /// (0 based) id of the frame within the video or the image folder
    };

    typedef GyroReading AccelReading;

    typedef Measurement<GyroReading> GyroMeasurement;
    typedef Measurement<AccelReading> AccelMeasurement;

    typedef Measurement<GyroReading> GyroMeasurement;
    typedef Measurement<AccelReading> AccelMeasurement;
    typedef std::deque<GyroMeasurement, Eigen::aligned_allocator<GyroMeasurement>> GyroMeasurementDeque;
    typedef std::deque<AccelMeasurement, Eigen::aligned_allocator<AccelMeasurement>> AccelMeasurementDeque;
    typedef Measurement<CameraData> CameraMeasurement;
    typedef std::deque<CameraMeasurement,Eigen::aligned_allocator<CameraMeasurement> > CameraMeasurementDeque;

}



class segwayRobotDatasetReader{
public:



    segwayRobotDatasetReader(std::string datasetPath);


    bool loadFisheyeList();
    bool loadGyroReadings();
    bool loadAccelReadings();

    std::pair<double, double> getAllMeasOverlapTime();
    void interpolateRawData();




    // get time aligned data
    inline okvis::CameraMeasurementDeque getCameraMeasDeque(){
        return CropCameraMeasDeque_;
    }
    inline okvis::GyroMeasurementDeque getGyroMeasDeque(){
        return  interp_GyroMeasDeque_;

    }
    inline okvis::AccelMeasurementDeque getAccelMeasDeue(){
        return  interp_AccelMeasDeque_;
    }




    // get raw data
    inline okvis::CameraMeasurementDeque getRawCameraMeasDeque(){
        return CameraMeasDeque_;
    }
    inline okvis::GyroMeasurementDeque getRawGyroMeasDeque(){
        return  GyroMeasDeque_;

    }
    inline okvis::AccelMeasurementDeque getRawAccelMeasDeue(){
        return  AccelMeasDeque_;
    }











private:

    std::string datasetPath_;




    okvis::CameraMeasurementDeque CameraMeasDeque_, CropCameraMeasDeque_ ;  // Crop camera deque to make use of the camera data where other sensors datas are all available
    okvis::GyroMeasurementDeque  GyroMeasDeque_, interp_GyroMeasDeque_;  // @ ~100Hz
    okvis::AccelMeasurementDeque AccelMeasDeque_, interp_AccelMeasDeque_; // @ ~125Hz



    double lastFisheyeTimestamp_;
    double lastGyroTimestamp_;
    double lastAccelTimestamp_;
    double lastPriorEstPose_;




    std::pair<double,double> time_overlap_start_end_;

    Eigen::Vector3d gyroScale_;
    Eigen::Vector3d gyroBias_;
    Eigen::Vector3d accelScale_;
    Eigen::Vector3d accelBias_;



    static double getMax(std::vector<double> times){

        double t = -1;
        for (auto i : times)
            if (t < i) t = i;

        return t;
    }

    static double getMin(std::vector<double> times){

        double t = 1e9;
        for (auto i : times)
            if (t > i) t = i;

        return t;
    }

    template <typename  T>
    static T interpolate(double t0, double t1, double interp_t,T& v0, T& v1){

        T interp_v;
        double r = (interp_t - t0)/(t1-t0);
        interp_v = v0 + r*(v1 - v0);

        return interp_v;

    }



    static Eigen::Matrix3d interpolateSO3 (double t0, double t1, double interp_t, Eigen::Matrix3d& v0, Eigen::Matrix3d& v1){

        double r = (interp_t - t0)/(t1-t0);

        Eigen::Quaterniond q0(v0);
        Eigen::Quaterniond q1(v1);
        Eigen::Quaterniond interp_q = q0.slerp(r,q1);
        Eigen::Matrix3d interp_R(interp_q);

        return interp_R;
    }

};

std::ostream& operator<<(std::ostream &os, const okvis::GyroMeasurementDeque & rhs);



#endif
