/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/
#ifndef NOISEPARAMS_H
#define NOISEPARAMS_H 
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

class NoiseParams {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NoiseParams();

        void setGyroscopeNoise(double std);
        void setGyroscopeNoise(const Eigen::Vector3d& std);
        void setGyroscopeNoise(const Eigen::Matrix3d& cov);

        void setAccelerometerNoise(double std);
        void setAccelerometerNoise(const Eigen::Vector3d& std);
        void setAccelerometerNoise(const Eigen::Matrix3d& cov);  

        void setDisturbanceNoise(double std);
        void setDisturbanceNoise(const Eigen::Vector3d& std);
        void setDisturbanceNoise(const Eigen::Matrix3d& cov);

        void setGyroscopeBiasNoise(double std);
        void setGyroscopeBiasNoise(const Eigen::Vector3d& std);
        void setGyroscopeBiasNoise(const Eigen::Matrix3d& cov);

        void setAccelerometerBiasNoise(double std);
        void setAccelerometerBiasNoise(const Eigen::Vector3d& std);
        void setAccelerometerBiasNoise(const Eigen::Matrix3d& cov);  

        void setContactNoise(double std);
        void setContactNoise(const Eigen::Vector3d& std);
        void setContactNoise(const Eigen::Matrix3d& cov);

        Eigen::Matrix3d getGyroscopeCov();
        Eigen::Matrix3d getAccelerometerCov();
        Eigen::Matrix3d getDisturbanceCov();
        Eigen::Matrix3d getGyroscopeBiasCov();
        Eigen::Matrix3d getAccelerometerBiasCov();
        Eigen::Matrix3d getContactCov();

        friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);  

    private:
        Eigen::Matrix3d Qg_;
        Eigen::Matrix3d Qa_;
        Eigen::Matrix3d Qd_;
        Eigen::Matrix3d Qbg_;
        Eigen::Matrix3d Qba_;
        Eigen::Matrix3d Ql_;
        Eigen::Matrix3d Qc_;
};

} // end inekf namespace
#endif 