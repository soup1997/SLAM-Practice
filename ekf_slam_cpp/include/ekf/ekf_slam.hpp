#ifndef EKF_EKF_SLAM_HPP
#define EKF_EKF_SLAM_HPP
#define INF 100000

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <util/measurement_package.hpp>

using namespace std;

class ekfslam
{
private:
    bool _start;

    int _N, _lm, _dim; // pose size, number of landmarks, total dimension

    Eigen::MatrixXd mu;     // mean state vector
    Eigen::MatrixXd Sigma;    // total sigma
    Eigen::MatrixXd lm_sig; // landmark sigma

    Eigen::MatrixXd mu_pred; // for prediction step
    Eigen::MatrixXd Sigma_pred; // for prediction step

    Eigen::MatrixXd R; // motion noise covariance matrix
    Eigen::MatrixXd Q; // sensor noise covariance matrix

    Eigen::MatrixXd z = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd h; // expected obsevation
    Eigen::MatrixXd measError; // z - h(mu)

    vector<bool> observed;

public:
    ekfslam(int pose_size, int num_landmark, float motion_noise);
    ~ekfslam();

    void motion_model(const Odom &odom);
    void prediction(const Odom &odom);
    void correction(const vector<Laser>& observations);
};

#endif