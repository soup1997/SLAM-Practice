#define PI 3.141592

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <tuple>

using namespace std;

class EKF
{
private:
    Eigen::MatrixXd mu;
    Eigen::MatrixXd S;

    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;

public:
    EKF();
    vector<double> inv_motion_model(vector<Eigen::Vector3d> u_t);
    double normalize_rotation(double theta);
    tuple<Eigen::MatrixXd, Eigen::MatrixXd> prediction(Eigen::MatrixXd mu, Eigen::MatrixXd S, vector<Eigen::Vector3d> u_t);
    tuple<Eigen::MatrixXd, Eigen::MatrixXd> correction(Eigen::MatrixXd mu_pred, Eigen::MatrixXd S_pred, Eigen::MatrixXd z, Eigen::MatrixXd landMark);
};