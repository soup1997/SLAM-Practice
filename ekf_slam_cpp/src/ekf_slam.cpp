#include <ekf/ekf_slam.hpp>

ekfslam::ekfslam(int pose_size, int num_landmark, float motion_noise) : _N(pose_size), _lm(num_landmark), _dim(_N + (2 * _lm))
{
    mu = Eigen::MatrixXd::Zero(_dim, 1);
    Sigma = Eigen::MatrixXd::Zero(_dim, _dim);

    mu_pred = Eigen::MatrixXd::Zero(_dim, 1);
    Sigma_pred = Eigen::MatrixXd::Zero(_dim, _dim);

    lm_sig = INF * Eigen::MatrixXd::Identity((2 * _lm), (2 * _lm));
    Sigma.bottomRightCorner((2 * _lm), (2 * _lm)) = lm_sig;

    R = Eigen::MatrixXd::Zero(_dim, _dim);
    R(0, 0) = motion_noise;
    R(1, 1) = motion_noise;
    R(2, 2) = motion_noise / 10;

    z = Eigen::MatrixXd::Zero(2, 1);
    h = Eigen::MatrixXd::Zero(2, 1);
    measError = Eigen::MatrixXd::Zero(2, 1);
}

void ekfslam::motion_model(const Odom &odom)
{
    double theta = mu(2);
    double r1 = odom.r1;
    double t = odom.t;
    double r2 = odom.r2;

    mu_pred(0) += t * cos(theta + r1);
    mu_pred(1) += t * sin(theta + r1);
    mu_pred(2) += r1 + r2;
}

void ekfslam::prediction(const Odom &odom)
{
    motion_model(odom);

    double theta = mu(2);
    double r1 = odom.r1;
    double t = odom.t;
    double r2 = odom.r2;

    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(_dim, _dim);
    G(0, _N - 1) = -t * sin(theta + r1);
    G(1, _N - 1) = t * cos(theta + r1);

    Sigma_pred = (G * Sigma * G.transpose()) + R;
}

void ekfslam::correction(const vector<Laser> &observations)
{
    
    long long id;
    double range;
    double bearing;

    double x_pred = mu_pred(0);
    double y_pred = mu_pred(1);
    double theta_pred = mu_pred(2);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero((2 * _lm), _dim);

    for (auto i = 0; i < observations.size(); i++)
    {
        Laser landmark = observations[i];

        id = landmark.id;
        range = landmark.range;
        bearing = landmark.bearing;

        Eigen::MatrixXd delta = Eigen::MatrixXd::Zero(2, 1);

        if(!observed[id - 1]) {
            mu_pred(2*id + 1) = x_pred + range*cos(bearing + theta_pred);
            mu_pred(2*id + 2) = y_pred + range*sin(bearing + theta_pred);
            observed[id - 1] = true;
        }
        
        z(0) = range;
        z(1) = bearing;

        delta(0) = mu_pred(2*id + 1) - x_pred;
        delta(1) = mu_pred(2*id + 2) - y_pred;
        double q = pow(delta(0), 2) + pow(delta(1), 2);

        h(0) = sqrt(q);
        h(1) = atan2(delta(1), delta(0) - theta_pred);

        measError = z - h;
    }
}
