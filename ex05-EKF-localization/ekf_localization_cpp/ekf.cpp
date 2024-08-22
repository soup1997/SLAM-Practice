#include <ekf.hpp>

EKF::EKF() {
    mu = Eigen::MatrixXd::Zero(3, 1);
    S = Eigen::MatrixXd::Identity(3, 3);

    R = Eigen::MatrixXd::Identity(3, 3);
    R.diagonal() << pow(0.25, 2), pow(0.25, 2), pow(0.25, 2);

    Q = Eigen::MatrixXd::Identity(2, 2);
    Q.diagonal() << pow(0.1, 2), pow(0.1, 2);
}

double EKF::normalize_rotation(double theta) {
    return atan2(sin(theta), cos(theta));
}

vector<double> EKF::inv_motion_model(vector<Eigen::Vector3d> u_t) {
    vector<double> odom;
    
    double trans = sqrt(pow((u_t[1][0] - u_t[0][0]), 2));
    double rot1 = normalize_rotation(atan2((u_t[1][1] - u_t[0][1]), (u_t[1][0] - u_t[0][0])) -u_t[0][2]);
    double rot2 = normalize_rotation(u_t[1][2] - u_t[0][2] - rot1);

    odom.insert(odom.end(), {rot1, trans, rot2});

    return odom;
}

tuple<Eigen::MatrixXd, Eigen::MatrixXd> EKF::prediction(Eigen::MatrixXd mu, Eigen::MatrixXd S, vector<Eigen::Vector3d> u_t) {
    double theta = mu(2, 0);

    vector<double> odom = inv_motion_model(u_t);
    double rot1 = odom[0];
    double trans = odom[1];
    double rot2 = odom[2];
    
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
    G(0, 2) = -trans * sin(theta + rot1); 
    G(1, 2) =  trans * cos(theta + rot1);

    Eigen::MatrixXd mu_pred = Eigen::Vector3d::Zero(3, 1);
    mu_pred(0, 0) = trans * cos(theta + rot1);
    mu_pred(1, 0) = trans * sin(theta + rot1);
    mu_pred(2, 0) = rot1 + rot2;

    mu_pred += mu;

    Eigen::MatrixXd S_pred = G * S * G.transpose() + this->R;

    return make_tuple(mu_pred, S_pred);
}

tuple<Eigen::MatrixXd, Eigen::MatrixXd> EKF::correction(Eigen::MatrixXd mu_pred, Eigen::MatrixXd S_pred, Eigen::MatrixXd z, Eigen::MatrixXd landMark) {
    int j;
    double lx, ly;
    double q, dist;

    Eigen::MatrixXd delta(2, 1);
    Eigen::MatrixXd h(2, 1);
    Eigen::MatrixXd H(2, 3);
    Eigen::MatrixXd z_copy;
    Eigen::MatrixXd K;

    for(auto i = 0; i < z.cols(); i++) {
        j = static_cast<int>(z(2, i));
        lx = landMark(j, 0);
        ly = landMark(j, 1);

        q = pow((lx - mu_pred(0, 0)), 2) + pow((ly - mu_pred(1, 0)), 2);
        dist = sqrt(q);

        delta << lx - mu_pred(0, 0),
                 ly - mu_pred(1, 0);

        h << dist,
             normalize_rotation(atan2(delta(1, 0), delta(0, 0)) - mu_pred(2, 0));

        H << -delta(0, 0) , -delta(1, 0) , 0,
              delta(1, 0) , -delta(0, 0) , -q;

        H *= (1/q);
        K = S_pred * H.transpose() * (H * S_pred * H.transpose() + this->Q).inverse();

        z_copy = z.block(0, i, 2, 1);
        z_copy.resize(2, 1);

        mu_pred = mu_pred + (K * (z_copy - h));
        mu_pred(2, 0) = normalize_rotation(mu_pred(2, 0));
        S_pred = (Eigen::MatrixXd::Identity(3, 3) - K*H) * S_pred;
    }

    return make_tuple(mu_pred, S_pred);
}