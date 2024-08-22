#include <iostream>
#include <ekf.hpp>

int main(int argc, char** argv) {
    Eigen::MatrixXd mu(3, 1);
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(3, 3);

    vector<Eigen::Vector3d> u_t;
    Eigen::Vector3d u1, u2;

    tuple<Eigen::MatrixXd, Eigen::MatrixXd> pred;

    mu << 2, 2, PI/2;
    S.diagonal() << 1, 1, 1;

    u1 << 0, 0, 0;
    u2 << 1, 0, 0;
    u_t.push_back(u1);
    u_t.push_back(u2);

    auto ekf = EKF();
    pred = ekf.prediction(mu, S, u_t);
    Eigen::MatrixXd mu_pred = get<0>(pred);
    Eigen::MatrixXd S_pred = get<1>(pred);

    cout << mu_pred << "\n\n" << S_pred << endl;

    return 0;
}
