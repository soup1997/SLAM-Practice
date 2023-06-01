#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

using namespace std;

class ekf
{
private:
    Eigen::MatrixXd mu;
    Eigen::MatrixXd S;

    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;

public:
    ekf();
    ~ekf(){};
    vector<Eigen::MatrixXd> prediction();
    vector<Eigen::MatrixXd> correction();
    void ekf_main();
};

