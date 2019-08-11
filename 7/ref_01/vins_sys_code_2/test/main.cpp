#include <iostream>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
int main() {
    Eigen::Matrix<double,2,3> J;
    J(0,0) = 1;
    J(0,1) = 2;
    J(0,2) = 1;
    J(1,0) = 1.5;
    J(1,1) = 0.5;
    J(1,2) = 0.8;

    Eigen::Matrix2d covariance;
    covariance << 0.1, 0,
                    0, 3;
    std::cout<<covariance<<std::endl;
    Eigen::Matrix2d I;
    I = covariance.inverse();


    Eigen::Vector3d delta_x;

    Eigen::Vector2d r;
    r(0) = 0.8;
    r(1) = 1.3;

    Eigen::Matrix3d JIJ = J.transpose() * I * J;
    Eigen::Vector3d b = -J.transpose() * I * r;
    delta_x = JIJ.ldlt().solve(b);
    std::cout<<delta_x<<std::endl;


    std::cout << "Hello, World!" << std::endl;
    return 0;
}