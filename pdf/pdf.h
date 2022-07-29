#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>



class PDF{

public:
    PDF(){}
    ~PDF(){}

    // 其中mu(d*1) sigma(d*d) 和数据(d*1)都只是位置部分，没加速度
    //
    double calculate_probability(Eigen::VectorXd _data,
                                 Eigen::VectorXd _mu,
                                 Eigen::MatrixXd _sigma);
};
