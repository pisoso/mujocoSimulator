#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>



class GMR{

public:
    GMR(){}
    ~GMR(){}

    // 返回的是速度向量（d×1）
    // 其中priors(1*k) mu(2d*k) sigma(2d*2d*k) 包含了速度 data是某个点的位置(d*1)
    Eigen::VectorXd gaussian_mixture_regression(Eigen::VectorXd _data,
                                                std::vector<double> _priors,
                                                std::vector<Eigen::VectorXd> _mu,
                                                std::vector<Eigen::MatrixXd> _sigma);
};
