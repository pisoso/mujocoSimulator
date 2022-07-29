#include <iostream>
#include "gmr.h"
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <pdf.h>

using namespace std;



Eigen::VectorXd GMR::gaussian_mixture_regression(Eigen::VectorXd _data,
                                                     std::vector<double> _priors,
                                                     std::vector<Eigen::VectorXd> _mu,
                                                     std::vector<Eigen::MatrixXd> _sigma)
{
    // 数据点维度
    int d = _mu[0].size()/2;
    // 簇的数量
    int k = _mu.size();
    // 计算A
    std::vector<Eigen::MatrixXd> A;
    Eigen::MatrixXd A_temp;
    for(int i=0;i<k;i++)
    {
        A_temp = _sigma[i].bottomLeftCorner(d,d) * (_sigma[i].topLeftCorner(d,d)).inverse();
        A.push_back(A_temp);
    }
    // 计算b
    std::vector<Eigen::MatrixXd> b;
    Eigen::MatrixXd b_temp;
    for(int i=0;i<k;i++)
    {
        b_temp = _mu[i].bottomRows(d) - A[i] * _mu[i].topRows(d);
        b.push_back(b_temp);
    }


    std::vector<double> Pxi;
    double Pxi_temp;
    double Pxi_sum = 0;

    PDF prob;

    // 给定输入x，计算每个GMM分量的影响
    for(int i=0;i<k;i++)
    {
        Pxi_temp = _priors[i] * prob.calculate_probability(_data,_mu[i].head(3),_sigma[i].topLeftCorner(3,3));
        Pxi.push_back(Pxi_temp);
        Pxi_sum = Pxi_temp + Pxi_sum;
    }

    std::vector<double> beta;
    for(int i=0;i<k;i++)
    {
        beta.push_back( Pxi[i] / Pxi_sum);   // h_k
    }

    // 计算期望平均值y，给定输入x
    Eigen::VectorXd y;
    y.setZero(d);
    Eigen::VectorXd y_temp;
    for(int i=0;i<k;i++)
    {
        y_temp = beta[i] *(A[i] * _data + b[i]);
        y = y + y_temp;
    }


    return y;

}
