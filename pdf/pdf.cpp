#include <iostream>
#include "pdf.h"
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>

using namespace std;



double PDF::calculate_probability(Eigen::VectorXd _data, Eigen::VectorXd _mu, Eigen::MatrixXd _sigma)
{
    // 数据点维度
    double d = _mu.size();

    Eigen::VectorXd data_diff;
    data_diff = _data - _mu;

    Eigen::VectorXd prob_temp;
    prob_temp =  data_diff.transpose() *_sigma.inverse() * data_diff;

    double prob;
    prob = prob_temp(0);

    prob =exp(-0.5 * prob) / sqrt(pow(2*M_PI,d) * abs(_sigma.determinant()));////////////////

    return prob;
}
