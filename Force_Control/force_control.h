#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H
#include <string>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include <Eigen/Core>

using namespace pinocchio;
class Force_Control
{
public:
    Force_Control();
    Force_Control(std::string _urdf_filename);

public:
    Eigen::VectorXd cartesian_impedence_control_law(const Eigen::VectorXd& q,
                                                    const Eigen::VectorXd& dq,
                                                    const Eigen::VectorXd& F_ext,
                                                    const pinocchio::SE3& oMdes,
                                                    const Eigen::VectorXd &v_end);

    void set_M(Eigen::MatrixXd _M);
    void set_K(Eigen::MatrixXd _K);
    void set_D(Eigen::MatrixXd _D);
private:
    Model* m_model;
    Data* m_data;

    Eigen::MatrixXd m_M_d;
    Eigen::MatrixXd m_K_d;
    Eigen::MatrixXd m_D_d;
};

#endif // FORCE_CONTROL_H
