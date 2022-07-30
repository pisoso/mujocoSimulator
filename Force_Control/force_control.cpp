#include "force_control.h"

Force_Control::Force_Control()
{
    // Set K--Siffness Matrix, D--Damping Matrix
    m_M_d = Eigen::MatrixXd::Identity(6,6);
    m_K_d = Eigen::MatrixXd::Identity(6,6);
    m_D_d = Eigen::MatrixXd::Identity(6,6);
}

Force_Control::Force_Control(std::string _urdf_filename)
{
    m_model = new Model;
    // Load the urdf model
    pinocchio::urdf::buildModel(_urdf_filename, *m_model);

    // Create data required by the algorithms
    m_data = new Data(*m_model);

    // Set K--Siffness Matrix, D--Damping Matrix

    m_M_d = Eigen::MatrixXd::Identity(6,6)*2;
    m_K_d = Eigen::MatrixXd::Identity(6,6)*10;
    m_D_d = Eigen::MatrixXd::Identity(6,6)*3;
}

Eigen::VectorXd Force_Control::cartesian_impedence_control_law(const Eigen::VectorXd &q,
                                                               const Eigen::VectorXd &dq,
                                                               const Eigen::VectorXd &F_ext,
                                                               const pinocchio::SE3& oMdes,
                                                               const Eigen::VectorXd &v_end)
{
    Eigen::VectorXd v_end_tar(6);
    v_end_tar.head(3)= v_end;
    v_end_tar.tail(3).setZero();


    // Compute error between desire and current pose
    pinocchio::forwardKinematics(*m_model, *m_data,q);
    const pinocchio::SE3 dMi = oMdes.actInv(m_data->oMi[m_model->nq]);
    Eigen::VectorXd x_err = pinocchio::log6(dMi).toVector();
//    x_err.tail(3).setZero();

//    x_err.head(3) = x_err_new;
//    x_err(3) = 0;
//    x_err(4) = 0;
    x_err(5) = 0;

//    Eigen::Matrix<double, 6, 1> error;
//    error.head(3) << x_err_new;
//    error.tail(3).setZero();

    // Compute Mass Matrix
    crba(*m_model, *m_data, q);

    /* Computes the non-linear effects (Corriolis, centrifual and gravitationnal effects) */
    /* This function is equivalent to pinocchio::rnea(model, data, q, v, 0). */
    /* data.nle=b(q,dq/dt)=C(q,dq/dt)*dq/dt/+G(q) */
    nonLinearEffects(*m_model,*m_data,q,dq);

    // Compute Body Jacobian Matrix
    Eigen::MatrixXd J_b = Eigen::MatrixXd(6,m_model->nq);
    pinocchio::computeJointJacobian(*m_model,*m_data,q,m_model->nq,J_b);
    Eigen::MatrixXd JtJ_b = J_b*J_b.transpose();
    Eigen::MatrixXd J_inv_b = J_b.transpose()*JtJ_b.inverse();

    // Compute Space Jacobian Matrix
    Eigen::MatrixXd J_s = Eigen::MatrixXd(6,m_model->nq);
    pinocchio::computeJointJacobians(*m_model,*m_data,q);
    pinocchio::getJointJacobian(*m_model,*m_data,m_model->nq,WORLD,J_s);
    Eigen::MatrixXd JtJ_s = J_s*J_s.transpose();
    Eigen::MatrixXd J_inv_s = J_s.transpose()*JtJ_s.inverse();

    // Compute Jacobian Time Variation
    Eigen::MatrixXd dJ_b = Eigen::MatrixXd(6,m_model->nv);
    dJ_b.fill(0.0);
    computeJointJacobiansTimeVariation(*m_model,*m_data,q,dq);
    getJointJacobianTimeVariation(*m_model, *m_data, m_model->nq, LOCAL, dJ_b);

//    Eigen::MatrixXd dJ_s = Eigen::MatrixXd(6,m_model->nv);
//    dJ_s.fill(0.0);
//    computeJointJacobiansTimeVariation(*m_model,*m_data,q,dq);
//    getJointJacobianTimeVariation(*m_model, *m_data, m_model->nq, WORLD, dJ_s);

    // Compute tau_d

//    Eigen::VectorXd tau_d = m_data->M * J_inv_s * m_M_d.inverse() * (m_K_d* ( - J_s*dq) - m_D_d*x_err - m_M_d * dJ_s * dq)
//                          + (J_s.transpose() - m_data->M * J_inv_s * m_M_d.inverse()) * F_ext+m_data->nle;

    Eigen::VectorXd tau_d = m_data->M * J_inv_b * m_M_d.inverse() * (m_K_d* ( - J_b*dq) - m_D_d*x_err - m_M_d * dJ_b * dq)
                          + (J_b.transpose() - m_data->M * J_inv_b * m_M_d.inverse()) * F_ext+m_data->nle;

    std::cout << "pinocchio" <<  m_data->nle.transpose() << std::endl;
//    Eigen::VectorXd tau_d =  J.transpose()*(-m_K_d*J*v - m_D_d*x_err);

    return tau_d;
}

void Force_Control::set_M(Eigen::MatrixXd _M)
{
    m_M_d = _M;
}

void Force_Control::set_K(Eigen::MatrixXd _K)
{
    m_K_d = _K;
}

void Force_Control::set_D(Eigen::MatrixXd _D)
{
    m_D_d = _D;
}

