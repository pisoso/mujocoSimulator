#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

//#include <pinocchio/parsers/urdf.hpp>

//#include <pinocchio/algorithm/joint-configuration.hpp>
//#include <pinocchio/algorithm/kinematics.hpp>
//#include "pinocchio/algorithm/rnea.hpp"
//#include "pinocchio/algorithm/crba.hpp"
//#include "pinocchio/algorithm/jacobian.hpp"
#include <iostream>
#include <random>
#include <force_control.h>


#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include "string.h"
#include <iostream>
#include <Eigen/Dense>
#include <gmr.h>


//simulation end time
double simend = 100;

#define nv_mujoco 7

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file

//char xmlpath[] = "/home/scu/workspace/ws_lc/code/MyRobotSimulator/panda_xml/old/assets/panda_peg_in_hole.xml";
//char xmlpath[] = "/home/scu/workspace/ws_lc/code/MyRobotSimulator/panda_xml_github/panda_mujoco/my_panda.xml";
char xmlpath[] = "/home/scu/workspace/ws_lc/code/mujocoSimulator/panda_xml/new/my_panda.xml";
char datapath[] = "/home/scu/workspace/ws_lc/code/mujocoSimulator/panda_xml_github/panda_mujoco/data.csv";

char datafile[] = "data.csv";


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjvOption vopt;
//mjuiState uistate;
mjvPerturb pert;




// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
int camera = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void choice_model_part(mjuiState* state){

    if (state->type==mjEVENT_PRESS && state->mouserect==3 && m) {
      // set perturbation
      int newperturb = 0;
      if (state->control && pert.select>0) {
        // right: translate;  left: rotate
        if (state->right) {
          newperturb = mjPERT_TRANSLATE;
        } else if (state->left) {
          newperturb = mjPERT_ROTATE;
        }

        // perturbation onset: reset reference
        if (newperturb && !pert.active) {
          mjv_initPerturb(m, d, &scn, &pert);
        }
      }
      pert.active = newperturb;

      // handle double-click
      if (state->doubleclick) {
        // determine selection mode
        int selmode;
        if (state->button==mjBUTTON_LEFT) {
          selmode = 1;
        } else if (state->control) {
          selmode = 3;
        } else {
          selmode = 2;
        }

        // find geom and 3D click point, get corresponding body
        mjrRect r = state->rect[3];
        mjtNum selpnt[3];
        int selgeom, selskin;
        int selbody = mjv_select(m, d, &vopt,
                                 (mjtNum)r.width/(mjtNum)r.height,
                                 (mjtNum)(state->x-r.left)/(mjtNum)r.width,
                                 (mjtNum)(state->y-r.bottom)/(mjtNum)r.height,
                                 &scn, selpnt, &selgeom, &selskin);

        // set lookat point, start tracking is requested
        if (selmode==2 || selmode==3) {
          // copy selpnt if anything clicked
          if (selbody>=0) {
            mju_copy3(cam.lookat, selpnt);
          }

        }

        // set body selection
        else {
          if (selbody>=0) {
            // record selection
            pert.select = selbody;
            pert.skinselect = selskin;

            // compute localpos
            mjtNum tmp[3];
            mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
            mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
          } else {
            pert.select = 0;
            pert.skinselect = -1;
          }
        }

        // stop perturbation on select
        pert.active = 0;
      }

      return;
    }
}

//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
    //write name of the variable here (header)
    fprintf(fid,"t, ");
    fprintf(fid,"PE, KE, TE, "); // PE为势能，KE为动能，TE为总的能量
    fprintf(fid,"q1, q2, ");

    //Don't remove the newline
    fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
    //data here should correspond to headers in init_save_data()
    //seperate data by a space %f followed by space
    fprintf(fid,"%f, ",d->time);
    fprintf(fid,"%f, %f, %f, ",d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);
    fprintf(fid,"%f, %f ",d->qpos[0],d->qpos[1]);
    //Don't remove the newline
    fprintf(fid,"\n");
}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{
    //write control here
    mj_energyPos(m,d);
    mj_energyVel(m,d);
    // 分别打印势能和动能
    //  printf("%f %f %f %f \n",d->time,d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);

    //check equations
    //M*qacc + qfrc_bias = qfrc_applied + ctrl
    //M*qddot + f = qfrc_applied + ctrl

    // 计算惯性矩阵 M(q)
    double dense_M[nv_mujoco*nv_mujoco] = {0};
    mj_fullM(m,dense_M, d->qM);
    double M[nv_mujoco][nv_mujoco]={0};

    // 计算关节加速度
    double qddot[nv_mujoco]={0};
    for(int i=0;i<nv_mujoco;i++){
        qddot[i]=d->qacc[i];
    }

    // 计算科氏力 C 和重力 G
    double f[nv_mujoco]={0};
    std::cout << "科室里" << std::endl;
    for(int i=0;i<nv_mujoco;i++){
        f[i] = d->qfrc_bias[i];
        std::cout << f[i] << ",  ";
    }
    std::cout << std::endl;

    double position[nv_mujoco]={0};
    std::cout << "位置" << std::endl;
    for(int i=0;i<nv_mujoco;i++){
        position[i] = d->qpos[i];
        std::cout << position[i] << ",  ";
    }
    std::cout << std::endl;

    double vel[nv_mujoco]={0};
    std::cout << "速度" << std::endl;
    for(int i=0;i<nv_mujoco;i++){
        vel[i] = d->qvel[i];
        std::cout << vel[i] << ",  ";
    }
    std::cout << std::endl;




    double lhs[nv_mujoco]={0};
    mju_mulMatVec(lhs,dense_M,qddot,2,2); //lhs = M*qddot    // mju_mulMatVec将矩阵和向量相乘
    for(int i=0;i<nv_mujoco;i++){
        lhs[i] = lhs[i] + f[i]; //lhs = M*qddot + f
    }

    // qfrc_applied表示施加的广义力
    for(int i=0;i<nv_mujoco;i++){
        d->qfrc_applied[i] = 0.5*f[i];
    }

    // double rhs[nv_mujoco]={0};
    // rhs[0] = d->qfrc_applied[0];
    // rhs[1] = d->qfrc_applied[1];

    // printf("%f %f \n",lhs[0], rhs[0]);
    // printf("%f %f \n",lhs[1], rhs[1]);
    // printf("******\n");

    //control
    double Kp1 = 100, Kp2 = 10;
    double Kv1 = 9, Kv2 = 3;


    double qref[nv_mujoco] = {0,-M_PI/4,0,-3*M_PI/4,0,M_PI/2,M_PI/4}; // 期望位置
//    double qref[nv_mujoco] = {0.1,0.1,0,0,0,0,0.1}; // 期望位置

    //PD control
//    for(int i=0;i<nv_mujoco;i++){
//        d->qfrc_applied[i] = -Kp2*(d->qpos[i]-qref[i])-Kv2*d->qvel[i];
//    }

    //coriolis + gravity + PD control
    for(int i=0;i<nv_mujoco;i++){
        d->qfrc_applied[i] = f[i]-Kp2*(d->qpos[i]-qref[i])-Kv2*d->qvel[i];
//        d->ctrl[i] = -Kp2*(d->qpos[i]-qref[i])-Kv2*d->qvel[i];
    }

    //  //Feedback linearization
    //  //M*(-kp( ... ) - kv(...) + f)
    //  double tau[2]={0};
    //  tau[0]=-Kp1*(d->qpos[0]-qref1)-Kv1*d->qvel[0];
    //  tau[1]=-Kp2*(d->qpos[1]-qref2)-Kv2*d->qvel[1];

    //  mju_mulMatVec(tau,dense_M,tau,2,2); //lhs = M*tau
    //  tau[0] += f[0];
    //  tau[1] += f[1];
    //    d->qfrc_applied[0] = tau[0];
    //    d->qfrc_applied[1] = tau[1];


    //write data here (dont change/dete this function call; instead write what you need to save in save_data)
    if ( loop_index%data_frequency==0)
    {
        save_data(m,d);
    }
    loop_index = loop_index + 1;
}



//************************
// main function
int main(int argc, const char** argv)
{


    const std::string urdf_filename = (argc<=1) ? std::string("/home/scu/workspace/ws_lc/code/mujocoSimulator/panda_urdf/panda.urdf") : argv[1];
    Model* model_pinocchio = new Model;
    pinocchio::urdf::buildModel(urdf_filename, *model_pinocchio);
    Data* data_pinocchio = new Data(*model_pinocchio);

    // Compliance parameters
//    const double translational_stiffness{100.0};
//    const double rotational_stiffness{10.0};
//    Eigen::MatrixXd stiffness(3, 3), damping(3, 3);
//    stiffness.setZero();
//    stiffness << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
//    damping.setZero();
//    damping << 2.0 * sqrt(translational_stiffness) *
//               Eigen::MatrixXd::Identity(3, 3);

    // Compliance parameters
    const double translational_stiffness{1000};
    const double rotational_stiffness{500};
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                       Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                           Eigen::MatrixXd::Identity(3, 3);

//    damping.topLeftCorner(3, 3)= Eigen::MatrixXd::Identity(3, 3) * 10;


    Force_Control m_force_control(urdf_filename);

    m_force_control.set_D(stiffness);
    m_force_control.set_K(damping);

    Eigen::Matrix<double, 4, 4> T;
    T<<0.707110,-0.707110,-0.000000,0.306891,
    -0.707110,-0.707110,0.000005,0.000005,
    -0.000000,-0.000005,-1.000000,0.697282,
    0.000000,0.000000, 0.000000,1.000000;



    int K=4;

    Eigen::VectorXd data_now(K);
    std::vector<double> priors(K);
    std::vector<Eigen::VectorXd> mu(K,Eigen::VectorXd(6));
    std::vector<Eigen::MatrixXd> sigma(K,Eigen::MatrixXd(6,6));

    priors[0] = 0.254391254;
    priors[1] = 0.216812776;
    priors[2] = 0.251428059;
    priors[3] = 0.277367911;



    mu[0] <<0.00516756002,
            0.06859017298,
            0.03047168260,
            -0.0096501562,
            -0.0684553314,
            -0.0422899274;

    mu[1] <<0.02744231941,
            0.40851487314,
            0.05151297796,
            -6.1030866960,
            -5.5716600752,
            3.16318561774;

    mu[2] <<0.03620945788,
            0.10955338299,
            0.07489201254,
            1.60582359241,
            -1.9769843997,
            0.12091899855;

    mu[3] <<0.03005254951,
            0.12622399517,
            0.06108606606,
            -0.0236326819,
            -0.1517876322,
            -0.0405949619;

    sigma[0] << 0.0262446558279427,0.0583901345697913,-0.0803293512693164,-0.0395966589334242,-0.0222644754201621,0.00609443778328629,
                0.0583901345697913,0.604435188742620,-0.332159834484945,0.276508675660526,-0.385266191662396,-0.180942359488535,
                -0.0803293512693164,-0.332159834484945,1.04968472065204,-0.529257330457612,-0.0990869189654551,-0.189746696489499,
                -0.0395966589334242,0.276508675660526,-0.529257330457612,0.723166926888016,0.0226408427904180,-0.0650647833937457,
                -0.0222644754201621,-0.385266191662396,-0.0990869189654552,0.0226408427904180,1.83105785144621,-1.37291182064493,
                0.00609443778328629,-0.180942359488535,-0.189746696489499,-0.0650647833937457,-1.37291182064493,2.11431229112121;

    sigma[1] << 0.000296260682656471,-0.000193033280362795,0.000630245710385136,-0.000490854894224850,-0.000138832880048130,-0.00135527534529982,
                -0.000193033280362795,0.000129179854183917,-0.000415735963869859,0.000270924372261687,5.56476588907543e-05,0.000924069441915855,
                0.000630245710385136,-0.000415735963869859,0.00142323166911585,-0.000963525077770972,-0.000385993109103294,-0.00316378020737185,
                -0.000490854894224850,0.000270924372261687,-0.000963525077770972,0.00561093186936438,0.00391415437523878,2.02596329981129e-05,
                -0.000138832880048130,5.56476588907543e-05,-0.000385993109103294,0.00391415437523878,0.00330231029516970,-0.000626202503347872,
                -0.00135527534529982,0.000924069441915855,-0.00316378020737185,2.02596329981129e-05,-0.000626202503347872,0.00797558943306820;

    sigma[2] << 1.62905937277927e-06,7.95199965714710e-06,1.76669830105120e-05,1.26871678604903e-05,-5.98132722989979e-05,-3.23729967345142e-05,
                7.95199965714710e-06,0.000126385811791026,0.000354911570155319,-6.00814169608908e-05,-0.000434048471478416,-0.000854347480332457,
                1.76669830105120e-05,0.000354911570155319,0.00104427175583082,-0.000549251064581322,-0.000892410522461579,-0.00256525169059996,
                1.26871678604903e-05,-6.00814169608908e-05,-0.000549251064581322,0.00546685618057053,0.000503638448256395,0.000870538648419092,
                -5.98132722989979e-05,-0.000434048471478416,-0.000892410522461579,0.000503638448256395,0.00851988178457502,0.000628099512880570,
                -3.23729967345142e-05,-0.000854347480332457,-0.00256525169059996,0.000870538648419092,0.000628099512880570,0.00670027146509008;

    sigma[3] << 0.000416637904977934,0.00204958948461731,0.000809667432723921,-0.000288234368076663,-0.00235658013434488,-0.000414659411087300,
                0.00204958948461731,0.0122907145882880,0.00355527945993862,-0.00125316688412478,-0.0133614441705645,-0.000699599446070683,
                0.000809667432723921,0.00355527945993862,0.00167388393051032,-0.000577700745605668,-0.00424730173082872,-0.00109390252614864,
                -0.000288234368076663,-0.00125316688412478,-0.000577700745605668,0.00111297014002095,0.00368208805907792,0.00192521067828774,
                -0.00235658013434488,-0.0133614441705645,-0.00424730173082872,0.00368208805907792,0.0204035421868748,0.00549642487552357,
                -0.000414659411087300,-0.000699599446070683,-0.00109390252614864,0.00192521067828774,0.00549642487552357,0.00503662945199089;

    GMR gmr_vec;




    //// mujoco
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);



    // make data
    d = mj_makeData(m);

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {180, -10, 3, 0.000000, 0.000000, 0.500000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    //改变重力加速度
    m->opt.gravity[2] = -9.81;

    // install control callback
//    mjcb_control = mycontroller;

    fid = fopen(datapath,"w");
    init_save_data();

    //设置起始位置
    double qstart[nv_mujoco] = {M_PI/8,-M_PI/4,0,-3*M_PI/4,0,M_PI/2,M_PI/4}; // 起始位置
    for(int i=0;i<nv_mujoco;i++){
        d->qpos[i] = qstart[i];
    }
    Eigen::Matrix<double, 7, 1> q_start_matrix;
    q_start_matrix << M_PI/8,-M_PI/4,0,-3*M_PI/4,0,M_PI/2,M_PI/4;


//    Eigen::Quaterniond orientation_d(data_pinocchio->oMi[model_pinocchio->nq].linear());

//    Eigen::VectorXd x_end = data_pinocchio->oMi[model_pinocchio->nq].translation();

    Eigen::VectorXd x_tar(3);
    x_tar << 0.245041801526,-0.275159165214,0.55345021317;
//     x_tar <<0.245041801526,-0.275159165214,0.443345021317;

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        int num_count =0;
        while( d->time - simstart < 1.0/30 )
        {
            mj_step(m, d);

            //write control here
            mj_energyPos(m,d);
            mj_energyVel(m,d);


            //check equations
            //M*qacc + qfrc_bias = qfrc_applied + ctrl
            //M*qddot + f = qfrc_applied + ctrl

            //pinocchio
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(d->qpos);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(d->qvel);
//            for(int i=0;i<nv_mujoco;i++){
//                q(i)=d->qpos[i];
//                v(i)=d->qvel[i];
//            }

            pinocchio::forwardKinematics(*model_pinocchio, *data_pinocchio,q);

            Eigen::Vector3d x_end = data_pinocchio->oMi[model_pinocchio->nq].translation();

            std::cout << "x:" << x_end << std::endl;

            // 这坨仅仅是显示用
            Eigen::MatrixXd J_s = Eigen::MatrixXd(6,model_pinocchio->nq);
            pinocchio::computeJointJacobians(*model_pinocchio,*data_pinocchio,q);
            pinocchio::getJointJacobian(*model_pinocchio,*data_pinocchio,model_pinocchio->nq,WORLD,J_s);
            Eigen::VectorXd q_now(7);
            Eigen::VectorXd v_now(6);
            for(int i=0;i<7;i++){
                q_now(i) = d->qvel[i];
            }
            v_now = J_s * q_now;
            std::cout << "v:" << v_now << std::endl;


            x_end = x_end - x_tar;

            std::cout << "x_change:" << x_end << std::endl;

            Eigen::Vector3d v_tar = gmr_vec.gaussian_mixture_regression(x_end,priors,mu,sigma);

            std::cout << "v_tar:" << v_tar << std::endl;



            // 施加外力
            // Get external Force
            Eigen::VectorXd F_ext = Eigen::MatrixXd::Random(6,1) * 0;
////            Eigen::VectorXd F_ext = Eigen::MatrixXd::Random(3,1) * 0;
//            if(d->time <1.1 && d->time>1){
////                F_ext(0) =10;
//                F_ext(2) =20;
//            }

//            Eigen::VectorXd tau_ext = Eigen::MatrixXd::Random(7,1) * 0;
//            if(d->time <1.05 && d->time>1){
//                tau_ext(1) = 2;
//            }


            Eigen::Vector3d position_d = data_pinocchio->oMi[model_pinocchio->nq].translation();
            position_d = position_d + 0.02 * v_tar;

            const pinocchio::SE3 oMdes(T.topLeftCorner<3,3>(),position_d);
            Eigen::VectorXd tau_d = m_force_control.cartesian_impedence_control_law(q,dq,F_ext,oMdes,v_tar);

            // 计算科氏力 C 和重力 G
            double f[nv_mujoco]={0};
//            std::cout << "科室里" << std::endl;
            for(int i=0;i<nv_mujoco;i++){
                f[i] = d->qfrc_bias[i];
//                std::cout << f[i] << ",  ";
            }
            std::cout << std::endl;


            std::cout << "mujoco" << std::endl;
            //coriolis + gravity + PD control
            for(int i=0;i<nv_mujoco;i++){

                d->ctrl[i] = tau_d(i);
//                d->ctrl[i] = d->qfrc_bias[i];
                std::cout <<  d->qfrc_bias[i] << ",";
            }
              std::cout <<   std::endl;



            num_count++;
                std::cout << d->time << "," << d->time-simstart << "," << num_count << std::endl;

        }

        if (d->time>=simend)
        {
            fclose(fid);
            break;
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo  and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
