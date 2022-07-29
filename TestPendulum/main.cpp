#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


char filename[] = "/home/scu/workspace/ws_lc/code/MyRobotSimulator/TestPendulum/pendulum.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

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



//  ---------
void set_torque_control(const mjModel* m,int _actuator_num,int _flag){
      if (_flag==0)
        m->actuator_gainprm[10*_actuator_num+0] = 0;
      else
        m->actuator_gainprm[10*_actuator_num+0] = 1;
}

// 重新设置kp值
void set_position_servo(const mjModel* m,int _actuator_num,double _kp){
    m->actuator_gainprm[10*_actuator_num + 0] = _kp;
    m->actuator_biasprm[10*_actuator_num + 1] = -_kp;

}

void set_velocity_servo(const mjModel* m, int _actuator_num, double _kv)
{
  m->actuator_gainprm[10*_actuator_num+0] = _kv;
  m->actuator_biasprm[10*_actuator_num+2] = -_kv;
}



void mycontroller(const mjModel* m,mjData* d)
{
    int actuator_num;

    //ctrl的维度为nu*1,nu为执行器/控件的数量，这里为3
    //1. ctrl[0]为力矩执行机构（torque actuator）
//    actuator_num = 0;
//    int flag = 1;
//    set_torque_control(m, actuator_num, flag);
//    d->ctrl[0] = 10*(0-d->qpos[0])-1*d->qvel[0];// PD control
    //读取传感器的当前位置和速度，但因为设置了噪声，会有误差
//    d->ctrl[0] = 10*(0-d->sensordata[0])-1*d->sensordata[1];// PD control

    //2. ctrl[1]为位置伺服机构（ position servo ）
//    actuator_num = 1;
//    double kp=5;
//    set_position_servo(m,actuator_num,kp);
//    for(unsigned int i=0;i<10;i++){
//        std::cout << m->actuator_gainprm[10*actuator_no +i ] << std::endl;
//        std::cout << m->actuator_biasprm[10*actuator_no +i ] << std::endl;
//    }
//    std::cout << "----------------" << std::endl;
//    d->ctrl[1] = 0.5; //比例增益

    //3. ctrl[2]为速度伺服机构（ velocity servo ）
//    actuator_num = 2;
//    double kv=3;
//    set_velocity_servo(m,actuator_num,kv);
//    d->ctrl[2] = 0.2;

    //4. PD control
    actuator_num = 1;
    double kp=10;
    set_position_servo(m,actuator_num,kp);

    actuator_num = 2;
    double kv=1;
    set_velocity_servo(m,actuator_num,kv);
//    d->ctrl[1] = -0.5;
//    d->ctrl[2] = 0;

}


// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


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

    //设置相机的位置
     double arr_view[] = {90, -5, 5, 0.000000, 0.000000, 1};
     cam.azimuth = arr_view[0];
     cam.elevation = arr_view[1];
     cam.distance = arr_view[2];
     cam.lookat[0] = arr_view[3];
     cam.lookat[1] = arr_view[4];
     cam.lookat[2] = arr_view[5];

//     //改变重力加速度
//     m->opt.gravity[2] = -1;

     //钟摆的起始位置
     //只有一个关节限制，设置起始关节角度为pi/2
     //qpos 的维度为nq×1=1×1,1个转动
     d->qpos[0] = 1.57;// pi/2

     // PD控制器回调函数
     mjcb_control = mycontroller;
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
//        // 显示世界坐标系
//        opt.frame = mjFRAME_WORLD;

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

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

