#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

char filename[] = "/home/scu/workspace/ws_lc/code/MyRobotSimulator/panda_xml/assets/env.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera 抽象摄影机
mjvOption opt;                      // visualization options 可视化选项
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction 鼠标交互
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables 控制器相关变量
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback 键盘回调
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
   // backspace: reset simulation 退格：重置模拟
   if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
   {
       mj_resetData(m, d);
       mj_forward(m, d);
   }
}

// mouse button callback 鼠标按钮回调
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
   // update button state 更新按钮状态
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

   // compute mouse displacement, save 计算鼠标位移，保存
   double dx = xpos - lastx;
   double dy = ypos - lasty;
   lastx = xpos;
   lasty = ypos;

   // get current window size
   int width, height;
   glfwGetWindowSize(window, &width, &height);

   // get shift key state 获取shift键状态
   bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

   // determine action based on mouse button 基于鼠标按钮确定操作
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


// scroll callback 滚动回调
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
   // emulate vertical mouse motion = 5% of window height 模拟垂直鼠标运动=窗口高度的5%
   mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv)
{

   // activate software     激活软件
   mj_activate("mjkey.txt");


   // load and compile model 加载和编译模型
   char error[1000] = "Could not load binary model";

   // check command-line arguments 检查命令行参数
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

   // create window, make OpenGL context current, request v-sync  创建窗口，将OpenGL上下文设置为当前，请求v-sync
   GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
   glfwMakeContextCurrent(window);
   glfwSwapInterval(1);

   // initialize visualization data structures 初始化可视化数据结构
   mjv_defaultCamera(&cam);
   mjv_defaultOption(&opt);
   mjv_defaultScene(&scn);
   mjr_defaultContext(&con);
   mjv_makeScene(m, &scn, 2000);                // space for 2000 objects 2000个对象的空间
   mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context 特定于模型的上下文

   // install GLFW mouse and keyboard callbacks 安装GLFW鼠标和键盘回调
   glfwSetKeyCallback(window, keyboard);
   glfwSetCursorPosCallback(window, mouse_move);
   glfwSetMouseButtonCallback(window, mouse_button);
   glfwSetScrollCallback(window, scroll);

   // 改变相机初始视角
   // double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
   // cam.azimuth = arr_view[0];
   // cam.elevation = arr_view[1];
   // cam.distance = arr_view[2];
   // cam.lookat[0] = arr_view[3];
   // cam.lookat[1] = arr_view[4];
   // cam.lookat[2] = arr_view[5];

   // use the first while condition if you want to simulate for a period.
   // 如果要模拟一段时间，请使用第一个while条件。
   while( !glfwWindowShouldClose(window))
   {
       // advance interactive simulation for 1/60 sec
       //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
       //  this loop will finish on time for the next frame to be rendered at 60 fps.
       //  Otherwise add a cpu timer and exit this loop when it is time to render.
       //提前交互式模拟1/60秒
       //假设MuJoCo可以比实时更快地进行模拟，而实时通常可以做到这一点，
       //该循环将按时完成，以便下一帧以60 fps的速度渲染。
       //否则，请添加cpu计时器，并在渲染时退出此循环。
       mjtNum simstart = d->time;
       while( d->time - simstart < 1.0/60.0 )
       {
           mj_step(m, d);
       }

      // get framebuffer viewport 获取帧缓冲区视口
       mjrRect viewport = {0, 0, 0, 0};
       glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

       // update scene and render 更新场景并渲染
       mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
       mjr_render(viewport, &scn, &con);
       //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

       // swap OpenGL buffers (blocking call due to v-sync) 交换OpenGL缓冲区（由于v-sync而阻止调用）
       glfwSwapBuffers(window);

       // process pending GUI events, call GLFW callbacks 处理挂起的GUI事件，调用GLFW回调
       glfwPollEvents();

   }

   // free visualization storage 免费可视化存储
   mjv_freeScene(&scn);
   mjr_freeContext(&con);

   // free MuJoCo model and data, deactivate 免费的 MuJoCo 模型和数据，停用
   mj_deleteData(d);
   mj_deleteModel(m);
   mj_deactivate();

   // terminate GLFW (crashes with Linux NVidia drivers) 终止 GLFW（使用 Linux NVidia 驱动程序崩溃）
   #if defined(__APPLE__) || defined(_WIN32)
       glfwTerminate();
   #endif

   return 1;
}

