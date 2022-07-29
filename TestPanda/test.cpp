#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"

char error[1000];
mjModel* m;  // MuJoCo model
mjData* d;   // MuJoCo data

// MuJoCo data structures
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

int main(void)
{
    // activate software     激活软件
//    mj_activate("mjkey.txt");

    // load model from file and check for errors
    m = mj_loadXML("/home/scu/workspace/ws_lc/code/MyRobotSimulator/TestDemo/hello.xml", NULL, error, 1000);
    if( !m )
    {
      printf("%s\n", error);
      return 1;
    }

    // make data corresponding to model
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


    while( !glfwWindowShouldClose(window))
    {
        //提前交互式模拟1/60秒
        //假设MuJoCo可以比实时更快地进行模拟，而实时通常可以做到这一点，
        //该循环将按时完成，以便下一帧以60 fps的速度渲染。
        //否则，请添加cpu计时器，并在渲染时退出此循环。
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
           mj_step(m, d);
        }
        // run simulation for 10 seconds
        //       while( d->time<10 )
        //          mj_step(m, d);


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


    // free model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
