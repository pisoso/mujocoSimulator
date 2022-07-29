# 需要cmake的最低版本
cmake_minimum_required(VERSION 3.5)

# 设置目标名称
set( TARGET_NAME "TestDemo" )

# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCE_FILES "*.cpp" "*.c" )
# 把.cpp和.c文件都塞到HEADER_FILES里
file(GLOB HEADER_FILES "*.h" "*.hpp" )

# 添加可执行文件
add_executable(main main.cpp)
add_executable(seds_test seds_test.cpp)



# 链接其他的库
target_link_libraries(main
        PRIVATE
        pinocchio::pinocchio
        glfw
        mujoco
        Force_Control
        gmr
)

# 链接其他的库
target_link_libraries(seds_test
        PRIVATE
        pinocchio::pinocchio
        glfw
        mujoco
        Force_Control
        gmr

)
