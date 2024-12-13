//
// Created by qiayuan on 2/6/21.
//

#include <termios.h>
#include "../include/keyboard_controller/keyboard_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <atomic>

namespace keyboard_controller {
    bool KeyBoardController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                       ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        running_ = true;
        state_ = 6; // 初始化为停止状态
        keyboard_thread_ = std::thread(&KeyBoardController::keyboardListener, this);

        return true;
    }

    char getKey() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt); // 获取当前终端属性
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // 禁用行缓冲与回显
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar(); // 读取一个字符
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复终端属性
        return ch;
    }

    void KeyBoardController::keyboardListener() {
        while (running_) {
            char key = getKey(); // 获取按键输入
            switch (key) {
                case 'i': state_ = 0; break;
                case ',': state_ = 1; break;
                case 'j': state_ = 2; break;
                case 'l': state_ = 3; break;
                case 'u': state_ = 4; break;
                case 'o': state_ = 5; break;
                case 'k': state_ = 6; break;
                default: break;
            }
        }
    }

    void KeyBoardController::update(const ros::Time& time, const ros::Duration& period)
    {
        ROS_INFO("Load the command you publish.\n"
                 "---------------------------\n"
                 "Move keys:\n"
                 "   u    i    o\n"
                 "   j    k    l\n"
                 "        ,     \n"
                 "                ");
        double tau = 0.2;
        static double cmd_[7][4] = { { tau, tau, tau, tau },                      //  forward
                                     { -2 * tau, -2 * tau, -2 * tau, -2 * tau },  //  backward
                                     { -tau, tau, tau, -tau },                    //  left
                                     { 2 * tau, -2 * tau, -2 * tau, 2 * tau },    //  right
                                     { 2 * tau, -2 * tau, 2 * tau, -2 * tau },    //  clockwise
                                     { -tau, tau, -tau, tau } ,                   //  counterclockwise
                                     { 0, 0, 0, 0 } };                            //  stop

        front_left_joint_.setCommand(cmd_[state_][0]);
        front_right_joint_.setCommand(cmd_[state_][1]);
        back_left_joint_.setCommand(cmd_[state_][2]);
        back_right_joint_.setCommand(cmd_[state_][3]);
    }

    PLUGINLIB_EXPORT_CLASS(keyboard_controller::KeyBoardController, controller_interface::ControllerBase)
}  // namespace keyboard_controller
