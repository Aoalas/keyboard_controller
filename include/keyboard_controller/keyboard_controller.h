//
// Created by qiayuan on 2/6/21.
//

#ifndef KEYBOARD_CONTROLLER_KEYBOARD_CONTROLLER_H
#define KEYBOARD_CONTROLLER_KEYBOARD_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <thread>
#include <atomic>

namespace keyboard_controller {

    class KeyBoardController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        std::atomic<int> state_; // 使用原子变量以线程安全地更新状态
        std::thread keyboard_thread_; // 键盘监听线程
        bool running_; // 控制线程运行

        bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
                  ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
        void keyboardListener();
    private:
        ros::Time last_change_;
    };
}  //  namespace keyboard_controller

#endif  // KEYBOARD_CONTROLLER_KEYBOARD_CONTROLLER_H
