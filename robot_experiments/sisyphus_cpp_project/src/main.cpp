#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/exception.h>

// 7-Step Cycle for maximum safety
enum class TaskState { 
    MOVE_WAIT, 
    PRE_PICK, 
    PICK, 
    POST_PICK, 
    PRE_PLACE, 
    PLACE, 
    POST_PLACE 
};

int main(int argc, char** argv) {
    try {
        std::string robot_ip = "172.16.0.2";
        franka::Robot robot(robot_ip);
        franka::Gripper gripper(robot_ip);
        franka::Model model = robot.loadModel();

        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << 600.0 * Eigen::Matrix3d::Identity(); 
        stiffness.bottomRightCorner(3, 3) << 20.0 * Eigen::Matrix3d::Identity();
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(600.0) * Eigen::Matrix3d::Identity();
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(20.0) * Eigen::Matrix3d::Identity();

        double z_offset = 0.1134; // Gripper length
        double clearance = 0.1; // 5 cm hover height

        // Define Base Poses
        Eigen::Vector3d pos_pick(0.5205, 0.4232, 0.1312 - z_offset);
        Eigen::Quaterniond ori_pick(0.0249, 0.9205, 0.3899, -0.0028);

        Eigen::Vector3d pos_place(0.5205, -0.0, 0.2221 - z_offset);
        Eigen::Quaterniond ori_place(0.0083, -0.9250, -0.3798, -0.0003);

        Eigen::Vector3d pos_wait(0.5252, 0.2075, 0.5284 - z_offset);
        Eigen::Quaterniond ori_wait(0.0260, 0.9232, 0.3835, -0.0036);

        // Define Hover Poses
        Eigen::Vector3d pos_pre_pick = pos_pick;  pos_pre_pick.z() += clearance;
        Eigen::Vector3d pos_pre_place = pos_place; pos_pre_place.z() += clearance;

        TaskState current_state = TaskState::MOVE_WAIT;
        std::cout << "Sisyphus C++ V5 - High Clearance Waypoints" << std::endl;

        gripper.move(0.08, 0.1);

        while (true) {
            Eigen::Vector3d goal_pos;
            Eigen::Quaterniond goal_ori;
            std::string state_name;
            
            // --- TARGET SELECTION ---
            switch(current_state) {
                case TaskState::MOVE_WAIT: 
                    goal_pos = pos_wait; goal_ori = ori_wait; state_name = "WAIT/MIDDLE"; break;
                case TaskState::PRE_PICK: 
                    goal_pos = pos_pre_pick; goal_ori = ori_pick; state_name = "HOVER ABOVE BALL"; break;
                case TaskState::PICK: 
                    goal_pos = pos_pick; goal_ori = ori_pick; state_name = "GOING DOWN TO PICK"; break;
                case TaskState::POST_PICK: 
                    goal_pos = pos_pre_pick; goal_ori = ori_pick; state_name = "LIFTING BALL"; break;
                case TaskState::PRE_PLACE: 
                    goal_pos = pos_pre_place; goal_ori = ori_place; state_name = "HOVER ABOVE RAMP"; break;
                case TaskState::PLACE: 
                    goal_pos = pos_place; goal_ori = ori_place; state_name = "GOING DOWN TO RELEASE"; break;
                case TaskState::POST_PLACE: 
                    goal_pos = pos_pre_place; goal_ori = ori_place; state_name = "CLEARING RAMP"; break;
            }

            std::cout << ">>> " << state_name << std::endl;

            bool is_paused = false;
            double pause_timer = 0.0;
            Eigen::Vector3d virtual_pos;
            Eigen::Quaterniond virtual_ori;
            bool first_tick = true;

            auto impedance_control = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d current_pos(transform.translation());
                Eigen::Quaterniond current_ori(transform.rotation());
                
                if (first_tick) {
                    virtual_pos = current_pos;
                    virtual_ori = current_ori;
                    first_tick = false;
                }

                Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_forces(robot_state.O_F_ext_hat_K.data());
                double force_magnitude = external_forces.head(3).norm();

                if (force_magnitude > 8.5) { 
                    is_paused = true;
                    pause_timer = 0.0;
                } else if (is_paused) {
                    pause_timer += period.toSec();
                    if (pause_timer > 1.5) is_paused = false;
                }

                double filter = 0.0012; 
                if (is_paused) {
                    virtual_pos = current_pos;
                    virtual_ori = current_ori;
                } else {
                    virtual_pos = (1.0 - filter) * virtual_pos + filter * goal_pos;
                    virtual_ori = virtual_ori.slerp(filter, goal_ori);
                }

                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << current_pos - virtual_pos;
                if (virtual_ori.coeffs().dot(current_ori.coeffs()) < 0.0) { current_ori.coeffs() << -current_ori.coeffs(); }
                Eigen::Quaterniond error_quaternion(current_ori.inverse() * virtual_ori);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                error.tail(3) << -transform.linear() * error.tail(3);

                std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

                Eigen::VectorXd tau_task(7), tau_d(7);
                tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
                tau_d << tau_task + coriolis;

                std::array<double, 7> tau_d_array;
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

                if (!is_paused && (current_pos - goal_pos).norm() < 0.03) {
                    return franka::MotionFinished(franka::Torques(tau_d_array));
                }
                return tau_d_array;
            };

            robot.control(impedance_control);

            // --- STATE LOGIC ---
            switch(current_state) {
                case TaskState::MOVE_WAIT: 
                    std::cout << "Waiting for ball..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    current_state = TaskState::PRE_PICK; break;
                
                case TaskState::PRE_PICK: 
                    current_state = TaskState::PICK; break;
                
                case TaskState::PICK: 
                    std::cout << "Action: GRASPING" << std::endl;
                    gripper.grasp(0.055, 0.1, 2.0, 0.02, 0.02);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    current_state = TaskState::POST_PICK; break;
                
                case TaskState::POST_PICK: 
                    current_state = TaskState::PRE_PLACE; break;
                
                case TaskState::PRE_PLACE: 
                    current_state = TaskState::PLACE; break;
                
                case TaskState::PLACE: 
                    std::cout << "Action: RELEASING" << std::endl;
                    gripper.move(0.08, 0.1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    current_state = TaskState::POST_PLACE; break;
                
                case TaskState::POST_PLACE: 
                    current_state = TaskState::MOVE_WAIT; break;
            }
        }
    } catch (const franka::Exception& e) {
        std::cerr << "Franka Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}