#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/exception.h>

struct Waypoint {
    std::string name;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    double duration; 
    bool grasp_after = false;
    bool release_after = false;
};

int main(int argc, char** argv) {
    try {
        std::string robot_ip = "172.16.0.2";
        franka::Robot robot(robot_ip);
        franka::Gripper gripper(robot_ip);

        Eigen::Quaterniond down_ori(0.0, 1.0, 0.0, 0.0);

        std::vector<Waypoint> path = {
            {"PRE-PICK",  {0.5546, -0.0486, 0.2273}, down_ori, 4.0}, 
            {"PICK",      {0.5555, -0.0513, 0.0571}, down_ori, 2.0, true, false}, 
            {"POST-PICK", {0.4536, 0.3823, 0.5087}, down_ori, 3.0},
            {"PRE-PLACE", {0.2456, 0.6113, 0.2719}, down_ori, 4.0},
            {"PLACE",     {0.2456, 0.6113, 0.0691}, down_ori, 2.0, false, true},  
            {"CLEARANCE", {0.2456, 0.6113, 0.2719}, down_ori, 2.0}
        };

        std::array<double, 7> home_pos = {{-0.0001, -0.7852, 0.0002, -2.3559, 0.0007, 1.5711, 0.7851}};

        std::cout << "Starting Pure C++ Pick and Place (Strict Position Control)..." << std::endl;
        gripper.move(0.08, 0.1);

        // --- EXECUTE CARTESIAN PATH ---
        for (const auto& point : path) {
            std::cout << ">>> Moving to: " << point.name << std::endl;

            Eigen::Vector3d start_pos;
            Eigen::Quaterniond start_ori;
            bool first_tick = true;
            double time = 0.0;
            
            robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
                time += period.toSec();
                
                // CRITICAL FIX: Capture the exact Commanded pose inside the loop
                if (first_tick) {
                    // Notice the _c below! This ensures 0.0 acceleration on the first tick.
                    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data()));
                    start_pos = initial_transform.translation();
                    start_ori = Eigen::Quaterniond(initial_transform.rotation());
                    first_tick = false;
                }

                double u = (time >= point.duration) ? 1.0 : 0.5 * (1.0 - std::cos(M_PI * time / point.duration));

                Eigen::Vector3d current_target_pos = start_pos + u * (point.pos - start_pos);
                Eigen::Quaterniond current_target_ori = start_ori.slerp(u, point.ori);

                Eigen::Affine3d target_transform = Eigen::Affine3d::Identity();
                target_transform.translation() = current_target_pos;
                target_transform.linear() = current_target_ori.toRotationMatrix();

                std::array<double, 16> pose_array;
                Eigen::Map<Eigen::Matrix4d> pose_map(pose_array.data());
                pose_map = target_transform.matrix();

                if (time >= point.duration) {
                    return franka::MotionFinished(franka::CartesianPose(pose_array));
                }
                return franka::CartesianPose(pose_array);
            });

            // Action Phase
            if (point.grasp_after) {
                std::cout << "Action: Grasping object..." << std::endl;
                // width [m], speed [m/s], force [Newtons], epsilon_inner [m], epsilon_outer [m]
                gripper.grasp(0.04, 0.1, 40.0, 0.02, 0.02);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else if (point.release_after) {
                std::cout << "Action: Releasing object..." << std::endl;
                gripper.move(0.08, 0.1);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // --- RETURN TO HOME (JOINT CONTROL) ---
        std::cout << "\n>>> Returning to Safe Home Position..." << std::endl;
        std::array<double, 7> start_q;
        bool home_tick = true;
        double time_j = 0.0;
        double home_duration = 5.0;

        robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
            time_j += period.toSec();
            
            // CRITICAL FIX: Capture commanded joints (q_c) on first tick
            if (home_tick) {
                start_q = robot_state.q_d; 
                home_tick = false;
            }

            double u = (time_j >= home_duration) ? 1.0 : 0.5 * (1.0 - std::cos(M_PI * time_j / home_duration));
            
            std::array<double, 7> current_q;
            for(size_t i = 0; i < 7; i++) {
                current_q[i] = start_q[i] + u * (home_pos[i] - start_q[i]);
            }

            if (time_j >= home_duration) {
                return franka::MotionFinished(franka::JointPositions(current_q));
            }
            return franka::JointPositions(current_q);
        });

        std::cout << "Pick and Place Complete!" << std::endl;

    } catch (const franka::Exception& e) { 
        std::cerr << "Hardware Exception: " << e.what() << std::endl; 
        // Automatic reflex recovery (optional, but nice if you crash a lot)
        // franka::Robot robot(robot_ip);
        // robot.automaticErrorRecovery();
        return -1; 
    }
    return 0;
}