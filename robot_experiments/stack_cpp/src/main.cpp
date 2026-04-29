#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

// Networking headers for Ubuntu (POSIX)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include <Eigen/Dense>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/exception.h>

// --- UDP Sender Helper Class (Dual Target) ---
class UdpSender {
private:
    int sockfd;
    struct sockaddr_in eeg_addr;
    struct sockaddr_in video_addr;
    bool initialized = false;

public:
    UdpSender(const std::string& eeg_ip, int eeg_port, const std::string& video_ip, int video_port) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd >= 0) {
            memset(&eeg_addr, 0, sizeof(eeg_addr));
            eeg_addr.sin_family = AF_INET;
            eeg_addr.sin_port = htons(eeg_port);
            inet_pton(AF_INET, eeg_ip.c_str(), &eeg_addr.sin_addr);

            memset(&video_addr, 0, sizeof(video_addr));
            video_addr.sin_family = AF_INET;
            video_addr.sin_port = htons(video_port);
            inet_pton(AF_INET, video_ip.c_str(), &video_addr.sin_addr);

            initialized = true;
        } else {
            std::cerr << "Failed to create UDP socket." << std::endl;
        }
    }

    ~UdpSender() {
        if (sockfd >= 0) close(sockfd);
    }

    void send(int trigger_value) {
        if (!initialized) return;
        std::string msg = std::to_string(trigger_value);
        
        sendto(sockfd, msg.c_str(), msg.length(), 0, (struct sockaddr*)&eeg_addr, sizeof(eeg_addr));
        sendto(sockfd, msg.c_str(), msg.length(), 0, (struct sockaddr*)&video_addr, sizeof(video_addr));
        
        std::cout << "[UDP] Sent Trigger: [" << trigger_value << "]" << std::endl;
    }
};

// --- MODIFIED: Added Trigger Fields ---
struct Waypoint {
    std::string name;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    double duration; 
    bool grasp_after = false;
    bool release_after = false;
    int trigger_motion = 0; // Fired right before the robot moves
    int trigger_action = 0; // Fired right before the gripper actuates
};

struct CubeData {
    std::array<double, 7> pre_pick_q;
    std::array<double, 7> pick_q;
    Eigen::Vector3d pre_place_xyz;
    Eigen::Vector3d place_xyz;
};

int main(int argc, char** argv) {
    try {
        // --- Initialize Dual UDP Connection ---
        std::string eeg_ip = "10.0.0.2"; 
        int eeg_port = 1000;
        std::string video_ip = "127.0.0.1"; 
        int video_port = 5005;              
        
        UdpSender udp(eeg_ip, eeg_port, video_ip, video_port);

        // --- Robot Initialization ---
        std::string robot_ip = "172.16.0.2";
        franka::Robot robot(robot_ip);
        franka::Gripper gripper(robot_ip);
        franka::Model model = robot.loadModel();

        // Get geometry for Joint-to-Cartesian Math
        franka::RobotState initial_state = robot.readOnce();
        std::array<double, 16> F_T_EE = initial_state.F_T_EE;
        std::array<double, 16> EE_T_K = initial_state.EE_T_K;

        double cube_height = 0.04;
        double finger_offset = 0.1034; 
        Eigen::Quaterniond down_ori(0.0, 1.0, 0.0, 0.0);

        // --- PORTED PYTHON DATA ---
        std::vector<CubeData> cubes = {
            { // CUBE 1
                {-0.0920499, 0.5526700, 0.0070220, -2.1236928, -0.0227715, 2.6450756, 0.6920618},
                {-0.0910745, 0.6309572, 0.0055707, -2.1017543, -0.0230764, 2.7015183, 0.6927476},
                {0.2412, 0.6043, 0.1569},
                {0.2412, 0.6043, 0.1169}
            },
            { // CUBE 2
                {0.0816911, 0.5491361, 0.0872336, -2.1110630, -0.0059721, 2.6425851, 0.9248035},
                {0.0877606, 0.6299139, 0.0821143, -2.0875576, -0.0117393, 2.7000316, 0.9291021},
                {0.2412, 0.6043, 0.1569 + cube_height},
                {0.2412, 0.6043, 0.1169 + cube_height}
            },
            { // CUBE 3
                {0.3149549, 0.6555755, 0.1134322, -1.9254474, -0.1017898, 2.5709333, 1.2158713},
                {0.3217765, 0.7291043, 0.1050727, -1.9009423, -0.1104822, 2.6196166, 1.2219025},
                {0.2412, 0.6043, 0.1569 + (2 * cube_height)},
                {0.2412, 0.6043, 0.1169 + (2 * cube_height)}
            },
            { // CUBE 4 
                {0.5162047, 0.8150493, 0.1255967, -1.6094433, -0.0977617, 2.4092304, 1.4764988},
                {0.5224169, 0.8901614, 0.1188737, -1.5806791, -0.1020845, 2.4553718, 1.4788992},
                {0.2412, 0.6043, 0.1569 + (3 * cube_height)},
                {0.2412, 0.6043, 0.1069 + (3 * cube_height)} 
            }
        };

        std::array<double, 7> home_pos = {{-0.0001323, -0.7852356, 0.0002684, -2.3559399, 0.0007338, 1.5711873, 0.7851058}};

        // --- BUILD THE MASTER SEQUENCE (WITH TRIGGERS) ---
        std::vector<Waypoint> path;
        for (size_t i = 0; i < cubes.size(); ++i) {
            std::string prefix = "CUBE " + std::to_string(i + 1) + " ";
            int base = (i + 1) * 10; // Cube 1 = 10, Cube 2 = 20, etc.
            
            // Convert Joints to Cartesian on the fly
            auto pre_pick_arr = model.pose(franka::Frame::kEndEffector, cubes[i].pre_pick_q, F_T_EE, EE_T_K);
            Eigen::Affine3d pre_pick_pose(Eigen::Matrix4d::Map(pre_pick_arr.data()));
            
            auto pick_arr = model.pose(franka::Frame::kEndEffector, cubes[i].pick_q, F_T_EE, EE_T_K);
            Eigen::Affine3d pick_pose(Eigen::Matrix4d::Map(pick_arr.data()));

            // Define the steps and embed the triggers dynamically
            path.push_back({prefix + "PRE-PICK", pre_pick_pose.translation(), down_ori, 6.0, false, false, base + 1, 0});
            path.push_back({prefix + "PICK", pick_pose.translation(), down_ori, 2.0, true, false, base + 2, base + 3});
            path.push_back({prefix + "LIFT", pre_pick_pose.translation(), down_ori, 1.5, false, false, base + 4, 0});
            path.push_back({prefix + "INTERMEDIATE", {0.4536, 0.3823, 0.25 - finger_offset}, down_ori, 3.0, false, false, base + 5, 0});
            
            // Place Sequence
            Eigen::Vector3d pre_place = cubes[i].pre_place_xyz;
            pre_place.z() -= finger_offset;
            path.push_back({prefix + "PRE-PLACE", pre_place, down_ori, 3.0, false, false, base + 6, 0});

            Eigen::Vector3d place = cubes[i].place_xyz;
            place.z() -= finger_offset;
            path.push_back({prefix + "PLACE", place, down_ori, 2.0, false, true, base + 7, base + 8});

            // Clearance
            path.push_back({prefix + "CLEARANCE", pre_place, down_ori, 1.5, false, false, base + 9, 0});
        }

        std::cout << "Starting Strict Position Control Stacking Sequence..." << std::endl;
        udp.send(1); // TRIGGER 1: Experiment Start
        gripper.move(0.08, 0.1);

        // --- EXECUTE MASTER CARTESIAN PATH ---
        for (const auto& point : path) {
            std::cout << ">>> Moving to: " << point.name << std::endl;

            // Fire motion trigger just before loop begins
            udp.send(point.trigger_motion);

            Eigen::Vector3d start_pos;
            Eigen::Quaterniond start_ori;
            bool first_tick = true;
            double time = 0.0;
            
            robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
                time += period.toSec();
                
                if (first_tick) {
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

            // Gripper Action Phase 
            if (point.grasp_after) {
                std::cout << "    [GRASPING]" << std::endl;
                udp.send(point.trigger_action); // Fire grasp trigger
                gripper.grasp(0.02, 0.1, 20.0, 0.02, 0.02);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            } else if (point.release_after) {
                std::cout << "    [RELEASING]" << std::endl;
                udp.send(point.trigger_action); // Fire release trigger
                gripper.move(0.08, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

        // --- RETURN TO HOME (JOINT CONTROL) ---
        std::cout << "\n>>> Sequence Complete! Returning to Home..." << std::endl;
        udp.send(90); // TRIGGER 90: Returning Home

        std::array<double, 7> start_q;
        bool home_tick = true;
        double time_j = 0.0;
        double home_duration = 4.0; 

        robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
            time_j += period.toSec();
            
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

        std::cout << "Stacking Task Successfully Concluded." << std::endl;
        udp.send(99); // TRIGGER 99: Experiment Complete

    } catch (const franka::Exception& e) { 
        std::cerr << "Hardware Exception: " << e.what() << std::endl; 
        return -1; 
    }
    return 0;
}