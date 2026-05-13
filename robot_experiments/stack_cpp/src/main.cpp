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

struct Waypoint {
    std::string name;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    double duration; 
    bool grasp_after = false;
    bool release_after = false;
    int trigger_motion = 0; 
    int trigger_action = 0; 
    bool abort_after = false; // Flag to break the loop early
};

struct CubeData {
    std::array<double, 7> pre_pick_q;
    std::array<double, 7> pick_q;
    Eigen::Vector3d pre_place_xyz;
    Eigen::Vector3d place_xyz;
};

int main(int argc, char** argv) {
    try {
        // --- EXPERIMENT MODE SELECTION ---
        int mode_input = 0;
        std::cout << "==========================================\n";
        std::cout << " Select Experiment Mode:\n";
        std::cout << "   0: Fault-Free Run (Control)\n";
        std::cout << "   1: Faulty Run (EEG Surprise Factors)\n";
        std::cout << "==========================================\n";
        std::cout << "Choice: ";
        std::cin >> mode_input;
        bool is_faulty = (mode_input == 1);

        if (is_faulty) {
            std::cout << "\n[WARNING] Faults ENABLED. Prepare for sudden movements and drops.\n\n";
        }

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

        franka::RobotState initial_state = robot.readOnce();
        std::array<double, 16> F_T_EE = initial_state.F_T_EE;
        std::array<double, 16> EE_T_K = initial_state.EE_T_K;

        double cube_height = 0.04;
        double finger_offset = 0.1034; 
        Eigen::Quaterniond down_ori(0.0, 1.0, 0.0, 0.0);

        std::vector<CubeData> cubes = {
            { // CUBE 1
                {-0.0920499, 0.5526700, 0.0070220, -2.1236928, -0.0227715, 2.6450756, 0.6920618},
                {-0.0910745, 0.6309572, 0.0055707, -2.1017543, -0.0230764, 2.7015183, 0.6927476},
                {0.2412, 0.6043, 0.1569},
                {0.2412, 0.6043, 0.1169}
            },
            { // CUBE 2 (Will have TRAJECTORY DEVIATION fault if mode == 1)
                {0.0816911, 0.5491361, 0.0872336, -2.1110630, -0.0059721, 2.6425851, 0.9248035},
                {0.0877606, 0.6299139, 0.0821143, -2.0875576, -0.0117393, 2.7000316, 0.9291021},
                {0.2412, 0.6043, 0.1569 + cube_height},
                {0.2412, 0.6043, 0.1169 + cube_height}
            },
            { // CUBE 3
                {0.3149549, 0.6555755, 0.1134322, -1.9254474, -0.1017898, 2.5709333, 1.2158713},
                {0.3217765, 0.7291043, 0.1050727, -1.9009423, -0.1104822, 2.6196166, 1.2219025},
                {0.2412, 0.6043, 0.1569 + (2 * cube_height)},
                {0.2412, 0.6043, 0.11 + (2 * cube_height)}
            },
            { // CUBE 4 (Will have mid-air drop fault if mode == 1)
                {0.5162047, 0.8150493, 0.1255967, -1.6094433, -0.0977617, 2.4092304, 1.4764988},
                {0.5224169, 0.8901614, 0.1188737, -1.5806791, -0.1020845, 2.4553718, 1.4788992},
                {0.2412, 0.6043, 0.1569 + (3 * cube_height)},
                {0.2412, 0.6043, 0.10 + (3 * cube_height)} 
            }
        };

        std::array<double, 7> home_pos = {{-0.0001323, -0.7852356, 0.0002684, -2.3559399, 0.0007338, 1.5711873, 0.7851058}};

        // --- BUILD THE MASTER SEQUENCE ---
        std::vector<Waypoint> path;
        for (size_t i = 0; i < cubes.size(); ++i) {
            std::string prefix = "CUBE " + std::to_string(i + 1) + " ";
            int base = (i + 1) * 10; 
            
            auto pre_pick_arr = model.pose(franka::Frame::kEndEffector, cubes[i].pre_pick_q, F_T_EE, EE_T_K);
            Eigen::Affine3d pre_pick_pose(Eigen::Matrix4d::Map(pre_pick_arr.data()));
            
            auto pick_arr = model.pose(franka::Frame::kEndEffector, cubes[i].pick_q, F_T_EE, EE_T_K);
            Eigen::Affine3d pick_pose(Eigen::Matrix4d::Map(pick_arr.data()));

            // Move to Pre-Pick (Fast speed: 2.5 seconds)
            path.push_back({prefix + "PRE-PICK", pre_pick_pose.translation(), down_ori, 2.25, false, false, base + 1, 0, false});
            
            // Move down to Pick and grasp (Fast speed: 1.5 seconds)
            path.push_back({prefix + "PICK", pick_pose.translation(), down_ori, 1.25, true, false, base + 2, base + 3, false});
            
            // Move back up (Fast speed: 1.0 seconds)
            path.push_back({prefix + "LIFT", pre_pick_pose.translation(), down_ori, 0.75, false, false, base + 4, 0, false});

            // --- 2. LIFT / FAULT 1 PHASE ---
            if (is_faulty && i == 1) { // Cube 2 Fault
                // 1. Skip normal lift, veer directly to the wrong location
                std::array<double, 7> wrong_q = {-0.0684, 0.2537, -0.8500, -2.2450, 0.2818, 2.7106, 0.7612};
                auto wrong_arr = model.pose(franka::Frame::kEndEffector, wrong_q, F_T_EE, EE_T_K);
                Eigen::Affine3d wrong_pose(Eigen::Matrix4d::Map(wrong_arr.data()));

                // TRIGGER 80: Error Trajectory begins
                path.push_back({prefix + "WRONG LOCATION (FAULT)", wrong_pose.translation(), down_ori, 3.0, false, false, 80, 0, false});

                // 2. Recover to the safe hover pose you found
                std::array<double, 7> recover_q = {0.695693, 0.164098, 0.00732913, -1.93717, -0.00220801, 2.0995, 1.48411};
                auto recover_arr = model.pose(franka::Frame::kEndEffector, recover_q, F_T_EE, EE_T_K);
                Eigen::Affine3d recover_pose(Eigen::Matrix4d::Map(recover_arr.data()));

                // TRIGGER 82: Correction/Recovery begins
                //path.push_back({prefix + "RECOVERY HOVER", recover_pose.translation(), down_ori, 3.0, false, false, 82, 0, false});

            } else {
                // NORMAL LIFT (For all other cubes)
                path.push_back({prefix + "LIFT", pre_pick_pose.translation(), down_ori, 1.0, false, false, base + 4, 0, false});
            }

            // --- FAULT 2: TRUE MID-AIR DROP (Cube 4) ---
            double transit_duration = 2.25;
            int transit_trigger = base + 5;

            // --- FAULT 2: "GHOST" MID-AIR DROP (Cube 4) ---
            if (is_faulty && i == 3) { 
                // 1. Add the mid-air drop waypoint
                // Changed last argument (abort_after) from 'true' to 'false'
                // Use transit_trigger (base + 5) for the motion, and 81 ONLY for the gripper release!
            path.push_back({prefix + "INTERMEDIATE (MID-AIR DROP)", {0.4536, 0.3823, 0.25 - finger_offset}, down_ori, transit_duration, false, true, transit_trigger, 81, false});
                
                // 2. We NO LONGER 'continue'. 
                // The loop will now proceed to add PRE-PLACE, PLACE, and CLEARANCE below.
            } else {
                // Normal transit for non-faulted cubes
                path.push_back({prefix + "INTERMEDIATE", {0.4536, 0.3823, 0.25 - finger_offset}, down_ori, transit_duration, false, false, transit_trigger, 0, false});
            }
            
            // These will now be added for Cube 4 even if it dropped the cube!
            Eigen::Vector3d pre_place = cubes[i].pre_place_xyz;
            pre_place.z() -= finger_offset;

            path.push_back({prefix + "PRE-PLACE", pre_place, down_ori, 2.25, false, false, base + 6, 0, false});

            Eigen::Vector3d place = cubes[i].place_xyz;
            place.z() -= finger_offset;
            
            path.push_back({prefix + "PLACE", place, down_ori, 1.5, false, true, base + 7, base + 8, false});
            path.push_back({prefix + "CLEARANCE", pre_place, down_ori, 1.25, false, false, base + 9, 0, false});
        }

        std::cout << "Starting Strict Position Control Stacking Sequence..." << std::endl;
        udp.send(1); 
        gripper.move(0.08, 0.1);

        // --- EXECUTE MASTER CARTESIAN PATH ---
        for (const auto& point : path) {
            std::cout << ">>> Moving to: " << point.name << std::endl;

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
                udp.send(point.trigger_action); 
                gripper.grasp(0.04, 0.1, 5.0, 0.02, 0.02);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            } else if (point.release_after) {
                std::cout << "    [RELEASING]" << std::endl;
                udp.send(point.trigger_action); 
                gripper.move(0.08, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Abort Check for Fault 2
            if (point.abort_after) {
                std::cout << "\n>>> FAULT INJECTED: Sequence aborted mid-air." << std::endl;
                break;
            }
        }

        // --- RETURN TO HOME (JOINT CONTROL) ---
        std::cout << "\n>>> Returning to Home..." << std::endl;
        udp.send(90); 

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
        udp.send(99); 

    } catch (const franka::Exception& e) { 
        std::cerr << "Hardware Exception: " << e.what() << std::endl; 
        return -1; 
    }
    return 0;
}