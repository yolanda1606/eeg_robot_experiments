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

// --- MODIFIED: UDP Sender Helper Class (Dual Target) ---
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
            // Setup EEG Destination
            memset(&eeg_addr, 0, sizeof(eeg_addr));
            eeg_addr.sin_family = AF_INET;
            eeg_addr.sin_port = htons(eeg_port);
            inet_pton(AF_INET, eeg_ip.c_str(), &eeg_addr.sin_addr);

            // Setup Video Node Destination
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
        
        // Fire to EEG Laptop
        sendto(sockfd, msg.c_str(), msg.length(), 0, (struct sockaddr*)&eeg_addr, sizeof(eeg_addr));
        // Fire to Local Python Vision Node
        sendto(sockfd, msg.c_str(), msg.length(), 0, (struct sockaddr*)&video_addr, sizeof(video_addr));
        
        std::cout << "[UDP] Sent Trigger: [" << trigger_value << "] to EEG and Video Node" << std::endl;
    }
};

// --- Waypoint Struct ---
struct Waypoint {
    std::string name;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    double duration; 
    bool grasp_after = false;
    bool release_after = false;
    int trigger_value = 0; 
};

int main(int argc, char** argv) {
    try {
        // --- MODIFIED: Initialize Dual UDP Connection ---
        std::string eeg_ip = "10.0.0.2"; 
        int eeg_port = 1000;
        std::string video_ip = "127.0.0.1"; // Localhost where Python runs
        int video_port = 5005;              // The port your Python node listens to
        
        UdpSender udp(eeg_ip, eeg_port, video_ip, video_port);

        // Standard Robot Initialization
        std::string robot_ip = "172.16.0.2";
        franka::Robot robot(robot_ip);
        franka::Gripper gripper(robot_ip);

        Eigen::Quaterniond down_ori(0.0, 1.0, 0.0, 0.0);

        // --- Define Path with UDP Triggers ---
        std::vector<Waypoint> path = {
            {"PRE-PICK",  {0.5546, -0.0486, 0.2273}, down_ori, 4.0, false, false, 11}, 
            {"PICK",      {0.5555, -0.0513, 0.0571}, down_ori, 2.0, true, false,  12}, 
            {"POST-PICK", {0.4536, 0.3823, 0.5087}, down_ori, 3.0, false, false, 13},
            {"PRE-PLACE", {0.2456, 0.6113, 0.2719}, down_ori, 4.0, false, false, 14},
            {"PLACE",     {0.2456, 0.6113, 0.0691}, down_ori, 2.0, false, true,  15},  
            {"CLEARANCE", {0.2456, 0.6113, 0.2719}, down_ori, 2.0, false, false, 16}
        };

        std::array<double, 7> home_pos = {{-0.0001, -0.7852, 0.0002, -2.3559, 0.0007, 1.5711, 0.7851}};

        std::cout << "Starting Pure C++ Pick and Place (Strict Position Control)..." << std::endl;
        
        udp.send(1); // TRIGGER 1: Experiment Start
        gripper.move(0.08, 0.1);

        // --- EXECUTE CARTESIAN PATH ---
        for (const auto& point : path) {
            std::cout << ">>> Moving to: " << point.name << std::endl;
            
            // Send dual-trigger exactly before motion begins
            udp.send(point.trigger_value); 

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

            // Action Phase
            if (point.grasp_after) {
                std::cout << "Action: Grasping object..." << std::endl;
                udp.send(20); 
                gripper.grasp(0.04, 0.1, 40.0, 0.02, 0.02);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else if (point.release_after) {
                std::cout << "Action: Releasing object..." << std::endl;
                udp.send(21); 
                gripper.move(0.08, 0.1);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // --- RETURN TO HOME (JOINT CONTROL) ---
        std::cout << "\n>>> Returning to Safe Home Position..." << std::endl;
        udp.send(30); 

        std::array<double, 7> start_q;
        bool home_tick = true;
        double time_j = 0.0;
        double home_duration = 5.0;

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

        std::cout << "Pick and Place Complete!" << std::endl;
        udp.send(99); 

    } catch (const franka::Exception& e) { 
        std::cerr << "Hardware Exception: " << e.what() << std::endl; 
        return -1; 
    }
    return 0;
}
