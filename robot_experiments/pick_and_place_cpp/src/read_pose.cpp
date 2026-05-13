#include <iostream>
#include <franka/robot.h>
#include <franka/exception.h> // <--- Add this line!

int main() {
    try {
        std::string robot_ip = "172.16.0.2";
        franka::Robot robot(robot_ip);

        // Read the current state of the robot exactly once
        franka::RobotState state = robot.readOnce();

        std::cout << "\n=== CURRENT ROBOT STATE ===" << std::endl;
        
        // Indices 12, 13, 14 of the O_T_EE array are the x, y, z translations
        std::cout << "\nCartesian Position (O_T_EE): " << std::endl;
        std::cout << "x: " << state.O_T_EE[12] << std::endl;
        std::cout << "y: " << state.O_T_EE[13] << std::endl;
        std::cout << "z: " << state.O_T_EE[14] << std::endl;

        std::cout << "\nJoint Positions (q): " << std::endl;
        std::cout << "{";
        for (size_t i = 0; i < 7; i++) {
            std::cout << state.q[i] << (i < 6 ? ", " : "");
        }
        std::cout << "}" << std::endl;

    } catch (const franka::Exception& e) {
        std::cerr << "Hardware Exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}