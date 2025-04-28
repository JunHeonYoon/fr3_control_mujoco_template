#define _USE_MATH_DEFINES
#include <cmath>

#ifndef CROBOTICS_CONTROLLER_H
#define CROBOTICS_CONTROLLER_H

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <fstream>
#include <iostream>

using namespace Eigen;

class cRoboticsController
{
    public:
        cRoboticsController(const std::string& urdf_path, 
                            const std::string& manipulator_control_mode, 
                            const double& dt);

        std::string manipulator_control_mode_; // position or torque
        double dt_;
            
        // pinocchio data
        pinocchio::Model model_;
        pinocchio::Data data_;

        std::ofstream logging_file_;

        // =============================================================================
        // ================================= User data =================================
        // =============================================================================
        // control mode state
        bool is_mode_changed_{false};
        enum CTRL_MODE {
            joint_ctrl_home,
            joint_ctrl_init,
            
            // -------------------------------------
            // TODO 1: Add your control modes here
            // Example:
            hw2_1,
            torque_ctrl_dynamic,
            // -------------------------------------
            
            DEFAULT
        };
        CTRL_MODE control_mode_{DEFAULT};

        // time state
        double play_time_;
        double control_start_time_;

        // Current Joint space state
        VectorXd q_;           // joint angle (7,1)
        VectorXd qdot_;        // joint velocity (7,1)
        VectorXd tau_;         // joint torque (7,1)

        // Cntrol Input 
        VectorXd q_desired_;   // desired joint angle (7,1)  -> using for position mode
        VectorXd tau_desired_; // desired joint torque (7,1) -> using for torque mode

        // Initial joint space state
        VectorXd q_init_;      // initial joint angle (7,1)
        VectorXd qdot_init_;   // initial joint velocity (7,1)

        // Task space state
        std::string ee_name_{"attachment_site"};
        Matrix4d x_;                            // Homogeneous matrix; pose of EE (4,4)
        VectorXd xdot_;                         // velocity of EE (6,1); linear + angular
        MatrixXd J_;                            // jacobian of EE (6,7)
        Matrix4d x_init_;                       // Homogeneous matrix; initial pose of EE (4,4)
        Matrix4d x_from_q_desired_;             // Homogeneous matrix; pose of EE from q_desired_ (4,4)
        Matrix<double, 6, 7> J_from_q_desired_; // jacobian of EE from q_desired_ (6,7)

        // Joint space Dynamics
        MatrixXd M_;     // mass matrix (7,7)
        VectorXd g_;     // gravity torques (7,1)
        VectorXd c_;     // centrifugal and coriolis forces (7,1)

        // =============================================================================
        // =============================== User functions ==============================
        // =============================================================================
        // Control functions
        void moveJointPosition(const VectorXd& target_position, double duration);
        void moveJointPositionTorque(const VectorXd& target_position, double duration);

        // ----------------------------------------------
        // TODO 2: Add your control function here
        // Example:
        void HW2_1();
        void torqueCtrlDynamic();
        // ----------------------------------------------

        // ===================================================================

        // Core functions
        void keyMapping(const int &key);
        void compute(const double& play_time);
        void setMode(const CTRL_MODE& control_mode);
        void printState();
        bool updateModel(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau);
        bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
        bool updateDynamics(const VectorXd& q, const VectorXd& qdot);
        VectorXd getCtrlInput();
};

#endif // CROBOTICS_CONTROLLER_H