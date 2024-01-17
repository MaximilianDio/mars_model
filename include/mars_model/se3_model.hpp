
#ifndef MARS_MODEL_SE3_HPP
#define MARS_MODEL_SE3_HPP

#include <string>
#include <vector>

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "utils.hpp"

namespace mars
{
// joint names and frame names for submodels
const std::string obj_joint_name = "obj_se3_joint";
// grasp frames
const std::vector<std::string> obj_grasp_frame_names = {"obj_p_01", "obj_p_02"};
// frame obtained from measurement e.g. mocap
const std::string obj_meaured_frame_name = "obj_measured";
// COM frame of object
const std::string obj_com_frame_name = "obj_com";
// reference frame of object for tracking
const std::string obj_ref_frame_name = "obj_ref";


class SE3
{
public:
    /**
     * @brief Construct a new SE3 object by copying a pinocchio model
    */
    SE3(pinocchio::Model &model)
    {
        model_ = model;
    }
    /**
     * @brief Construct a new SE3 object by loading a urdf file
    */
    SE3(const std::string &urdf_filename)
    {
        pinocchio::urdf::buildModel(urdf_filename, model_);
    }

    /**
     * @brief Get the frame ids and compute relative frames
    */
    void init()
    {
        // get frame ids

        auto getFrameId = [&](const std::string &frame_name, pinocchio::JointIndex &frame_id) {
            if (model_.existFrame(frame_name))
                frame_id = model_.getFrameId(frame_name);
            else
                std::cout << "frame: " << frame_name << " does not belong to the model\n";
        };

        getFrameId(obj_com_frame_name, com_frame_id_);
        getFrameId(obj_meaured_frame_name, measured_frame_id_);
        getFrameId(obj_ref_frame_name, ref_frame_id_);

        for (const auto &frame_name : obj_grasp_frame_names)
        {
            pinocchio::JointIndex frame_id;
            getFrameId(frame_name, frame_id);
            grasp_frame_ids_.push_back(frame_id);
        }

        // compute relative frames

        T_ref2com_ = model_.frames[ref_frame_id_].placement;
        T_meas2com_ = model_.frames[measured_frame_id_].placement;

        T_grasp2meas_.reserve(grasp_frame_ids_.size());
        T_grasp2com_.reserve(grasp_frame_ids_.size());
        // T_mocap2grasp
        for (const auto &frame_id : grasp_frame_ids_)
        {
            T_grasp2com_.push_back(model_.frames[frame_id].placement);

            T_grasp2meas_.push_back(T_meas2com_ * model_.frames[frame_id].placement);
        }

        // get mass matrix and gravity vector
        mass_ = model_.inertias[0].mass();
        theta_ = model_.inertias[0].inertia();

        M_(0, 0) = mass_;
        M_(1, 1) = mass_;
        M_(2, 2) = mass_;
        M_.block<3, 3>(3, 3) = theta_;

        std::cout << "Object: Number of graspframes: " << grasp_frame_ids_.size() << "\n";
    }

    /**
     * @brief Compute the mass matrix in world frame of the dynamics equation
     * 
     * @param q generalized coordinates
     * @param Mw mass matrix in world frame
     * 
     * @return Eigen::Ref<Eigen::MatrixXd> mass matrix in world frame
     * 
     * @details M = blockdiag(m*I,R_WB*I_B*R_BW )
    */
    Eigen::Ref<Eigen::MatrixXd> mass_matrix(const Eigen::VectorXd &q, Eigen::Ref<Eigen::MatrixXd> Mw)
    {
        // transform mass matrix to world frame
        const auto quat = Eigen::Quaternion<double>(q.tail<4>());

        // M = blockdiag(m*I,R_WB*I_B*R_BW )
        Mw.block<3, 3>(0, 0).noalias() = M_.block<3, 3>(0, 0);
        Mw.block<3, 3>(3, 3).noalias() =
            quat.toRotationMatrix() * M_.block<3, 3>(3, 3) * quat.inverse().toRotationMatrix();

        return Mw;
    }

    /**
     * @brief Compute the gravity term of the dynamics equation
     * 
     * @param q generalized coordinates
     * @param g gravity term
     * 
     * @return Eigen::Ref<Eigen::VectorXd> gravity term
     * 
     * @details gravityElements(g = [m * g_vec; 0])
    */
    Eigen::Ref<Eigen::VectorXd> gravity_term(const Eigen::VectorXd & /*q*/, Eigen::Ref<Eigen::VectorXd> g)
    {
        g.head(3) = model_.gravity.linear() * mass_;
        return g;
    }

    /**
     * @brief Compute the nonlinear term of the dynamics equation
     * 
     * @param q generalized coordinates
     * @param v generalized velocities
     * @param nle nonlinear term
     * 
     * @return Eigen::Ref<Eigen::VectorXd> nonlinear term
     * 
     * @details nonlinearElements(n = [-m * g_vec; w x(I * w)]) // Minus g_vec is correct here!
    */
    Eigen::Ref<Eigen::VectorXd> non_linear_term(const Eigen::VectorXd &q,
                                                const Eigen::VectorXd &v,
                                                Eigen::Ref<Eigen::VectorXd> nle)
    {
        const auto quat = Eigen::Quaternion<double>(q.tail<4>());

        // nonlinearElements(n = [-m * g_vec; w x(I * w)]) // Minus g_vec is correct here!
        nle.head(3) = -model_.gravity.linear() * mass_;
        nle.tail(3) = pinocchio::cross(v.tail<3>(),
                                       quat.toRotationMatrix() * M_.block<3, 3>(3, 3) *
                                           quat.inverse().toRotationMatrix() * v.tail<3>());

        return nle;
    }

    /**
     * @brief Compute the kinematic ODE of the dynamics equation
     * 
     * @param q generalized coordinates
     * @param v generalized velocities
     * @param qdot generalized velocities
     * 
     * @return Eigen::Ref<Eigen::VectorXd> generalized velocities
     * 
     * @details dq = [I,0;0,Z(quat)]*[v;w] (Z defines the quaternion propagation from angular velocity)
     *          Z is defined in utils.hpp
     * 
    */
    Eigen::Ref<Eigen::VectorXd> kinematic_ode(const Eigen::VectorXd &q,
                                              const Eigen::VectorXd &v,
                                              Eigen::Ref<Eigen::VectorXd> qdot)
    {
        OMEGA(Eigen::Quaterniond(q.tail<4>()), Z_);
        qdot << v.head(3), Z_ * v.tail<3>();
        return qdot;
    }

    /**
     * @brief Compute the frame jacobian of a grasp frame in world frame
     * 
     * @param grasp_idx index of the grasp frame
     * @param q generalized coordinates
     * @param J frame jacobian of the grasp frame in world frame
     * 
     * @return Eigen::Ref<Eigen::MatrixXd> frame jacobian of the grasp frame in world frame
     * 
     * @details G^T_i = [I, -[r]_x; 0, I]
    */
    Eigen::Ref<Eigen::MatrixXd> frame_jacobian(const int grasp_idx,
                                               const Eigen::VectorXd &q,
                                               Eigen::Ref<Eigen::MatrixXd> J)
    {
        const auto quat = Eigen::Quaternion<double>(q.tail<4>());
        // G^T_i = [I, -[r]_x]
        //         [0,    I  ]
        J.setIdentity();
        pinocchio::skew(-quat.toRotationMatrix() * T_grasp2com_[grasp_idx].translation(), J.block<3, 3>(0, 3));

        return J;
    }

    /**
     * @brief Compute the local acceleration of a grasp frame in world frame
     * 
     * @param grasp_idx index of the grasp frame
     * @param q generalized coordinates
     * @param v generalized velocities
     * @param a local acceleration of the grasp frame in world frame
     * 
     * @return Eigen::Ref<Eigen::VectorXd> local acceleration of the grasp frame in world frame
     * 
     * @details dw = w x (w x r)
    */
    Eigen::Ref<Eigen::VectorXd> frame_local_acc(const int grasp_idx,
                                                const Eigen::VectorXd &q,
                                                const Eigen::VectorXd &v,
                                                Eigen::Ref<Eigen::VectorXd> a)
    {
        const auto quat = Eigen::Quaternion<double>(q.tail<4>());

        // dw = w x (w x r)
        a.head(3) =
            v.tail<3>(3).cross(v.tail<3>(3).cross(quat.toRotationMatrix() * T_grasp2com_[grasp_idx].translation()));
        a.tail(3).setZero();

        return a;
    }


    // pinocchio model
    pinocchio::Model model_;

private:
    std::vector<pinocchio::JointIndex> grasp_frame_ids_ = {};
    pinocchio::JointIndex com_frame_id_;
    pinocchio::JointIndex measured_frame_id_;
    pinocchio::JointIndex ref_frame_id_;

    // relative frames
    pinocchio::SE3 T_meas2com_;                // transformation from measurement to com frame
    pinocchio::SE3 T_ref2com_;                 // transformation from ref to com frames
    std::vector<pinocchio::SE3> T_grasp2meas_; // transformation from grasp to mocap frames
    std::vector<pinocchio::SE3> T_grasp2com_;  // transformation from grasp to com frames

    Eigen::Matrix<double, 4, 3> Z_;
    // mass matrix in com frame
    Eigen::MatrixXd M_ = Eigen::MatrixXd::Zero(6, 6);

    double mass_;
    Eigen::Matrix3d theta_; // inertia matrix in com frame
};

} // namespace mars

#endif // MARS_MODEL_SE3_HPP