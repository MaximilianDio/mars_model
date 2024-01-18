
#ifndef MARS_MODEL_SE3_HPP
#define MARS_MODEL_SE3_HPP

#include <string>
#include <vector>

#include "base_model.hpp"

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


class SE3 : public BaseModel
{
public:
    SE3(pinocchio::Model &model) : BaseModel(model)
    {
    }
    SE3(const std::string &urdf_filename) : BaseModel(urdf_filename)
    {
    }

    void init(const std::vector<std::string> &grasp_frame_names)
    {
        getFrameId(obj_com_frame_name, com_frame_id_);
        getFrameId(obj_meaured_frame_name, measured_frame_id_);
        getFrameId(obj_ref_frame_name, ref_frame_id_);

        frame_ids_.clear();
        frame_ids_.reserve(grasp_frame_names.size());
        for (const auto &frame_name : grasp_frame_names)
        {
            pinocchio::JointIndex frame_id;
            getFrameId(frame_name, frame_id);
            frame_ids_.push_back(frame_id);
        }

        // compute relative frames

        T_ref2com_ = model_.frames[ref_frame_id_].placement;
        T_meas2com_ = model_.frames[measured_frame_id_].placement;

        T_grasp2com_.clear();
        T_grasp2meas_.clear();
        T_grasp2meas_.reserve(frame_ids_.size());
        T_grasp2com_.reserve(frame_ids_.size());
        // T_mocap2grasp
        for (const auto &frame_id : frame_ids_)
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

        std::cout << "Object: Number of graspframes: " << frame_ids_.size() << "\n";
    }


    Eigen::Ref<const Eigen::MatrixXd> mass_matrix(const Eigen::VectorXd &q, Eigen::Ref<Eigen::MatrixXd> Mw)
    {
        // transform mass matrix to world frame
        const auto quat = Eigen::Quaternion<double>(q.tail<4>());

        // M = blockdiag(m*I,R_WB*I_B*R_BW )
        Mw.block<3, 3>(0, 0).noalias() = M_.block<3, 3>(0, 0);
        Mw.block<3, 3>(3, 3).noalias() =
            quat.toRotationMatrix() * M_.block<3, 3>(3, 3) * quat.inverse().toRotationMatrix();

        return Mw;
    }


    Eigen::Ref<const Eigen::VectorXd> gravity_term(const Eigen::VectorXd & /*q*/, Eigen::Ref<Eigen::VectorXd> g)
    {
        g.head(3) = model_.gravity.linear() * mass_;
        return g;
    }


    Eigen::Ref<const Eigen::VectorXd> non_linear_term(const Eigen::VectorXd &q,
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

    Eigen::Ref<const Eigen::VectorXd> kinematic_ode(const Eigen::VectorXd &q,
                                                    const Eigen::VectorXd &v,
                                                    Eigen::Ref<Eigen::VectorXd> qdot)
    {
        OMEGA(Eigen::Quaterniond(q.tail<4>()), Z_);
        qdot << v.head(3), Z_ * v.tail<3>();
        return qdot;
    }

    const pinocchio::SE3 &frame_placement(const int idx, pinocchio::SE3 &M)
    {
        M = T_WB_ * T_grasp2com_[idx];
        return M;
    }

    Eigen::Ref<const Eigen::VectorXd> frame_velocity(const int idx, Eigen::Ref<Eigen::VectorXd> v_frame)
    {
        // v_frame = [v_WB; w_WB]
        v_frame.head(3) = v_.linear() + v_.angular().cross(T_WB_.rotation() * T_grasp2com_[idx].translation());
        v_frame.tail(3) = v_.angular();

        return v_frame;
    }

    void update_kinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        T_WB_ = pinocchio::SE3(Eigen::Quaternion<double>(q.tail<4>()), q.head<3>());
        v_ = pinocchio::MotionTpl<double>(v.head(3), v.tail<3>());
    }

    Eigen::Ref<const Eigen::MatrixXd> frame_jacobian(const int grasp_idx, Eigen::Ref<Eigen::MatrixXd> J)
    {
        // G^T_i = [I, -[r]_x]
        //         [0,    I  ]
        J.setIdentity();
        pinocchio::skew(-T_WB_.rotation() * T_grasp2com_[grasp_idx].translation(), J.block<3, 3>(0, 3));

        return J;
    }

    Eigen::Ref<const Eigen::VectorXd> frame_local_acc(const int grasp_idx, Eigen::Ref<Eigen::VectorXd> a)
    {
        // dw = w x (w x r)
        a.head(3) = v_.angular().cross(v_.angular().cross(T_WB_.rotation() * T_grasp2com_[grasp_idx].translation()));
        a.tail(3).setZero();

        return a;
    }

private:
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

    pinocchio::MotionTpl<double> v_;
    pinocchio::SE3 T_WB_; // transformation from world to base frame
};
} // namespace mars

#endif // MARS_MODEL_SE3_HPP