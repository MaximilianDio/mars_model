#ifndef MARS_SERIAL_LINK_HPP
#define MARS_SERIAL_LINK_HPP

#include "base_model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"


namespace mars
{

class SerialLink : public BaseModel
{
public:
    SerialLink(pinocchio::Model &model) : BaseModel(model)
    {
        data_ = pinocchio::Data(model_);
    }
    SerialLink(const std::string &urdf_filename) : BaseModel(urdf_filename)
    {
        data_ = pinocchio::Data(model_);
    }

    void init(const std::vector<std::string> &frame_names)
    {
        for (const auto &frame_name : frame_names)
        {
            pinocchio::JointIndex frame_id;
            getFrameId(frame_name, frame_id);
            frame_ids_.push_back(frame_id);
        }
    }

    void update_kinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        pinocchio::forwardKinematics(model_, data_, q, v);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_, q);
    }

    Eigen::Ref<const Eigen::MatrixXd> mass_matrix(const Eigen::VectorXd &q, Eigen::Ref<Eigen::MatrixXd> Mw)
    {
        // compute mass matrix in world frame
        pinocchio::crba(model_, data_, q);
        Mw = data_.M;

        // make symmetric
        Mw.template triangularView<Eigen::StrictlyLower>() =
            Mw.transpose().template triangularView<Eigen::StrictlyLower>();
        return Mw;
    }

    Eigen::Ref<const Eigen::VectorXd> gravity_term(const Eigen::VectorXd &q, Eigen::Ref<Eigen::VectorXd> g)
    {
        g = pinocchio::computeGeneralizedGravity(model_, data_, q);
        return g;
    }

    Eigen::Ref<const Eigen::VectorXd> non_linear_term(const Eigen::VectorXd &q,
                                                      const Eigen::VectorXd &v,
                                                      Eigen::Ref<Eigen::VectorXd> nle)
    {
        nle = pinocchio::rnea(model_, data_, q, v, Eigen::VectorXd::Zero(model_.nv));
        return nle;
    }

    const pinocchio::SE3 &frame_placement(const int idx, pinocchio::SE3 &M)
    {
        M = data_.oMf[frame_ids_[idx]];
        return M;
    }

    virtual Eigen::Ref<const Eigen::VectorXd> frame_velocity(const int idx, Eigen::Ref<Eigen::VectorXd> x_frame)
    {
        x_frame =
            pinocchio::getFrameVelocity(model_, data_, frame_ids_[idx], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
        return x_frame;
    }

    Eigen::Ref<const Eigen::MatrixXd> frame_jacobian(const int idx, Eigen::Ref<Eigen::MatrixXd> J)
    {
        pinocchio::getFrameJacobian(model_, data_, frame_ids_[idx], pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    };

    Eigen::Ref<const Eigen::VectorXd> frame_local_acc(const int idx, Eigen::Ref<Eigen::VectorXd> a)
    {
        a = getFrameClassicalAcceleration(model_, data_, frame_ids_[idx], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
        return a;
    }

private:
    pinocchio::Data data_;
};

} // namespace mars

#endif // MARS_SERIAL_LINK_HPP
