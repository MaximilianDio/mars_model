
#ifndef MARS_BASE_MODEL_HPP
#define MARS_BASE_MODEL_HPP

#include <string>
#include <vector>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "utils.hpp"

namespace mars
{

class BaseModel
{
public:
    /**
     * @brief Construct a new model by copying a pinocchio model
    */
    BaseModel(pinocchio::Model &model, const std::vector<std::string> &/*frame_names*/)
    {
        model_ = model;
    }
    /**
     * @brief Construct a new model by loading a urdf file
    */
    BaseModel(const std::string &urdf_filename, const std::vector<std::string> &/*frame_names*/)
    {
        pinocchio::urdf::buildModel(urdf_filename, model_);
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
    virtual Eigen::Ref<const Eigen::MatrixXd> mass_matrix(const Eigen::VectorXd &q, Eigen::Ref<Eigen::MatrixXd> Mw) = 0;

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
    virtual Eigen::Ref<const Eigen::VectorXd> gravity_term(const Eigen::VectorXd &q, Eigen::Ref<Eigen::VectorXd> g) = 0;

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
    virtual Eigen::Ref<const Eigen::VectorXd> non_linear_term(const Eigen::VectorXd &q,
                                                              const Eigen::VectorXd &v,
                                                              Eigen::Ref<Eigen::VectorXd> nle) = 0;

    /**
     * @brief Compute the kinematic ODE of the dynamics equation
     * 
     * @param q generalized coordinates
     * @param v generalized velocities
     * @param qdot generalized velocities
     * 
     * @return Eigen::Ref<Eigen::VectorXd> generalized velocities
     * 
     * 
    */
    virtual Eigen::Ref<const Eigen::VectorXd> kinematic_ode(const Eigen::VectorXd & /*q*/,
                                                            const Eigen::VectorXd &v,
                                                            Eigen::Ref<Eigen::VectorXd> qdot)
    {
        qdot = v;
        return qdot;
    }

    /**
     * @brief Update the kinematics of the model (frame placements, jacobians, etc.)
     * 
     * @param q generalized coordinates
     * @param v generalized velocities
     * 
     * @details This function should be called before calling any other kinematics related functions like 
     * frame_jacobian, frame_local_acc, etc.
     * 
     * @remarks No acceleration is computed here. The local frame acceleration is not dependent on the generalized accelerations.
    */
    virtual void update_kinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &v) = 0;


    virtual const pinocchio::SE3 &frame_placement(const int idx, pinocchio::SE3 &M) = 0;

    virtual Eigen::Ref<const Eigen::VectorXd> frame_velocity(const int idx, Eigen::Ref<Eigen::VectorXd> v_frame) = 0;
    /**
     * @brief Compute the frame jacobian of a grasp frame in world frame
     * 
     * @param grasp_idx index of the grasp frame
     * @param q generalized coordinates
     * @param J frame jacobian of the grasp frame in world frame
     * 
     * @return Eigen::Ref<Eigen::MatrixXd> frame jacobian of the grasp frame in world frame
    */
    virtual Eigen::Ref<const Eigen::MatrixXd> frame_jacobian(const int idx, Eigen::Ref<Eigen::MatrixXd> J) = 0;

    /**
     * @brief Compute the local frame acceleration in world frame
     * 
     * @param grasp_idx index of the grasp frame
     * @param q generalized coordinates
     * @param v generalized velocities
     * @param a local acceleration of the grasp frame in world frame
     * 
     * @return Eigen::Ref<Eigen::VectorXd> local frame acceleration in world frame
    */
    virtual Eigen::Ref<const Eigen::VectorXd> frame_local_acc(const int idx, Eigen::Ref<Eigen::VectorXd> a) = 0;

    // pinocchio model
    pinocchio::Model model_;

protected:
    std::vector<pinocchio::FrameIndex> frame_ids_ = {};

    void getFrameId(const std::string &frame_name, pinocchio::FrameIndex &frame_id)
    {
        if (model_.existFrame(frame_name))
            frame_id = model_.getFrameId(frame_name);
        else
            throw(std::invalid_argument("frame: " + frame_name + " does not belong to the model\n"));
    };

private:
    /**
     * @brief initialize the model and get the frame ids
    */
    virtual void init(const std::vector<std::string> &frame_names) = 0;
};

} // namespace mars

#endif // MARS_MODEL_SE3_HPP