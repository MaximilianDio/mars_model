
#ifndef MARS_MODEL_SE3_HPP
#define MARS_MODEL_SE3_HPP

#include <string>
#include <vector>

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

        T_com_ref_ = model_.frames[ref_frame_id_].placement;
        T_com_measured_ = model_.frames[measured_frame_id_].placement;

        T_measured_grasp_.reserve(grasp_frame_ids_.size());
        T_com_grasp_.reserve(grasp_frame_ids_.size());
        // T_mocap2grasp
        for (const auto &frame_id : grasp_frame_ids_)
        {
            T_measured_grasp_.push_back(model_.frames[frame_id].placement);

            // T_com_grasp[i] = T_com_mocap * T_mocap_grasp[i]
            T_com_grasp_.push_back(T_com_measured_ * model_.frames[frame_id].placement);
        }

        std::cout << "Object: Number of graspframes: " << grasp_frame_ids_.size() << "\n";
    }


private:
    // pinocchio model
    pinocchio::Model model_;

    std::vector<pinocchio::JointIndex> grasp_frame_ids_ = {};
    pinocchio::JointIndex com_frame_id_;
    pinocchio::JointIndex measured_frame_id_;
    pinocchio::JointIndex ref_frame_id_;

    // relative frames
    pinocchio::SE3 T_com_measured_;                // transformation from measurement to com frame
    pinocchio::SE3 T_com_ref_;                     // transformation from ref to com frames
    std::vector<pinocchio::SE3> T_measured_grasp_; // transformation from grasp to mocap frames
    std::vector<pinocchio::SE3> T_com_grasp_;      // transformation from grasp to com frames
};

} // namespace mars

#endif // MARS_MODEL_SE3_HPP