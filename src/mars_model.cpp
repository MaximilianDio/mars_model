#include "mars_model/mars_model.hpp"
#include "eigen3/Eigen/Dense"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/workspace/model_and_simulation/mars_model/models/iiwa7"
#endif

namespace mars
{

MarsModel::MarsModel()
{
    Eigen::Vector2d a;
    a << 1, 2;

    std::cout << "a = " << a.transpose() << std::endl;
    using namespace pinocchio;

    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa7.urdf");

    // Load the urdf model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << std::endl;
}

} // namespace mars
