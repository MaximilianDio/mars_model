#include "mars_model/mars_model.hpp"
#include <iostream>


int main()
{
    std::cout << "## Unconstrained Model ##" << std::endl;

    // Create a model with 2 variables and 1 constraint
    mars::MarsModel model;

    // Print the solution
    std::cout << "Solution: " << model.test_value << std::endl;

    return 0;
}