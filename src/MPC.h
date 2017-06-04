#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
    MPC();
    
    virtual ~MPC();
    
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    void Solve(const Eigen::VectorXd state, const Eigen::VectorXd coeffs, double& steering_angle, double& throttle, vector<double>& mpc_x_vals, vector<double>& mpc_y_vals);
    
private:
    
    // Return options for IPOPT solver
    std::string getOptionsString() const;
    
    // options for IPOPT solver
    const std::string _options;
};

#endif /* MPC_H */
