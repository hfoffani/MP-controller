#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
    private:
        bool silent = false;

    public:
        MPC();

        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

        // Sets the silent flag.
        void set_silent(bool silent);
};

#endif /* MPC_H */
