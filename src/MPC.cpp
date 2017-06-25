#include "MPC.h"
#include "bounds.h"
#include "util.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    /*
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints
      fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    }
    */
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

typedef CPPAD_TESTVECTOR(double) Dvector;

void SetBound(Dvector& var_bound,
              size_t start_i,
              size_t end_i,
              double bound)
{
  for (size_t i = start_i; i < end_i; ++i)
  {
    var_bound[i] = bound;
  }
}

void SetBounds(Dvector& vars_lowerbound,
               Dvector& vars_upperbound,
               size_t start_i,
               size_t end_i,
               double lower,
               double upper)
{
  SetBound(vars_lowerbound, start_i, end_i, lower);
  SetBound(vars_upperbound, start_i, end_i, upper);
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 6 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  const size_t x_start = 0;
  const size_t y_start = 1 * N;
  const size_t psi_start = 2 * N;
  const size_t v_start = 3 * N;
  const size_t cte_start = 4 * N;
  const size_t epsi_start = 5 * N;
  const size_t delta_start = 6 * (N - 1);
  const size_t a_start = 7 * (N - 1);

  for (int i = 0; i < state.size(); ++i)
  {
    vars[i * N] = state[i];
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.

  static const double max_angle_rad = deg2rad(180);
  static const double max_steer_rad = deg2rad(25);

  BoundSetter bounds(vars_lowerbound, vars_upperbound);

  bounds.SetBounds(x_start, y_start, numeric_limits<double>::min(), numeric_limits<double>::max());
  bounds.SetBounds(y_start, psi_start, numeric_limits<double>::min(), numeric_limits<double>::max());
  bounds.SetBounds(psi_start, v_start, -max_angle_rad, max_angle_rad);
  bounds.SetBounds(v_start, cte_start, 0, 60);
  bounds.SetBounds(cte_start, epsi_start, numeric_limits<double>::min(), numeric_limits<double>::max());
  bounds.SetBounds(epsi_start, delta_start, -max_angle_rad, max_angle_rad);
  bounds.SetBounds(delta_start, a_start, -max_steer_rad, max_steer_rad);
  bounds.SetBounds(a_start, n_vars, -1, 1);

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
