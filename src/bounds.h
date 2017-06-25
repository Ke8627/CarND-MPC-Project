#ifndef BOUNDS_H
#define BOUNDS_H

#include <cppad/cppad.hpp>

typedef CPPAD_TESTVECTOR(double) Dvector;

class BoundSetter
{
public:
  BoundSetter(Dvector& vars_lowerbound, 
              Dvector& vars_upperbound);

  void SetBounds(size_t start_i, 
                 size_t end_i, 
                 double lower, 
                 double upper);

private:
  void SetBound(Dvector& var_bound,
                size_t start_i,
                size_t end_i,
                double bound);

  Dvector& m_vars_lowerbound;
  Dvector& m_vars_upperbound;
};

#endif /* BOUNDS_H */
