#include "bounds.h"

BoundSetter::BoundSetter(Dvector& vars_lowerbound, Dvector& vars_upperbound)
  : m_vars_lowerbound(vars_lowerbound),
    m_vars_upperbound(vars_upperbound)
{
}

void BoundSetter::SetBounds(size_t start_i,
                            size_t end_i,
                            double lower,
                            double upper)
{
  SetBound(m_vars_lowerbound, start_i, end_i, lower);
  SetBound(m_vars_upperbound, start_i, end_i, upper);
}

void BoundSetter::SetBound(Dvector& var_bound,
                           size_t start_i,
                           size_t end_i,
                           double bound)
{
  for (size_t i = start_i; i < end_i; ++i)
  {
    var_bound[i] = bound;
  }
}
