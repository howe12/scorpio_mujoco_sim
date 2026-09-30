#pragma once

namespace se2_grid 
{

  template<typename Scalar>
  struct Clamp
  {
    Clamp(const Scalar& min, const Scalar& max)
        : min_(min),
          max_(max)
    {
    }
    Scalar operator()(const Scalar& x) const
    {
      return x < min_ ? min_ : (x > max_ ? max_ : x);
    }
    Scalar min_, max_;
  };

}  // namespace se2_grid
