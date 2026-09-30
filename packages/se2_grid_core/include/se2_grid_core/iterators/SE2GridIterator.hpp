#pragma once

#include "se2_grid_core/SE2Grid.hpp"

#include <Eigen/Core>

namespace se2_grid {

class SE2GridIterator
{
public:

  SE2GridIterator(const se2_grid::SE2Grid &se2Grid)
  {
    size_ = se2Grid.getSizePos();
    startIndex_ = se2Grid.getStartIndex();
    linearSize_ = size_.prod();
    linearIndex_ = 0;
    isPastEnd_ = false;
  }

  SE2GridIterator(const SE2GridIterator* other)
  {
    size_ = other->size_;
    startIndex_ = other->startIndex_;
    linearSize_ = other->linearSize_;
    linearIndex_ = other->linearIndex_;
    isPastEnd_ = other->isPastEnd_;
  }

  inline bool operator !=(const SE2GridIterator& other) const
  {
    return linearIndex_ != other.linearIndex_;
  }

  inline Eigen::Array3i operator *() const
  {
    return Eigen::Array3i((int)linearIndex_ % size_(0), (int)linearIndex_ / size_(0), 0);
  }

  inline const size_t& getLinearIndex() const
  {
    return linearIndex_;
  }

  inline void wrapIndexToRange(int& idx, int buffer_size) const
  {
    if (idx < buffer_size)
      {
        if(idx >= 0)
          return;
        else if(idx >= -buffer_size)
        {
          idx += buffer_size;
          return;
        }
        else
        {
          idx = idx % buffer_size;
          idx += buffer_size;
        }
      }
      else if(idx < buffer_size*2)
      {
        idx -= buffer_size;
        return;
      }
      else
        idx = idx % buffer_size;
  }

  inline Eigen::Array3i getUnwrappedIndex() const
  {
    Eigen::Array3i wrapper_index = *(*this);
    if (!(startIndex_ == 0).all())
    {
      Eigen::Array2i index = wrapper_index.head(2) - startIndex_;
      wrapIndexToRange(index(0), size_(0));
      wrapIndexToRange(index(1), size_(1));
      wrapper_index.head(2) = index;
    }
    return wrapper_index;
  }

  inline SE2GridIterator& operator ++()
  {
    size_t newIndex = linearIndex_ + 1;
    if (newIndex < linearSize_)
    {
      linearIndex_ = newIndex;
    } else
    {
      isPastEnd_ = true;
    }
    return *this;
  }

  inline SE2GridIterator end() const
  {
    SE2GridIterator res(this);
    res.linearIndex_ = linearSize_ - 1;
    return res;
  }

  inline bool isPastEnd() const
  {
    return isPastEnd_;
  }

 protected:

  Eigen::Array2i size_;

  Eigen::Array2i startIndex_;

  size_t linearSize_;

  size_t linearIndex_;

  bool isPastEnd_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace se2_grid

