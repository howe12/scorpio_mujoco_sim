#pragma once

#include "se2_grid_core/SE2Grid.hpp"

#include <Eigen/Core>

namespace se2_grid
{

class SubmapIterator
{
public:
    SubmapIterator(const se2_grid::SE2Grid& se2Grid, const Eigen::Array2i& submapStartIndex,
                    const Eigen::Array2i& submapSize)
    {
        size_ = se2Grid.getSizePos();
        startIndex_ = se2Grid.getStartIndex();
        index_ = submapStartIndex;
        index_3.head(2) = index_;
        submapSize_ = submapSize;
        submapStartIndex_ = submapStartIndex;
        submapIndex_.setZero();
        isPastEnd_ = false;
    }

    SubmapIterator(const SubmapIterator* other)
    {
        size_ = other->size_;
        startIndex_ = other->startIndex_;
        submapSize_ = other->submapSize_;
        submapStartIndex_ = other->submapStartIndex_;
        index_ = other->index_;
        index_3 = other->index_3;
        submapIndex_ = other->submapIndex_;
        isPastEnd_ = other->isPastEnd_;
    }

    inline bool operator !=(const SubmapIterator& other) const
    {
        return (index_ != other.index_).any();
    }

    inline const Eigen::Array3i& operator *() const
    {
        return index_3;
    }

    inline const Eigen::Array2i& getSubmapIndex() const
    {
        return submapIndex_;
    }

    inline SubmapIterator& operator ++()
    {
        Eigen::Array2i tempIndex = index_;
        Eigen::Array2i tempSubmapIndex = submapIndex_;

        if (tempSubmapIndex[1] + 1 < submapSize_[1])
        {
            tempSubmapIndex[1]++;
        } else 
        {
            tempSubmapIndex[0]++;
            tempSubmapIndex[1] = 0;
        }

        if (tempSubmapIndex[0] >= 0 && tempSubmapIndex[1] >= 0 &&
            tempSubmapIndex[0] < submapSize_[0] && tempSubmapIndex[1] < submapSize_[1])
        {
            Eigen::Array2i unwrappedSubmapTopLeftIndex;

            if ((startIndex_==0).all())
                unwrappedSubmapTopLeftIndex = submapStartIndex_;
            else
            {
                unwrappedSubmapTopLeftIndex = submapStartIndex_ - startIndex_;
                for (int i = 0; i < unwrappedSubmapTopLeftIndex.size(); i++)
                    SE2Grid::wrapIndexOne(unwrappedSubmapTopLeftIndex(i), size_(i));
            }
            
            tempIndex = unwrappedSubmapTopLeftIndex + tempSubmapIndex;
            if (!((startIndex_==0).all()))
            {
                tempIndex += startIndex_;
                for (int i = 0; i < tempIndex.size(); i++)
                    SE2Grid::wrapIndexOne(tempIndex(i), size_(i));
            }

            index_ = tempIndex;
            index_3.head(2) = index_;
            submapIndex_ = tempSubmapIndex;
        }
        else
        {
            isPastEnd_ = true;
        }
        
        return *this;
    }

    inline bool isPastEnd() const
    {
        return isPastEnd_;
    }

    inline const Eigen::Array2i& getSubmapSize() const
    {
        return submapSize_;
    }

private:

  Eigen::Array2i size_;

  Eigen::Array2i startIndex_;

  Eigen::Array2i index_;

  Eigen::Array3i index_3 = Eigen::Array3i::Zero();

  Eigen::Array2i submapSize_;

  Eigen::Array2i submapStartIndex_;

  Eigen::Array2i submapIndex_;

  bool isPastEnd_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace se2_grid

