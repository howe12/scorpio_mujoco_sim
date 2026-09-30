#pragma once

#include "se2_grid_core/SE2Grid.hpp"
#include "se2_grid_core/iterators/SubmapIterator.hpp"

#include <cmath>
#include <Eigen/Core>

#include <memory>

namespace se2_grid 
{

  class EllipseIterator
  {
  public:

    EllipseIterator(const SE2Grid& se2Grid, const Eigen::Vector2d& center, 
        const Eigen::Array2d& length, double rotation = 0.0) : map(se2Grid), center_(center)
    {
      semiAxisSquare_ = (0.5 * length).square();
      double sinRotation = std::sin(rotation);
      double cosRotation = std::cos(rotation);
      transformMatrix_ << cosRotation, sinRotation, sinRotation, -cosRotation;
      mapLength_ = se2Grid.getLengthPos();
      mapPosition_ = se2Grid.getPosition();
      resolution_ = se2Grid.getResolutionPos();
      bufferSize_ = se2Grid.getSizePos();
      bufferStartIndex_ = se2Grid.getStartIndex();

      Eigen::Array2i submapStartIndex;
      Eigen::Array2i submapBufferSize;
      Eigen::Array3i submapStartIndex3;

      const Eigen::Rotation2Dd rotationMatrix(rotation);
      Eigen::Vector2d u = rotationMatrix * Eigen::Vector2d(length(0)*0.5, 0.0);
      Eigen::Vector2d v = rotationMatrix * Eigen::Vector2d(0.0, length(1)*0.5);
      
      // origin ???
      // const Eigen::Vector2d boundingBoxHalfLength = (u.cwiseAbs2()*4.0 + v.cwiseAbs2()*4.0).array().sqrt();
      // Eigen::Vector2d topLeft = center + boundingBoxHalfLength;
      // Eigen::Vector2d bottomRight = center - boundingBoxHalfLength;

      // AABB method
      std::vector<Eigen::Vector2d> poses;
      poses.emplace_back(center-u-v);
      poses.emplace_back(center-u+v);
      poses.emplace_back(center+u-v);
      Eigen::Vector2d topLeft = center+u+v;
      Eigen::Vector2d bottomRight = topLeft;
      for (auto& p : poses)
      {
        for (int i=0; i<2; i++)
        {
          if (topLeft(i) < p(i))
            topLeft(i) = p(i);
          if (bottomRight(i) > p(i))
            bottomRight(i) = p(i);
        }
      }

      Eigen::Vector3d topLeft3 = Eigen::Vector3d::Zero();
      Eigen::Vector3d bottomRight3 = Eigen::Vector3d::Zero();
      topLeft3.head(2) = topLeft;
      bottomRight3.head(2) = bottomRight;
      se2Grid.boundPos(topLeft3);
      se2Grid.boundPos(bottomRight3);
      se2Grid.pos2Index(topLeft3, submapStartIndex3);
      Eigen::Array2i endIndex;
      Eigen::Array3i endIndex3;
      se2Grid.pos2Index(bottomRight3, endIndex3);
      endIndex = endIndex3.head(2);

      submapStartIndex = submapStartIndex3.head(2);

      Eigen::Array2i tempStart = submapStartIndex;
      if (!se2Grid.isDefaultStartIndex())
      {
        tempStart -= bufferStartIndex_;
        endIndex -= bufferStartIndex_;
        se2Grid.wrapIndex2(tempStart);
        se2Grid.wrapIndex2(endIndex);
      }
      submapBufferSize = endIndex - tempStart + Eigen::Array2i::Ones();
      internalIterator_ = std::make_shared<SubmapIterator>(se2Grid, submapStartIndex, submapBufferSize);
      if (!isInside())
        ++(*this);
    }

    inline bool operator !=(const EllipseIterator& other) const
    {
      return (internalIterator_ != other.internalIterator_);
    }

    inline const Eigen::Array3i& operator *() const
    {
      return *(*internalIterator_);
    }

    inline EllipseIterator& operator ++()
    {
      ++(*internalIterator_);
      if (internalIterator_->isPastEnd())
        return *this;

      for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_))
        if (isInside())
          break;

      return *this;
    }

    inline bool isPastEnd() const
    {
      return internalIterator_->isPastEnd();
    }

    inline const Eigen::Array2i& getSubmapSize() const
    {
      return internalIterator_->getSubmapSize();
    }

  private:

    inline bool isInside() const
    {
      Eigen::Vector3d p;
      map.index2Pos(*(*internalIterator_), p);
      double value = ((transformMatrix_ * (p.head(2) - center_)).array().square() / semiAxisSquare_).sum();
      return (value <= 1.0);
    }

    const SE2Grid& map;

    Eigen::Vector2d center_;

    Eigen::Array2d semiAxisSquare_;

    Eigen::Matrix2d transformMatrix_;

    std::shared_ptr<SubmapIterator> internalIterator_;

    Eigen::Array2d mapLength_;
    Eigen::Vector2d mapPosition_;
    double resolution_;
    Eigen::Array2i bufferSize_;
    Eigen::Array2i bufferStartIndex_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}  // namespace se2_grid
