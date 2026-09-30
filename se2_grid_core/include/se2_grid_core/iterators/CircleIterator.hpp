#pragma once

#include "se2_grid_core/SE2Grid.hpp"
#include "se2_grid_core/iterators/SubmapIterator.hpp"

#include <cmath>
#include <Eigen/Core>

#include <memory>

namespace se2_grid 
{
    class CircleIterator
    {
    public:

        CircleIterator(const SE2Grid& se2Grid, const Eigen::Vector2d& center, double radius)
            : center_(center), radius_(radius), map(se2Grid)
        {
            radiusSquare_ = radius_ * radius_;
            mapLength_ = se2Grid.getLengthPos();
            mapPosition_ = se2Grid.getPosition();
            resolution_ = se2Grid.getResolutionPos();
            bufferSize_ = se2Grid.getSizePos();
            bufferStartIndex_ = se2Grid.getStartIndex();

            Eigen::Array2i submapStartIndex;
            Eigen::Array2i submapBufferSize;
            Eigen::Array3i submapStartIndex3;

            Eigen::Vector2d topLeft = center.array() + radius;
            Eigen::Vector2d bottomRight = center.array() - radius;
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
            if(!isInside())
                ++(*this);
        }

        inline bool operator !=(const CircleIterator& other) const
        {
            return (internalIterator_ != other.internalIterator_);
        }

        inline const Eigen::Array3i& operator *() const
        {
            return *(*internalIterator_);
        }

        inline CircleIterator& operator ++()
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

    private:

        inline bool isInside() const
        {
            Eigen::Vector3d p;
            map.index2Pos(*(*internalIterator_), p);
            double squareNorm = (p.head(2) - center_).array().square().sum();
            return (squareNorm <= radiusSquare_);
        }

        Eigen::Vector2d center_;

        double radius_;

        const SE2Grid& map;

        double radiusSquare_;

        std::shared_ptr<SubmapIterator> internalIterator_;

        Eigen::Array2d mapLength_;
        Eigen::Vector2d mapPosition_;
        double resolution_;
        Eigen::Array2i bufferSize_;
        Eigen::Array2i bufferStartIndex_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

