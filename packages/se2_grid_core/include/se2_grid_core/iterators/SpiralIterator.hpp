#pragma once

#include "se2_grid_core/SE2Grid.hpp"

#include <cmath>
#include <Eigen/Core>

#include <memory>

namespace se2_grid 
{
    class SpiralIterator
    {
    public:

        SpiralIterator(const SE2Grid& se2Grid, const Eigen::Vector2d& center, double radius)
            : center_(center), radius_(radius), map(se2Grid), distance_(0)
        {
            radiusSquare_ = radius_ * radius_;
            mapLength_ = se2Grid.getLengthPos();
            mapPosition_ = se2Grid.getPosition();
            resolution_ = se2Grid.getResolutionPos();
            bufferSize_ = se2Grid.getSizePos();
            startIndex_ = se2Grid.getStartIndex();

            Eigen::Vector3d center3 = Eigen::Vector3d::Zero();
            center3.head(2) = center;
            se2Grid.pos2Index(center3, indexCenter_);
            nRings_ = std::ceil(radius_ / resolution_);

            if (indexCenter_[0] >= 0 && indexCenter_[1] >= 0 &&
                indexCenter_[0] < bufferSize_[0] && indexCenter_[1] < bufferSize_[1])
            {
                pointsRing_.push_back(indexCenter_);
            } else
            {
                while (pointsRing_.empty() && !isPastEnd())
                    generateRing();
            }
        }

        inline bool operator !=(const SpiralIterator& /*other*/) const
        {
            return (pointsRing_.back() != pointsRing_.back()).any();
        }

        inline const Eigen::Array3i& operator *() const
        {
            return pointsRing_.back();
        }

        inline SpiralIterator& operator ++()
        {
            pointsRing_.pop_back();
            if (pointsRing_.empty() && !isPastEnd())
                generateRing();
            return *this;
        }

        inline bool isPastEnd() const
        {
            return (distance_ == nRings_ && pointsRing_.empty());
        }

    private:

        inline static int signum(const int val)
        {
            return static_cast<int>(0 < val) - static_cast<int>(val < 0);
        }

        inline bool isInside(const Eigen::Array3i& index) const
        {
            Eigen::Vector3d p;
            map.index2Pos(index, p);
            double squareNorm = (p.head(2) - center_).array().square().sum();
            return (squareNorm <= radiusSquare_);
        }

        inline void generateRing()
        {
            distance_++;
            Eigen::Array2i point(distance_, 0);
            Eigen::Array3i pointInMap = Eigen::Array3i::Zero();
            Eigen::Array2i normal;
            do {
                // origin: you should let se2Grid convertToDefaultStartIndex
                // pointInMap.x() = point.x() + indexCenter_.x();
                // pointInMap.y() = point.y() + indexCenter_.y();
                // if (pointInMap[0] >= 0 && pointInMap[1] >= 0 &&
                //     pointInMap[0] < bufferSize_[0] && pointInMap[1] < bufferSize_[1])
                // {
                    // if (distance_ == nRings_ || distance_ == nRings_ - 1)
                    // {
                    //     if (isInside(pointInMap))
                    //         pointsRing_.push_back(pointInMap);
                    // } else
                    // {
                    //     pointsRing_.push_back(pointInMap);
                    // }
                // }
                // new: you needn't let se2Grid convertToDefaultStartIndex
                Eigen::Array2i unwrapperIndexCenter_ = indexCenter_.head(2) - startIndex_;
                for (int i = 0; i < 2; i++)
                    SE2Grid::wrapIndexOne(unwrapperIndexCenter_(i), bufferSize_(i));
                pointInMap.head(2) = point + unwrapperIndexCenter_;
                if (pointInMap[0] >= 0 && pointInMap[1] >= 0 &&
                    pointInMap[0] < bufferSize_[0] && pointInMap[1] < bufferSize_[1])
                {
                    pointInMap.head(2) += startIndex_;
                    for (int i = 0; i < 2; i++)
                        SE2Grid::wrapIndexOne(pointInMap(i), bufferSize_(i));
                    if (isInside(pointInMap))
                            pointsRing_.push_back(pointInMap);
                }

                normal.x() = -signum(point.y());
                normal.y() = signum(point.x());
                if (normal.x() != 0 && static_cast<unsigned int>(Eigen::Vector2d(point.x() + normal.x(), point.y()).norm()) == distance_)
                    point.x() += normal.x();
                else if (normal.y() != 0 && static_cast<unsigned int>(Eigen::Vector2d(point.x(), point.y() + normal.y()).norm()) == distance_)
                    point.y() += normal.y();
                else
                {
                    point.x() += normal.x();
                    point.y() += normal.y();
                }
            } while (static_cast<unsigned int>(point.x()) != distance_ || point.y() != 0);
        }

        Eigen::Vector2d center_;
        Eigen::Array3i indexCenter_;

        double radius_;

        double radiusSquare_;

        const SE2Grid& map;

        unsigned int nRings_;
        unsigned int distance_;
        std::vector<Eigen::Array3i> pointsRing_;

        Eigen::Array2d mapLength_;
        Eigen::Vector2d mapPosition_;
        double resolution_;
        Eigen::Array2i bufferSize_;
        Eigen::Array2i startIndex_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
