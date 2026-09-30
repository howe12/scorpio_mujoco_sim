#include "se2_grid_core/SE2Grid.hpp"

namespace se2_grid
{
    SE2Grid::SE2Grid(const std::vector<std::string>& layers_, const std::vector<bool>& has_so2_)
    {
        position.setZero();
        length_pos.setZero();
        length_yaw = 2.0 * M_PI + 0.02;
        resolution_pos = resolution_yaw = 0.0;
        size_pos.setZero();
        size_yaw = 0;
        start_index.setZero();
        layers = layers_;
        
        for (size_t i=0; i<layers.size(); i++)
        {
            std::vector<Eigen::MatrixXf> l;
            l.emplace_back(Eigen::MatrixXf());
            data.insert(std::pair<std::string, std::vector<Eigen::MatrixXf>>(layers[i], l));
            has_so2.insert(std::pair<std::string, bool>(layers[i], has_so2_[i]));
        }
    }

    void SE2Grid::initGeometry(const Eigen::Array2d& length_pos_, const double resolution_pos_,
                                const double resolution_yaw_, const Eigen::Vector2d& position_)
    {
      resolution_pos_inv = 1.0 / resolution_pos_;
      resolution_yaw_inv = 1.0 / resolution_yaw_;
      size_pos(0) = static_cast<int>(ceil(length_pos_(0) / resolution_pos_));
      size_pos(1) = static_cast<int>(ceil(length_pos_(1) / resolution_pos_));
      size_yaw = static_cast<int>(ceil(length_yaw / resolution_yaw_));

      for (size_t i=0; i<layers.size(); i++)
      {
        std::vector<Eigen::MatrixXf>& d = data.at(layers[i]);
        if (has_so2.at(layers[i]))
        {
          d.clear();
          for (int j=0; j<size_yaw; j++)
          {
            Eigen::MatrixXf m;
            m.resize(size_pos(0), size_pos(1));
            d.emplace_back(m);
          }
        }
        else
        {
          d[0].resize(size_pos(0), size_pos(1));
        }
      }
      clearAll();

      resolution_pos = resolution_pos_;
      resolution_yaw = resolution_yaw_;
      length_pos = (size_pos.cast<double>() * resolution_pos).matrix();
      position = position_;
      origin = (0.5 * length_pos).matrix();
      start_index.setZero();
    }

    void SE2Grid::convertToDefaultStartIndex()
    {
      if (isDefaultStartIndex())
        return;

      Eigen::Array2i bottom_right = start_index + size_pos - Eigen::Array2i::Ones();
      wrapIndex2(bottom_right);

      Quadrant quadrantOfTopLeft = getQuadrant(start_index);
      Quadrant quadrantOfBottomRight = getQuadrant(bottom_right);
      std::vector<Region> regions;

      if (quadrantOfTopLeft == Quadrant::TopLeft)
      {
        if (quadrantOfBottomRight == Quadrant::TopLeft)
        {
          regions.emplace_back(start_index, size_pos, Quadrant::TopLeft);
        }

        if (quadrantOfBottomRight == Quadrant::TopRight)
        {
          Eigen::Array2i topLeftSize(size_pos(0), size_pos(1) - start_index(1));
          regions.emplace_back(start_index, topLeftSize, Quadrant::TopLeft);

          Eigen::Array2i topRightIndex(start_index(0), 0);
          Eigen::Array2i topRightSize(size_pos(0), size_pos(1) - topLeftSize(1));
          regions.emplace_back(topRightIndex, topRightSize, Quadrant::TopRight);
        }

        if (quadrantOfBottomRight == Quadrant::BottomLeft)
        {
          Eigen::Array2i topLeftSize(size_pos(0) - start_index(0), size_pos(1));
          regions.emplace_back(start_index, topLeftSize, Quadrant::TopLeft);

          Eigen::Array2i bottomLeftIndex(0, start_index(1));
          Eigen::Array2i bottomLeftSize(size_pos(0) - topLeftSize(0), size_pos(1));
          regions.emplace_back(bottomLeftIndex, bottomLeftSize, Quadrant::BottomLeft);
        }

        if (quadrantOfBottomRight == Quadrant::BottomRight)
        {
          Eigen::Array2i topLeftSize(size_pos(0) - start_index(0), size_pos(1) - start_index(1));
          regions.emplace_back(start_index, topLeftSize, Quadrant::TopLeft);

          Eigen::Array2i topRightIndex(start_index(0), 0);
          Eigen::Array2i topRightSize(size_pos(0) - start_index(0), size_pos(1) - topLeftSize(1));
          regions.emplace_back(topRightIndex, topRightSize, Quadrant::TopRight);

          Eigen::Array2i bottomLeftIndex(0, start_index(1));
          Eigen::Array2i bottomLeftSize(size_pos(0) - topLeftSize(0), size_pos(1) - start_index(1));
          regions.emplace_back(bottomLeftIndex, bottomLeftSize, Quadrant::BottomLeft);

          Eigen::Array2i bottomRightIndex = Eigen::Array2i::Zero();
          Eigen::Array2i bottomRightSize(bottomLeftSize(0), topRightSize(1));
          regions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
        }
      }
      else if (quadrantOfTopLeft == Quadrant::TopRight)
      {
        if (quadrantOfBottomRight == Quadrant::TopRight)
        {
          regions.emplace_back(start_index, size_pos, Quadrant::TopRight);
        }

        if (quadrantOfBottomRight == Quadrant::BottomRight)
        {
          Eigen::Array2i topRightSize(size_pos(0) - start_index(0), size_pos(1));
          regions.emplace_back(start_index, topRightSize, Quadrant::TopRight);

          Eigen::Array2i bottomRightIndex(0, start_index(1));
          Eigen::Array2i bottomRightSize(size_pos(0) - topRightSize(0), size_pos(1));
          regions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
        }

      }
      else if (quadrantOfTopLeft == Quadrant::BottomLeft)
      {
        if (quadrantOfBottomRight == Quadrant::BottomLeft)
        {
          regions.emplace_back(start_index, size_pos, Quadrant::BottomLeft);
        }

        if (quadrantOfBottomRight == Quadrant::BottomRight)
        {
          Eigen::Array2i bottomLeftSize(size_pos(0), size_pos(1) - start_index(1));
          regions.emplace_back(start_index, bottomLeftSize, Quadrant::BottomLeft);

          Eigen::Array2i bottomRightIndex(start_index(0), 0);
          Eigen::Array2i bottomRightSize(size_pos(0), size_pos(1) - bottomLeftSize(1));
          regions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
        }

      }
      else if (quadrantOfTopLeft == Quadrant::BottomRight)
      {
        if (quadrantOfBottomRight == Quadrant::BottomRight) {
          regions.emplace_back(start_index, size_pos, Quadrant::BottomRight);
        }
      }

      for (auto& d : data)
      {
        auto temp(d.second);
        for (const auto& bufferRegion : regions) 
        {
          Eigen::Array2i index = bufferRegion.index;
          Eigen::Array2i size = bufferRegion.size;

          if (bufferRegion.quadrant == Quadrant::TopLeft)
          {
            for (size_t i=0; i<temp.size(); i++)
              temp[i].topLeftCorner(size(0), size(1)) = d.second[i].block(index(0), index(1), size(0), size(1));
          } else if (bufferRegion.quadrant == Quadrant::TopRight)
          {
            for (size_t i=0; i<temp.size(); i++)
              temp[i].topRightCorner(size(0), size(1)) = d.second[i].block(index(0), index(1), size(0), size(1));
          } else if (bufferRegion.quadrant == Quadrant::BottomLeft)
          {
            for (size_t i=0; i<temp.size(); i++)
              temp[i].bottomLeftCorner(size(0), size(1)) = d.second[i].block(index(0), index(1), size(0), size(1));
          } else if (bufferRegion.quadrant == Quadrant::BottomRight)
          {
            for (size_t i=0; i<temp.size(); i++)
              temp[i].bottomRightCorner(size(0), size(1)) = d.second[i].block(index(0), index(1), size(0), size(1));
          }
        }
        d.second = temp;;
      }
      
      start_index.setZero();
    }

    float SE2Grid::getValue(const std::string& layer, const Eigen::Vector3d& pos,
                            InterpolationMethods interpolationMethod) const
    {
      bool skipNextSwitchCase = false;
      switch (interpolationMethod)
      {
        case InterpolationMethods::INTER_CUBIC_CONVOLUTION: 
        {
          float value;
          if (getCubicConvolutionInterpolated(layer, pos, value))
          {
            return value;
          } else
          {
            interpolationMethod = InterpolationMethods::INTER_LINEAR;
            skipNextSwitchCase = true;
            std::cout<<"\033[33m Cubic Convolution fail, using Linear. \033[0m"<<std::endl;
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_CUBIC:
        {
          if (!skipNextSwitchCase)
          {
            float value;
            if (getCubicInterpolated(layer, pos, value))
            {
              return value;
            } else
            {
              interpolationMethod = InterpolationMethods::INTER_LINEAR;
              std::cout<<"\033[33m Cubic Convolution fail, using Linear. \033[0m"<<std::endl;
            }
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_LINEAR:
        {
          float value;
          if (getLinearInterpolated(layer, pos, value))
          {
            return value;
          }
          else 
          {
            interpolationMethod = InterpolationMethods::INTER_NEAREST;
            std::cout<<"\033[33m Linear fail, using Nearest. \033[0m"<<std::endl;
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_NEAREST:
        {
          Eigen::Array3i index;
          if (pos2Index(pos, index))
          {
            return at(layer, index);
          }
          break;
        }
        default:
          std::cout<<"\033[31m Nearest fail, Pos out of map, return 0. \033[0m"<<std::endl;
      }

      return 0.0;
    }

    float SE2Grid::getValueGrad(const std::string& layer, const Eigen::Vector3d& pos, Eigen::Vector3d& grad, 
                                InterpolationMethods interpolationMethod) const
    {
      bool skipNextSwitchCase = false;
      switch (interpolationMethod)
      {
        case InterpolationMethods::INTER_CUBIC_CONVOLUTION: 
        {
          float value;
          if (getCubicConvolutionInterpolated(layer, pos, value))
          {
            return value;
          } else
          {
            interpolationMethod = InterpolationMethods::INTER_LINEAR;
            skipNextSwitchCase = true;
            std::cout<<"\033[33m Cubic Convolution fail, using Linear. \033[0m"<<std::endl;
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_CUBIC:
        {
          if (!skipNextSwitchCase)
          {
            float value;
            if (getCubicInterpolated(layer, pos, value))
            {
              return value;
            } else
            {
              interpolationMethod = InterpolationMethods::INTER_LINEAR;
              std::cout<<"\033[33m Cubic Convolution fail, using Linear. \033[0m"<<std::endl;
            }
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_LINEAR:
        {
          float value;
          if (getLinearInterpolatedGrad(layer, pos, value, grad))
          {
            return value;
          }
          else 
          {
            interpolationMethod = InterpolationMethods::INTER_NEAREST;
            std::cout<<"\033[33m Linear fail, using Nearest. \033[0m"<<std::endl;
          }
          [[fallthrough]];
        }
        case InterpolationMethods::INTER_NEAREST:
        {
          Eigen::Array3i index;
          if (pos2Index(pos, index))
          {
            return at(layer, index);
          }
          break;
        }
        default:
          std::cout<<"\033[31m Nearest fail, Pos out of map, return 0. \033[0m"<<std::endl;
      }

      return 0.0;
    }

    bool SE2Grid::move(const Eigen::Vector2d& pos2)
    {
        Eigen::Array2i indexShift;
        Eigen::Vector2d positionShift = pos2 - position;
        
        Eigen::Vector2d positionShiftTemp = (positionShift.array() * resolution_pos_inv).matrix();
        for (int i = 0; i < indexShift.size(); i++)
            indexShift[i] = static_cast<int>(positionShiftTemp[i] + 0.5 * (positionShiftTemp[i] > 0 ? 1 : -1));

        Eigen::Vector2d alignedPositionShift = (indexShift.cast<double>() * resolution_pos).matrix();
        indexShift = -indexShift;
        // Delete fields that fall out of map (and become empty cells).
        for (int i = 0; i < indexShift.size(); i++)
        {
            if (indexShift(i) != 0)
            {
                if (abs(indexShift(i)) >= size_pos(i))
                    clearAll();
                else
                {
                    // Drop cells out of map.
                    int sign = (indexShift(i) > 0 ? 1 : -1);
                    int startIndex = start_index(i) - (sign < 0 ? 1 : 0);
                    int endIndex = startIndex - sign + indexShift(i);
                    int nCells = abs(indexShift(i));
                    int index = (sign > 0 ? startIndex : endIndex);
                    wrapIndexOne(index, size_pos(i));

                    if (index + nCells <= size_pos(i))
                    {
                        // One region to drop.
                        if (i == 0)
                            clearRows(index, nCells);
                        else if (i == 1)
                            clearCols(index, nCells);
                    }
                    else
                    {
                        // Two regions to drop.
                        int firstIndex = index;
                        int firstNCells = size_pos(i) - firstIndex;
                        if (i == 0)
                            clearRows(firstIndex, firstNCells);
                        else if (i == 1)
                            clearCols(firstIndex, firstNCells);

                        int secondIndex = 0;
                        int secondNCells = nCells - firstNCells;
                        if (i == 0)
                            clearRows(secondIndex, secondNCells);
                        else if (i == 1)
                            clearCols(secondIndex, secondNCells);
                    }
                }
            }
        }

        // Update information.
        start_index += indexShift;
        wrapIndex2(start_index);
        position += alignedPositionShift;
        return indexShift.any();
    }
}
