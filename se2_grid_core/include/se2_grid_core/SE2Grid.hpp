#pragma once

#include <unordered_map>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI_M2 (2.0 * M_PI)

namespace se2_grid 
{
  /*
  * Interpolations are ordered in the order
  * of increasing accuracy and computational complexity.
  * INTER_NEAREST - fastest, but least accurate,
  * INTER_CUBIC - slowest, but the most accurate.
  * see:
  * https://en.wikipedia.org/wiki/Bicubic_interpolation
  * https://web.archive.org/web/20051024202307/http://www.geovista.psu.edu/sites/geocomp99/Gc99/082/gc_082.htm
  * for more info. Cubic convolution algorithm is also known as piecewise cubic
  * interpolation and in general does not guarantee continuous
  * first derivatives.
  */
  enum class InterpolationMethods
  {
    INTER_NEAREST,            // nearest neighbor interpolation
    INTER_LINEAR,             // bilinear interpolation
    INTER_CUBIC_CONVOLUTION,  // piecewise bicubic interpolation using convolution algorithm
    INTER_CUBIC               // standard bicubic interpolation
  };

  enum class Quadrant
  {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
  };
  
  class Region
  {
  public:
    Eigen::Array2i size;
    Eigen::Array2i index;
    Quadrant quadrant;
    Region() : size(Eigen::Array2i::Zero()), index(Eigen::Array2i::Zero()), quadrant(Quadrant::TopLeft) {}
    Region(Eigen::Array2i idx_, Eigen::Array2i size_, Quadrant q) : size(size_), index(idx_), quadrant(q) {}
  };

  class SE2Grid 
  {
  public:

    SE2Grid(const std::vector<std::string>& layers_, const std::vector<bool>& has_so2_);

    SE2Grid() : SE2Grid(std::vector<std::string>(), std::vector<bool>()) {}

    SE2Grid(const SE2Grid&) = default;
    SE2Grid& operator=(const SE2Grid&) = default;
    SE2Grid(SE2Grid&&) = default;
    SE2Grid& operator=(SE2Grid&&) = default;

    virtual ~SE2Grid() = default;

    void initGeometry(const Eigen::Array2d& length_pos_, const double resolution_pos_,
                      const double resolution_yaw_,
                      const Eigen::Vector2d& position_ = Eigen::Vector2d::Zero());

    void convertToDefaultStartIndex();

    float getValue(const std::string& layer, const Eigen::Vector3d& pos,
                   InterpolationMethods interpolationMethod = InterpolationMethods::INTER_LINEAR) const;
      
    float getValueGrad(const std::string& layer, const Eigen::Vector3d& pos, Eigen::Vector3d& grad, 
                      InterpolationMethods interpolationMethod = InterpolationMethods::INTER_LINEAR) const;

    bool move(const Eigen::Vector2d& pos2);

    inline void add(const std::string& layer, const std::vector<Eigen::MatrixXf>& data_)
    {
      assert(size_pos(0) == data_[0].rows());
      assert(size_pos(1) == data_[0].cols());
      assert(size_yaw == (int) data_.size() || data_.size() == 1);

      if (exists(layer))
      {
        data.at(layer) = data_;
        if (data_.size() > 1)
          has_so2.at(layer) = true;
        else
          has_so2.at(layer) = false;
      }
      else
      {
        data.insert(std::pair<std::string, std::vector<Eigen::MatrixXf>>(layer, data_));
        layers.push_back(layer);
        if (data_.size() > 1)
          has_so2.insert(std::pair<std::string, bool>(layer, true));
        else
          has_so2.insert(std::pair<std::string, bool>(layer, false));
      }
    }

    inline void add(const std::string& layer, const bool so2, const double value = NAN)
    {
      std::vector<Eigen::MatrixXf> l;
      if (so2)
      {
        for (int i=0; i<size_yaw; i++)
        {
          l.emplace_back(Eigen::MatrixXf::Constant(size_pos(0), size_pos(1), value));
        }
      }
      else
      {
        l.emplace_back(Eigen::MatrixXf::Constant(size_pos(0), size_pos(1), value));
      }
      add(layer, l);
    }

    inline bool hasSO2(const std::string& layer) const
    {
      if (!(has_so2.find(layer) == has_so2.end()))
        return has_so2.at(layer);
      return false;
    }

    inline bool exists(const std::string& layer) const
    {
      return !(data.find(layer) == data.end());
    }

    inline std::vector<Eigen::MatrixXf>& operator[](const std::string& layer)
    {
      return data.at(layer);
    }

    inline const std::vector<Eigen::MatrixXf>& operator[](const std::string& layer) const
    {
      return data.at(layer);
    }

    inline bool erase(const std::string& layer)
    {
      const auto dataIterator = data.find(layer);
      if (dataIterator == data.end())
      {
        return false;
      }
      data.erase(dataIterator);

      const auto so2Iterator = has_so2.find(layer);
      if (so2Iterator == has_so2.end())
      {
        return false;
      }
      has_so2.erase(so2Iterator);

      const auto layerIterator = std::find(layers.begin(), layers.end(), layer);
      if (layerIterator == layers.end())
      {
        return false;
      }
      layers.erase(layerIterator);

      return true;
    }

    inline const std::vector<std::string>& getLayers() const
    {
      return layers;
    }

    inline const std::unordered_map<std::string, bool>& getHasSO2() const
    {
      return has_so2;
    }

    inline float& value(const std::string& layer, const Eigen::Vector3d& pos)
    {
      Eigen::Array3i index;
      pos2Index(pos, index);
      return at(layer, index);
    }

    inline float& at(const std::string& layer, const Eigen::Array3i& index)
    {
      return data.at(layer)[index(2)](index(0), index(1));
    }

    inline float at(const std::string& layer, const Eigen::Array3i& index) const
    {
      return data.at(layer)[index(2)](index(0), index(1));
    }

    inline void clear(const std::string& layer)
    {
      std::vector<Eigen::MatrixXf> d = data.at(layer);

      for (size_t i=0; i<d.size(); i++)
        d[i].setConstant(NAN);
    }

    inline void clearAll()
    {
      for (auto& d : data)
      {
        for (auto& l : d.second)
          l.setConstant(NAN);
      }
    }

    inline const Eigen::Array2d& getLengthPos() const
    {
      return length_pos;
    }

    inline const double& getLengthYaw() const
    {
      return length_yaw;
    }

    inline const Eigen::Vector2d& getPosition() const
    {
      return position;
    }

    inline double getResolutionPos() const
    {
      return resolution_pos;
    }

    inline double getResolutionYaw() const
    {
      return resolution_yaw;
    }

    inline const Eigen::Array2i& getSizePos() const
    {
      return size_pos;
    }

    inline const int& getSizeYaw() const
    {
      return size_yaw;
    }

    inline const Eigen::Array2i& getStartIndex() const
    {
      return start_index;
    }

    inline void setStartIndex(const Eigen::Array2i& idx)
    {
      start_index = idx;
    }

    inline std::string getFrameId() const
    {
      return frame_id;
    }

    inline void setrameId(const std::string& f)
    {
      frame_id = f;
    }

    inline bool isInMap(const Eigen::Vector3d& pos) const
    {
      Eigen::Vector2d pos_map = origin + position - pos.head(2);
      return pos_map.x() >= 0.0 && pos_map.y() >= 0.0 && \
             pos_map.x() < length_pos(0) && pos_map.y() < length_pos(1);
    }

    inline bool isInMap(const Eigen::Array3i& idx) const
    {
      return idx[0] >= 0 && idx[1] >= 0 && idx[2] >= 0 &&
             idx[0] < size_pos[0] && idx[1] < size_pos[1] && idx[2] < size_yaw;
    }

    inline bool isValid(const Eigen::Array3i& index, const std::string& layer) const
    {
      if (!has_so2.at(layer) && index[2]!=0)
        return false;
      
      return std::isfinite(at(layer, index));
    }

    inline bool isValid(const Eigen::Array3i& index) const
    {
      return isValid(index, layers);
    }

    inline bool isValid(const Eigen::Array3i& index, const std::vector<std::string>& layers) const
    {
      if (layers.empty())
        return false;
      
      return std::all_of(layers.begin(), layers.end(),
                          [&](const std::string& layer){return isValid(index, layer);});
    }

    inline void setPosition(const Eigen::Vector2d& pos2)
    {
      position = pos2;
    }

    inline bool isDefaultStartIndex() const
    {
      return (start_index == 0).all();
    }

    inline static void wrapIndexOne(int& idx, int buffer_size)
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

    inline double wrapYaw(const double& yaw) const
    {
      double new_yaw = yaw;
      while (new_yaw < 0)
        new_yaw += PI_M2;
      while (new_yaw > PI_M2)
        new_yaw -= PI_M2;
      return new_yaw;
    }

    inline void wrapIndex2(Eigen::Array2i& index) const
    {
      wrapIndexOne(index(0), size_pos(0));
      wrapIndexOne(index(1), size_pos(1));
    }

    inline void wrapIndex(Eigen::Array3i& index) const
    {
      wrapIndexOne(index(0), size_pos(0));
      wrapIndexOne(index(1), size_pos(1));
    }

    inline void boundPos(Eigen::Vector3d& pos) const
    {
      wrapYaw(pos(2));
      Eigen::Vector2d positionShifted = pos.head(2) - position + origin;

      for (int i = 0; i < positionShifted.size(); i++)
      {
        double epsilon = 10.0 * std::numeric_limits<double>::epsilon();
        if (std::fabs(pos(i)) > 1.0)
          epsilon *= std::fabs(pos(i));

        if (positionShifted(i) <= 0)
        {
          positionShifted(i) = epsilon;
          continue;
        }
        if (positionShifted(i) >= length_pos(i))
        {
          positionShifted(i) = length_pos(i) - epsilon;
          continue;
        }
      }

      pos.head(2) = positionShifted + position - origin;
    }

    inline void yaw2Index(const double& yaw, int& idx) const
    {
      idx = floor( wrapYaw(yaw) * resolution_yaw_inv );
    }

    inline bool pos2Index(const Eigen::Vector3d& pos, Eigen::Array3i& index) const
    {
      if (!isInMap(pos))
        return false;

      index.head(2) = floor( (origin + position - pos.head(2)).array() * resolution_pos_inv ).cast<int>();
      yaw2Index(pos(2), index(2));

      if ( !isDefaultStartIndex() )
      {
        index.head(2) += start_index;
        wrapIndex(index);
      }

      return true;
    }

    inline bool index2Pos(const Eigen::Array3i& index, Eigen::Vector3d& pos) const
    {
      if (!isInMap(index))
        return false;
      
      Eigen::Vector2d origin_off = (origin.array() - 0.5 * resolution_pos).matrix();

      Eigen::Array3i idx_true = index;
      if (!isDefaultStartIndex())
      {
        idx_true.head(2) -= start_index;
        wrapIndex(idx_true);
      }
      pos.head(2) = position + origin_off - (idx_true.head(2).cast<double>() * resolution_pos).matrix();
      pos(2) = (index(2) + 0.5) * resolution_yaw;

      return true;
    }

    inline bool index2PosWithValue(const std::string& layer, const Eigen::Array3i& index, Eigen::Vector4d& pos) const
    {
      const auto value = at(layer, index);
      if (!std::isfinite(value))
        return false;
      
      Eigen::Vector3d pos3;
      index2Pos(index, pos3);
      pos.head(3) = pos3;
      pos.z() = value;

      return true;
    }

    inline Quadrant getQuadrant(const Eigen::Array2i& index)
    {
      if (index[0] >= start_index[0] && index[1] >= start_index[1])
        return Quadrant::TopLeft;
      if (index[0] >= start_index[0] && index[1] < start_index[1])
        return Quadrant::TopRight;
      if (index[0] < start_index[0] && index[1] >= start_index[1])
        return Quadrant::BottomLeft;
      if (index[0] < start_index[0] && index[1] < start_index[1])
        return Quadrant::BottomRight;

      return Quadrant::TopLeft;
    }

    inline void getMapRange(Eigen::Vector2d& min_range, Eigen::Vector2d& max_range)
    {
      min_range = position - origin;
      max_range = position + origin;
      return;
    }

  private:

    inline void clearCols(unsigned int index, unsigned int nCols)
    {
      for (auto& layer : layers)
      {
        for (auto& l : data.at(layer))
          l.block(0, index, size_pos(0), nCols).setConstant(NAN);
      }
    }

    inline void clearRows(unsigned int index, unsigned int nRows)
    {
      for (auto& layer : layers)
      {
        for (auto& l : data.at(layer))
          l.block(index, 0, nRows, size_pos(1)).setConstant(NAN);
      }
    }

    inline bool getLinearInterpolated(const std::string& layer, const Eigen::Vector3d& pos, float& value) const
    {
      Eigen::Vector3d point;
      Eigen::Array3i point_idx;

      Eigen::Vector3d pos_m = pos;
      pos_m(0) -= 0.5 * resolution_pos;
      pos_m(1) -= 0.5 * resolution_pos;
      pos_m(2) -= 0.5 * resolution_yaw;
      if (!pos2Index(pos_m, point_idx))
      {
        boundPos(pos_m);
        pos2Index(pos_m, point_idx);
      }

      index2Pos(point_idx, point);

      if (hasSO2(layer))
      {
        float values[2][2][2];
        Eigen::Vector3d temp;
        Eigen::Array3i temp_idx;
        for (int i=0; i<2; i++)
          for (int j=0; j<2; j++)
            for (int k=0; k<2; k++)
            {
              temp = point;
              temp(0) += resolution_pos * i;
              temp(1) += resolution_pos * j;
              temp(2) += resolution_yaw * k;
              boundPos(temp);
              pos2Index(temp, temp_idx);
              values[i][j][k] = this->at(layer, temp_idx);
            }

        const Eigen::Vector2d positionRed = (pos - point).head(2) * resolution_pos_inv;
        const Eigen::Vector2d positionRedFlip = Eigen::Vector2d::Ones() - positionRed;
        const double diff_yaw = atan2(sin(pos(2)-point(2)), cos(pos(2)-point(2))) * resolution_yaw_inv;

        float v00 = values[0][0][0] * positionRedFlip.x() + values[1][0][0] * positionRed.x();
        float v01 = values[0][0][1] * positionRedFlip.x() + values[1][0][1] * positionRed.x();
        float v10 = values[0][1][0] * positionRedFlip.x() + values[1][1][0] * positionRed.x();
        float v11 = values[0][1][1] * positionRedFlip.x() + values[1][1][1] * positionRed.x();
        float v0 = v00 * positionRedFlip.y() + v10 * positionRed.y();
        float v1 = v01 * positionRedFlip.y() + v11 * positionRed.y();

        value = v0 * (1.0 - diff_yaw) + v1 * diff_yaw;
      }
      else
      {
        float values[2][2];
        Eigen::Vector3d temp;
        Eigen::Array3i temp_idx;
        for (int i=0; i<2; i++)
          for (int j=0; j<2; j++)
          {
            temp = point;
            temp(0) += resolution_pos * i;
            temp(1) += resolution_pos * j;
            temp(2) = 0.0;
            boundPos(temp);
            pos2Index(temp, temp_idx);
            temp_idx[2] = 0;
            values[i][j] = this->at(layer, temp_idx);
          }

        const Eigen::Vector2d positionRed = (pos - point).head(2) * resolution_pos_inv;
        const Eigen::Vector2d positionRedFlip = Eigen::Vector2d::Ones() - positionRed;

        float v0 = values[0][0] * positionRedFlip.x() + values[1][0] * positionRed.x();
        float v1 = values[0][1] * positionRedFlip.x() + values[1][1] * positionRed.x();
        value = v0 * positionRedFlip.y() + v1 * positionRed.y();
      }

      return true;
    }

    inline bool getLinearInterpolatedGrad(const std::string& layer, const Eigen::Vector3d& pos,
                                          float& value, Eigen::Vector3d& grad) const
    {
      grad.setZero();

      Eigen::Vector3d point;
      Eigen::Array3i point_idx;

      Eigen::Vector3d pos_m = pos;
      pos_m(0) -= 0.5 * resolution_pos;
      pos_m(1) -= 0.5 * resolution_pos;
      pos_m(2) -= 0.5 * resolution_yaw;
      if (!pos2Index(pos_m, point_idx))
      {
        boundPos(pos_m);
        pos2Index(pos_m, point_idx);
      }

      index2Pos(point_idx, point);

      if (hasSO2(layer))
      {
        float values[2][2][2];
        Eigen::Vector3d temp;
        Eigen::Array3i temp_idx;
        for (int i=0; i<2; i++)
          for (int j=0; j<2; j++)
            for (int k=0; k<2; k++)
            {
              temp = point;
              temp(0) += resolution_pos * i;
              temp(1) += resolution_pos * j;
              temp(2) += resolution_yaw * k;
              boundPos(temp);
              pos2Index(temp, temp_idx);
              values[i][j][k] = this->at(layer, temp_idx);
            }

        const Eigen::Vector2d positionRed = (pos - point).head(2) * resolution_pos_inv;
        const Eigen::Vector2d positionRedFlip = Eigen::Vector2d::Ones() - positionRed;
        const float diff_yaw = atan2(sin(pos(2)-point(2)), cos(pos(2)-point(2))) * resolution_yaw_inv;

        float v00 = values[0][0][0] * positionRedFlip.x() + values[1][0][0] * positionRed.x();
        float v01 = values[0][0][1] * positionRedFlip.x() + values[1][0][1] * positionRed.x();
        float v10 = values[0][1][0] * positionRedFlip.x() + values[1][1][0] * positionRed.x();
        float v11 = values[0][1][1] * positionRedFlip.x() + values[1][1][1] * positionRed.x();
        float v0 = v00 * positionRedFlip.y() + v10 * positionRed.y();
        float v1 = v01 * positionRedFlip.y() + v11 * positionRed.y();

        value = v0 * (1.0 - diff_yaw) + v1 * diff_yaw;

        grad(2) = (v1 - v0) * resolution_yaw_inv;
        grad(1) = ((v10 - v00) * (1.0 - diff_yaw) + (v11 - v01) * diff_yaw) * resolution_pos_inv;
        grad(0) = (1.0 - diff_yaw) * positionRedFlip.y() * (values[1][0][0] - values[0][0][0]);
        grad(0) += (1.0 - diff_yaw) * positionRed.y() * (values[1][1][0] - values[0][1][0]);
        grad(0) += diff_yaw * positionRedFlip.y() * (values[1][0][1] - values[0][0][1]);
        grad(0) += diff_yaw * positionRed.y() * (values[1][1][1] - values[0][1][1]);
        grad(0) *= resolution_pos_inv;
      }
      else
      {
        float values[2][2];
        Eigen::Vector3d temp;
        Eigen::Array3i temp_idx;
        for (int i=0; i<2; i++)
          for (int j=0; j<2; j++)
          {
            temp = point;
            temp(0) += resolution_pos * i;
            temp(1) += resolution_pos * j;
            temp(2) = 0.0;
            boundPos(temp);
            pos2Index(temp, temp_idx);
            temp_idx[2] = 0;
            values[i][j] = this->at(layer, temp_idx);
          }

        const Eigen::Vector2d positionRed = (pos - point).head(2) * resolution_pos_inv;
        const Eigen::Vector2d positionRedFlip = Eigen::Vector2d::Ones() - positionRed;

        float v0 = values[0][0] * positionRedFlip.x() + values[1][0] * positionRed.x();
        float v1 = values[0][1] * positionRedFlip.x() + values[1][1] * positionRed.x();
        value = v0 * positionRedFlip.y() + v1 * positionRed.y();

        grad(2) = 0.0;
        grad(1) = (v1 - v0) * resolution_pos_inv;
        grad(0) = positionRedFlip.y() * (values[1][1] - values[0][1]);
        grad(0) += positionRed.y() * (values[1][0] - values[0][0]);
        grad(0) *= resolution_pos_inv;
      }

      return true;        
    }

    inline bool getCubicConvolutionInterpolated(const std::string& layer, const Eigen::Vector3d& pos, float& value) const
    {
      // TODO
      std::cout << "Not implemented yet: "<<layer <<" "<<pos.transpose()<<" "<<value<< std::endl;
      double interpolatedValue = 0.0;
      if (!std::isfinite(interpolatedValue))
      {
        return false;
      }
      return false;
    }

    inline bool getCubicInterpolated(const std::string& layer, const Eigen::Vector3d& pos, float& value) const
    {
      // TODO
      std::cout << "Not implemented yet: "<<layer <<" "<<pos.transpose()<<" "<<value<< std::endl;
      double interpolatedValue = 0.0;
      if (!std::isfinite(interpolatedValue))
      {
        return false;
      }
      return false;
    }

    std::unordered_map<std::string, std::vector<Eigen::MatrixXf>> data;

    std::unordered_map<std::string, bool> has_so2;

    std::vector<std::string> layers;

    Eigen::Array2d length_pos;

    double length_yaw;

    double resolution_pos;

    double resolution_pos_inv;

    double resolution_yaw;
    
    double resolution_yaw_inv;

    Eigen::Vector2d position;

    Eigen::Vector2d origin;

    Eigen::Array2i size_pos;

    int size_yaw;

    Eigen::Array2i start_index;

    std::string frame_id = std::string("map");

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
