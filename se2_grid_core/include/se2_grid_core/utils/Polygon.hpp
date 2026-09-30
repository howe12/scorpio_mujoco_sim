#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <algorithm>

namespace se2_grid {

class Polygon
{
 public:

  enum class TriangulationMethods 
  {
    FAN // Fan triangulation (only for convex polygons).
  };

  Polygon() {}

  Polygon(std::vector<Eigen::Vector2d> _vertices) : vertices(_vertices) {}

  inline bool isInside(const Eigen::Vector2d& point) const
  {
    int cross = 0;
    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
    {
      if ( ((vertices[i].y() > point.y()) != (vertices[j].y() > point.y())) && 
            (point.x() < (vertices[j].x() - vertices[i].x()) * (point.y() - vertices[i].y()) /
            (vertices[j].y() - vertices[i].y()) + vertices[i].x()) )
      {
        cross++;
      }
    }
    return bool(cross % 2);
  }

  inline void addVertex(const Eigen::Vector2d& vertex)
  {
    vertices.push_back(vertex);
  }

  inline const Eigen::Vector2d& getVertex(const size_t index) const
  {
    return vertices.at(index);
  }

  inline void removeVertices()
  {
    vertices.clear();
  }

  inline const Eigen::Vector2d& operator [](size_t index) const
  {
    return getVertex(index);
  }

  inline const std::vector<Eigen::Vector2d>& getVertices() const
  {
    return vertices;
  }

  inline size_t nVertices() const
  {
    return vertices.size();
  }

  inline double getArea() const
  {
    double area = 0.0;
    int j = vertices.size() - 1;
    for (size_t i = 0; i < vertices.size(); i++)
    {
      area += (vertices.at(j).x() + vertices.at(i).x())
            * (vertices.at(j).y() - vertices.at(i).y());
      j = i;
    }
    return std::abs(area / 2.0);
  }

  inline Eigen::Vector2d getCentroid() const
  {
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> vertices = getVertices();
    vertices.push_back(vertices.at(0));
    double area = 0.0;
    for (size_t i = 0; i < vertices.size() - 1; i++)
    {
      const double a = vertices[i].x() * vertices[i+1].y() - vertices[i+1].x() * vertices[i].y();
      area += a;
      centroid.x() += a * (vertices[i].x() + vertices[i+1].x());
      centroid.y() += a * (vertices[i].y() + vertices[i+1].y());
    }
    area *= 0.5;
    centroid /= (6.0 * area);
    return centroid;
  }

  inline void getBoundingBox(Eigen::Vector2d& center, Eigen::Array2d& length) const
  {
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const auto& vertex : vertices) 
    {
      if (vertex.x() > maxX) maxX = vertex.x();
      if (vertex.y() > maxY) maxY = vertex.y();
      if (vertex.x() < minX) minX = vertex.x();
      if (vertex.y() < minY) minY = vertex.y();
    }
    center.x() = (minX + maxX) / 2.0;
    center.y() = (minY + maxY) / 2.0;
    length.x() = (maxX - minX);
    length.y() = (maxY - minY);
  }

  inline bool convertToInequalityConstraints(Eigen::MatrixXd& A,
                                             Eigen::VectorXd& b) const
  {
    Eigen::MatrixXd V(nVertices(), 2);
    for (unsigned int i = 0; i < nVertices(); ++i)
      V.row(i) = vertices[i];

    // Create k, a list of indices from V forming the convex hull.
    // TODO: Assuming counter-clockwise ordered convex polygon.
    // MATLAB: k = convhulln(V);
    Eigen::MatrixXi k;
    k.resizeLike(V);
    for (unsigned int i = 0; i < V.rows(); ++i)
      k.row(i) << i, (i+1) % V.rows();
    Eigen::RowVectorXd c = V.colwise().mean();
    V.rowwise() -= c;
    A = Eigen::MatrixXd::Constant(k.rows(), V.cols(), NAN);

    unsigned int rc = 0;
    for (unsigned int ix = 0; ix < k.rows(); ++ix)
    {
      Eigen::MatrixXd F(2, V.cols());
      F.row(0) << V.row(k(ix, 0));
      F.row(1) << V.row(k(ix, 1));
      Eigen::FullPivLU<Eigen::MatrixXd> luDecomp(F);
      if (luDecomp.rank() == F.rows())
      {
        A.row(rc) = F.colPivHouseholderQr().solve(Eigen::VectorXd::Ones(F.rows()));
        ++rc;
      }
    }

    A = A.topRows(rc);
    b = Eigen::VectorXd::Ones(A.rows());
    b = b + A * c.transpose();

    return true;
  }

  inline bool offsetInward(double margin)
  {
    // Create a list of indices of the neighbours of each vertex.
    // TODO: Assuming counter-clockwise ordered convex polygon.
    std::vector<Eigen::Array2i> neighbourIndices;
    const unsigned int n = nVertices();
    neighbourIndices.resize(n);
    for (unsigned int i = 0; i < n; ++i)
    {
      neighbourIndices[i] << (i > 0 ? (i-1)%n : n-1), (i + 1) % n;
    }

    std::vector<Eigen::Vector2d> copy(vertices);
    for (unsigned int i = 0; i < neighbourIndices.size(); ++i)
    {
      Eigen::Vector2d v1 = vertices[neighbourIndices[i](0)] - vertices[i];
      Eigen::Vector2d v2 = vertices[neighbourIndices[i](1)] - vertices[i];
      v1.normalize();
      v2.normalize();
      const double angle = acos(v1.dot(v2));
      copy[i] += margin / sin(angle) * (v1 + v2);
    }
    vertices = copy;
    return true;
  }

  inline bool thickenLine(double thickness)
  {
    if (vertices.size() != 2) return false;
    const Eigen::Vector2d connection(vertices[1] - vertices[0]);
    const Eigen::Vector2d orthogonal = thickness * Eigen::Vector2d(connection.y(), -connection.x()).normalized();
    std::vector<Eigen::Vector2d> newVertices;
    newVertices.reserve(4);
    newVertices.push_back(vertices[0] + orthogonal);
    newVertices.push_back(vertices[0] - orthogonal);
    newVertices.push_back(vertices[1] - orthogonal);
    newVertices.push_back(vertices[1] + orthogonal);
    vertices = newVertices;
    return true;
  }

  inline std::vector<Polygon> triangulate(const TriangulationMethods& method = TriangulationMethods::FAN) const
  {
    // TODO Add more triangulation methods.
    // https://en.wikipedia.org/wiki/Polygon_triangulation
    std::vector<Polygon> polygons;
    if (vertices.size() < 3)
      return polygons;

    size_t nPolygons = vertices.size() - 2;
    polygons.reserve(nPolygons);

    if (nPolygons < 1)
    {
      // Special case.
      polygons.push_back(*this);
    }else
    {
      // General case.
      for (size_t i = 0; i < nPolygons; ++i)
      {
        Polygon polygon({vertices[0], vertices[i + 1], vertices[i + 2]});
        polygons.push_back((polygon));
      }
    }

    return polygons;
  }

  inline static Polygon fromCircle(Eigen::Vector2d center, double radius,
                                   int nVertices = 20)
  {
    Eigen::Vector2d centerToVertex(radius, 0.0), centerToVertexTemp;

    Polygon polygon;
    for (int j = 0; j < nVertices; j++)
    {
      double theta = j * 2 * M_PI / (nVertices - 1);
      Eigen::Rotation2D<double> rot2d(theta);
      centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
      polygon.addVertex(center + centerToVertexTemp);
    }
    return polygon;
  }

  inline static Polygon convexHullOfTwoCircles(Eigen::Vector2d center1,
                                               Eigen::Vector2d center2,
                                               double radius,
                                               int nVertices = 20)
  {
    if (center1 == center2) return fromCircle(center1, radius, nVertices);
    Eigen::Vector2d centerToVertex, centerToVertexTemp;
    centerToVertex = center2 - center1;
    centerToVertex.normalize();
    centerToVertex *= radius;

    se2_grid::Polygon polygon;
    for (int j = 0; j < ceil(nVertices / 2.0); j++)
    {
      double theta = M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
      Eigen::Rotation2D<double> rot2d(theta);
      centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
      polygon.addVertex(center1 + centerToVertexTemp);
    }
    for (int j = 0; j < ceil(nVertices / 2.0); j++)
    {
      double theta = 3 * M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
      Eigen::Rotation2D<double> rot2d(theta);
      centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
      polygon.addVertex(center2 + centerToVertexTemp);
    }
    return polygon;
  }

  inline static Polygon convexHull(Polygon& polygon1, Polygon& polygon2)
  {
    std::vector<Eigen::Vector2d> vertices_;
    vertices_.reserve(polygon1.nVertices() + polygon2.nVertices());
    vertices_.insert(vertices_.end(), polygon1.getVertices().begin(), polygon1.getVertices().end());
    vertices_.insert(vertices_.end(), polygon2.getVertices().begin(), polygon2.getVertices().end());

    return monotoneChainConvexHullOfPoints(vertices_);
  }

  inline static Polygon monotoneChainConvexHullOfPoints(const std::vector<Eigen::Vector2d>& points)
  {
    // Adapted from https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
    if (points.size() <= 3)
    {
      return Polygon(points);
    }
    std::vector<Eigen::Vector2d> pointsConvexHull(2 * points.size());

    // Sort points lexicographically.
    auto sortedPoints(points);
    std::sort(sortedPoints.begin(), sortedPoints.end(), sortVertices);

    int k = 0;
    // Build lower hull
    for (size_t i = 0; i < sortedPoints.size(); ++i)
    {
      while (k >= 2 && vectorsMakeClockwiseTurn(pointsConvexHull.at(k - 2), pointsConvexHull.at(k - 1), sortedPoints.at(i)))
      {
        k--;
      }
      pointsConvexHull.at(k++) = sortedPoints.at(i);
    }

    // Build upper hull.
    for (int i = sortedPoints.size() - 2, t = k + 1; i >= 0; i--)
    {
      while (k >= t && vectorsMakeClockwiseTurn(pointsConvexHull.at(k - 2), pointsConvexHull.at(k - 1), sortedPoints.at(i)))
      {
        k--;
      }
      pointsConvexHull.at(k++) = sortedPoints.at(i);
    }
    pointsConvexHull.resize(k - 1);

    Polygon polygon(pointsConvexHull);
    return polygon;
  }

 protected:

  static bool sortVertices(const Eigen::Vector2d& vector1,
                           const Eigen::Vector2d& vector2);

  static double computeCrossProduct2D(const Eigen::Vector2d& vector1,
                                      const Eigen::Vector2d& vector2);

  static double vectorsMakeClockwiseTurn(const Eigen::Vector2d& pointO,
                                         const Eigen::Vector2d& pointA,
                                         const Eigen::Vector2d& pointB);

  std::vector<Eigen::Vector2d> vertices;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
