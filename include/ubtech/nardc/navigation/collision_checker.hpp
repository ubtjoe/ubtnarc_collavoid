#pragma once
//! # defines
#define BOOST_GEOMETRY_OVERLAY_NO_THROW
//! c/c++ system headers
#include <limits>
#include <string>
#include <vector>
//! other headers
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <eigen3/Eigen/Dense>  // NOLINT [build/include_order]
#include <nlohmann/json.hpp>

namespace bg = boost::geometry;

//! aliases
using json = nlohmann::json;

namespace ubtech {
namespace nardc {
namespace navigation {

using point_t = bg::model::point<double, 2, bg::cs::cartesian>;
using polygon_t = bg::model::polygon<point_t>;
using points_t = std::vector<point_t>;

class CollisionChecker {
 public:
     /**
      * CollisionChecker()
      *
      * @brief construct empty CollisionChecker
      *
      * @note default checker radius is 5m
      */
     CollisionChecker() : checker_radius_(5) { }

     /**
      * CollisionChecker(double const &)
      *
      * @brief construct empty CollisionChecker with override for checker radius
      */
     explicit CollisionChecker(double const & radius) : checker_radius_(radius) { }

     /**
      * CollisionChecker(points_t const &, double const & (default 5))
      *
      * @brief construct CollisionChecker with points array
      *
      * @note default checker radius is 5m
      */
     explicit CollisionChecker(points_t const & points, double const & radius = 5)
         : points_(points), checker_radius_(radius) { }

     /**
      * CollisionChecker(string const &)
      *
      * @brief construct CollisionChecker objects from points passed in json-ic string representation
      *
      * @param[in] string const&, data encoding points, like from ranging sensors
      *
      * @note A two point example would be example could be: 
      *     string m =
      *          "[{\n"
      *          "    \"name\": "sample points",
      *          "    \"points\": [[0, 0], [1, 0]]\n"
      *          "}]\n";
      *
      */
     explicit CollisionChecker(std::string const & points_json,
             double const & radius = 5) : checker_radius_(radius) {
         json j = json::parse(points_json);
         for (auto const it : j) {
             std::string const name = it["name"];
             for (auto const jt : it["points"]) {
                 auto const x = static_cast<double>(jt[0]);
                 auto const y = static_cast<double>(jt[1]);
                 points_.push_back(point_t(x, y));
             }
         }
     }

     /**
      * set_points(points_t const &)
      *
      * @brief set points to the map
      *
      * @param[in] points set of (x, y) points to put into collision checker
      * @returns void
      */
     void set_points(points_t const & points) { points_ = points; }

     /**
      * add_points(points_t const &)
      *
      * @brief add points to the map
      *
      * @param[in] points set of (x, y) points to add to collision checker
      * @returns void
      */
     void add_points(points_t const & points) {
       for (auto const & p : points) {
         points_.emplace_back(p);
       }
     }

     /**
      * check_intersection(polygon_t const&, Eigen::Matrix3d const&) const
      *
      * @brief check whether or not points lie within a test polygon
      * 
      * @param[in] polygon test polygon
      * @param[in] transformation (optional, default = Identity) homogeneous
      * transformation to apply to polygon before checking collision
      * @returns true if intersection is found, false otherwise
      */
     bool check_intersection(polygon_t const & polygon,
             Eigen::Matrix3d const & transformation = Eigen::Matrix3d::Identity()) const {
         //! setup lambda function to transform the polygon
         auto const transformedPolygon = [&]() {
             if( transformation != Eigen::Matrix3d::Identity() ) {
                 //! Adjust transformation to be centered around polygon centroid
                 //! - initialize with (irrelevant) values to suppress compiler warning
                 const auto & init = std::numeric_limits<double>::signaling_NaN();
                 point_t centroid(init, init);
                 bg::centroid(polygon, centroid);
                 const double c_x = bg::get<0>(centroid);
                 const double c_y = bg::get<1>(centroid);

                 Eigen::Matrix3d pre_centroid = Eigen::Matrix3d::Identity();
                 pre_centroid.block<2, 1>(0, 2) = Eigen::Vector2d(c_x, c_y);

                 Eigen::Matrix3d post_centroid = Eigen::Matrix3d::Identity();
                 post_centroid.block<2, 1>(0, 2) = -Eigen::Vector2d(c_x, c_y);

                 const Eigen::Matrix3d centered_transformation = pre_centroid * transformation * post_centroid;

                 bg::strategy::transform::ublas_transformer<double, 2, 2> transformer(
                   centered_transformation(0, 0), centered_transformation(0, 1), centered_transformation(0, 2),
                   centered_transformation(1, 0), centered_transformation(1, 1), centered_transformation(1, 2),
                   centered_transformation(2, 0), centered_transformation(2, 1), centered_transformation(2, 2));

                 polygon_t transformed_polygon;
                 bg::transform(polygon, transformed_polygon, transformer);  // NOLINT [build/include_what_you_use]
                 return transformed_polygon;
             } else {
                 return polygon;
             }
         };
         //! using const ref avoids a copy when we do no transform
         auto const & query_polygon = transformedPolygon();

         //! check point intersections (return true if point is inside of the query polygon)
         point_t qp_centroid;
         bg::centroid(query_polygon, qp_centroid);
         std::cout << "CCHECKER: " << points_.size() << std::endl;
         size_t ctr = 0;
         for (auto const & p : points_) {
             //std::cout << "checking pt: " << ctr++ << std::endl;
             /*std::cout << "Point" << std::endl;
                 std::cout << bg::get<0>(p) << std::endl;
                 std::cout << bg::get<1>(p) << std::endl;*/
             if (bg::distance(p, qp_centroid) <= checker_radius_
                     && bg::within(p, query_polygon)) {
             /*std::cout << "Intersection Point" << std::endl;
                 std::cout << bg::get<0>(p) << std::endl;
                 std::cout << bg::get<1>(p) << std::endl;
                 auto const & poly2 = query_polygon;
std::cout << "poly2's coordinates:" << std::endl;
    for (unsigned i = 0; i != poly2.outer().size(); ++i) {
        double x = boost::geometry::get<0>(poly2.outer()[i]);
        double y = boost::geometry::get<1>(poly2.outer()[i]);
        std::cout << " " << x << ", " << y << std::endl;
    }*/
                 return true;
             }
         }
         return false;
     }

 private:
     points_t points_;
     double checker_radius_;
};
}  // end namespace navigation
}  // end namespace nardc
}  // end namespace ubtech
