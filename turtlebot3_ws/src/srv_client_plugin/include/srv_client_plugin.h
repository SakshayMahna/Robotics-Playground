#ifndef SRV_CLIENT_PLUGIN_H_
#define SRV_CLIENT_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "pp_msgs/PathPlanningPlugin.h"


namespace srv_client_plugin{
  /**
   * @class SrvClientPlugin
   * @brief A global planner that creates service request for a plan and forwards the response to the move_base global planner module
   */
  class SrvClientPlugin : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the SrvClientPlugin
       */
      SrvClientPlugin();
      /**
       * @brief  Constructor for the SrvClientPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the SrvClientPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
      * @brief Converts a x,y grid cell coordinate value to a linear index value (one-dimensional array index)
      * @param x Grid cell map x coordinate value
      * @param y Grid cell map y coordinate value
      * @return index value corresponding to the location on the one-dimensional array representation
      */
      size_t ToIndex(float x, float y);

      /**
      * @brief Converts a linear index value to a x,y grid cell coordinate value
      * @param index A linear index value, specifying a cell/pixel in an 1-D array
      * @param x Grid cell map x coordinate value
      * @param y Grid cell map y coordinate value
      */
      void FromIndex(size_t index, int &x, int &y);

      /**
      * @brief Converts x,y values in the world frame (in meters) into x,y grid map coordinates
      *        This transformation is derived from the map resolution and adjusts 
      *        w.r.t the location of the map origin
      * @param x X-Axis value in the world frame of reference (in meters)
      * @param y Y-Axis value in the world frame of reference (in meters)
      */
      void FromWorldToGrid(float &x, float &y);

      /**
      * @brief Converts x,y grid cell coordinates to world coordinates (in meters)
      *        This transformation is derived from the map resolution, adjusts
      *        w.r.t the location of the map origin and can include an offset
      *        to place the world coordinate at the center point of a grid cell
      * @param x Grid cell map x coordinate value
      * @param y Grid cell map y coordinate value
      */
      void FromGridToWorld(float &x, float &y); 

      /**
      * @brief Checks if world coordinates are inside grid map bounds
      * @param x X-Axis value in the world frame of reference (in meters)
      * @param y Y-Axis value in the world frame of reference (in meters)
      * @return true if a index is in map bounds, otherwise false
      */
      bool InGridMapBounds(float &x, float &y);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      bool initialized_;
      // x,y position (in meters) of grid map origin w.r.t world's coordinate origin
      float origin_x_;
      float origin_y_;
      // the resolution of the map which is expressed in meters per pixel
      // or the size of each grid cell (pixel) in meters
      // 0.05 means for example 5 centimers for each cell (pixel)
      float resolution_;
      // by default path is created along the corners/edges of grid cells
      bool path_at_node_center = false;
      float node_center_offset_ = 0;
      // map dimentions in number of grid cells
      int width_;
      int height_;
      int map_size_;
      // service client declaration
      ros::ServiceClient makeplan_service_;
       /**
       * @brief Publishes the global plan to display it's entire lenght in RVIZ
       * @param path The plan as filled by the planner
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
      ros::Publisher plan_pub_;
  };
}  
#endif