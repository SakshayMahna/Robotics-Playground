#include <pluginlib/class_list_macros.h>
#include <srv_client_plugin.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(srv_client_plugin::SrvClientPlugin, nav_core::BaseGlobalPlanner)

namespace srv_client_plugin
{

  SrvClientPlugin::SrvClientPlugin()
  {
    initialized_ = false;
  }

  SrvClientPlugin::SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialized_ = false;
    initialize(name, costmap_ros);
  }

  void SrvClientPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized_)
    {
      ros::NodeHandle private_nh("~/" + name);
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      origin_x_ = costmap_->getOriginX();
      origin_y_ = costmap_->getOriginY();

      width_ = costmap_->getSizeInCellsX();
      height_ = costmap_->getSizeInCellsY();
      resolution_ = costmap_->getResolution();
      map_size_ = width_ * height_;

      // create a client for the path planning service
      makeplan_service_ = private_nh.serviceClient<pp_msgs::PathPlanningPlugin>("make_plan");
      // wait for the service to be advertised and available, blocks until it is.
      makeplan_service_.waitForExistence();
      // create publisher to display the complete trajectory path in RVIZ
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      // place path waypoints at the center of each grid cell (vs. at the corners of grid cells)
      path_at_node_center = true;
      if (path_at_node_center)
      {
        // shift all of the coordinates by half a grid cell
        node_center_offset_ = resolution_ / 2;
      }

      initialized_ = true;
    }
  }

    // fill plan request, call plan service, process plan response
    bool SrvClientPlugin::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
      plan.clear();
      // clear previously published path in Rviz
      publishPlan(plan);

      // Fill costmap (costmap is a 1-D array map representation)
      std::vector<int> costmap(map_size_);

      for (size_t idx = 0; idx < map_size_; ++idx)
      {
        int x, y;
        x = idx % width_;
        y = std::floor(idx / width_);
        costmap.at(idx) = static_cast<int>(costmap_->getCost(x, y));
      }

      float start_x = start.pose.position.x;
      float start_y = start.pose.position.y;
      float goal_x = goal.pose.position.x;
      float goal_y = goal.pose.position.y;

      size_t start_index = 0;
      size_t goal_index = 0;

      // check if start/goal world coordinates are inside  grid map bounds
      if (InGridMapBounds(start_x, start_y) && InGridMapBounds(goal_x, goal_y))
      {
        // convert x,y in world coordinates/meters) to x,y in grid map cell coordinates
        FromWorldToGrid(start_x, start_y);
        FromWorldToGrid(goal_x, goal_y);

        // convert 2d representation into flat array index representation
        start_index = ToIndex(start_x, start_y);
        goal_index = ToIndex(goal_x, goal_y);
      }
      else
      {
        ROS_WARN("Start or goal position outside of the map's boundaries");
        return false;
      }

      // To-Do: check that a start and goal are not obstacles

      pp_msgs::PathPlanningPlugin makeplan;
      makeplan.request.costmap_ros = costmap;
      makeplan.request.start = start_index;
      makeplan.request.goal = goal_index;
      makeplan.request.width = width_;
      makeplan.request.height = height_;

      // call path planning service
      makeplan_service_.call(makeplan);

      std::vector<int> index_plan = makeplan.response.plan;

      ROS_DEBUG("Number of points: %d", unsigned(index_plan.size()));

      /* Process plan response */
      if (index_plan.size())
      {
        // insert start node into plan response
        index_plan.insert(index_plan.begin(), start_index);
        // insert goal node into plan response
        index_plan.push_back(goal_index);

        for (int p : index_plan)
        {
          int x, y;
          FromIndex(p, x, y);
          float x_path = static_cast<float>(x);
          float y_path = static_cast<float>(y);

          FromGridToWorld(x_path, y_path);
          geometry_msgs::PoseStamped position;
          position.header.frame_id = start.header.frame_id;
          position.pose.position.x = x_path;
          position.pose.position.y = y_path;
          position.pose.orientation.x = 0;
          position.pose.orientation.y = 0;
          position.pose.orientation.z = 0;
          position.pose.orientation.w = 1;

          plan.push_back(position);
        }

        plan.push_back(goal);

        // Publish the path for visualisation
        publishPlan(plan);

        return true;
      }
    else
      {
        // no plan found
        return false;
      }
    }

    void SrvClientPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {

      // create a message
      nav_msgs::Path gui_path;
      gui_path.poses.resize(path.size());

      gui_path.header.frame_id = "map";
      gui_path.header.stamp = ros::Time::now();

      // Extract the plan in world coordinates
      for (unsigned int i = 0; i < path.size(); i++)
      {
        gui_path.poses[i] = path[i];
      }
      plan_pub_.publish(gui_path);
    }

    size_t SrvClientPlugin::ToIndex(float x, float y)
    {
      return y * width_ + x;
    }

    void SrvClientPlugin::FromIndex(size_t index, int &x, int &y)
    {
      x = index % width_;
      y = std::floor(index / width_);
    }

    void SrvClientPlugin::FromWorldToGrid(float &x, float &y)
    {
      x = static_cast<size_t>((x - origin_x_) / resolution_);
      y = static_cast<size_t>((y - origin_y_) / resolution_);
    }

    void SrvClientPlugin::FromGridToWorld(float &x, float &y)
    {
      x = x * resolution_ + origin_x_ + node_center_offset_;
      y = y * resolution_ + origin_y_ + node_center_offset_;
    }

    bool SrvClientPlugin::InGridMapBounds(float &x, float &y)
    {
      if (x < origin_x_ || y < origin_y_ || x > origin_x_ + (width_ * resolution_) || y > origin_y_ + (height_ * resolution_))
        return false;
      return true;
    }

  }; // namespace srv_client_plugin