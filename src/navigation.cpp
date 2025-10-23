// MIT License (modified)

// Copyright (c) 2020 The Trustees of the University of Pennsylvania
// Authors:
// Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this **file** (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <reactive_planner_lib.h>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>
#include <object_pose_interface_msgs/msg/semantic_map_object_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <example_interfaces/msg/u_int32.hpp>
#include <foxglove_msgs/msg/geo_json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sstream>
#include <exception>

#define BEHAVIOR_SIT 0
#define BEHAVIOR_STAND 1
#define BEHAVIOR_WALK 2

#define MODE_STAND 0
#define MODE_START 1

// Define properties for dilations
const int points_per_circle = 5;
bg::strategy::buffer::join_miter join_strategy_input;
bg::strategy::buffer::end_flat end_strategy_input;
bg::strategy::buffer::point_circle point_strategy_input;
bg::strategy::buffer::side_straight side_strategy_input;

class NavigationNode : public rclcpp::Node {
	public:
		// Constructor
        NavigationNode() : rclcpp::Node(std::string("navigation_node")) {
        	RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Navigation Node Initialized");

			// Find parameters
			this->declare_parameter("pub_twist_topic", "/cmd_vel");
			this->declare_parameter("pub_twist_stamped_topic", "/cmd_vel_stamped");
		
			this->declare_parameter("pub_behaviorID_topic", "/behavior_id");
			this->declare_parameter("pub_behaviorMode_topic", "/behavior_mode");
			this->declare_parameter("pub_marker_topic", "/marker");

			//this->declare_parameter("sub_laser_topic", "/laser_scan");
			this->declare_parameter("sub_laser_topic", "/fake_lidar_scan");
			this->declare_parameter("sub_robot_topic", "/robot_pose");
			this->declare_parameter("sub_semantic_topic", "/pose_tracking/semantic_map");

			this->declare_parameter("world_frame_id", "world");
			this->declare_parameter("odom_frame_id", "odom");
			this->declare_parameter("laser_frame_id", "laser_frame");

			this->declare_parameter("target_object", "");
			this->declare_parameter("target_object_length", 0.0);
			this->declare_parameter("target_object_width", 0.0);

			this->declare_parameter("RobotRadius", 0.0);
			this->declare_parameter("ObstacleDilation", 0.0);
			this->declare_parameter("WalkHeight", 0.0);

			this->declare_parameter("AllowableRange", 0.0);
			this->declare_parameter("CutoffRange", 0.0);

			this->declare_parameter("ForwardLinCmdLimit", 0.0);
			this->declare_parameter("BackwardLinCmdLimit", 0.0);
			this->declare_parameter("AngCmdLimit", 0.0);
			this->declare_parameter("RFunctionExponent", 0.0);
			this->declare_parameter("Epsilon", 0.0);
			this->declare_parameter("VarEpsilon", 0.0);
			this->declare_parameter("Mu1", 0.0);
			this->declare_parameter("Mu2", 0.0);
			this->declare_parameter("WorkspaceMinX", -100.0);
			this->declare_parameter("WorkspaceMinY", -100.0);
			this->declare_parameter("WorkspaceMaxX", 100.0);
			this->declare_parameter("WorkspaceMaxY", 100.0);
			this->declare_parameter("SemanticMapUpdateRate", 0.0);

			this->declare_parameter("LinearGain", 0.0);
			this->declare_parameter("AngularGain", 0.0);

			this->declare_parameter("Goal_x", 0.0);
			this->declare_parameter("Goal_y", 0.0);
			
			this->declare_parameter("Tolerance", 0.0);
			this->declare_parameter("LowpassCutOff", 0.0);
			this->declare_parameter("LowpassSampling", 0.0);
			this->declare_parameter("LowpassOrder", 0.0);
			this->declare_parameter("LowpassSamples", 0.0);

			this->declare_parameter("DebugFlag", false);
			this->declare_parameter("SimulationFlag", true);

			this->set_parameter(rclcpp::Parameter("use_sim_time", true));

			// Confirm value of use_sim_time
			bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
			RCLCPP_INFO(this->get_logger(), "use_sim_time is: %s", use_sim_time ? "true" : "false");

			pub_twist_topic_ = this->get_parameter("pub_twist_topic").as_string();
			pub_twist_stamped_topic_ = this->get_parameter("pub_twist_stamped_topic").as_string();

			pub_behaviorID_topic_ = this->get_parameter("pub_behaviorID_topic").as_string();
			pub_behaviorMode_topic_ = this->get_parameter("pub_behaviorMode_topic").as_string();
			pub_marker_topic_ = this->get_parameter("pub_marker_topic").as_string();
			
			sub_laser_topic_ = this->get_parameter("sub_laser_topic").as_string();
			sub_laser_topic_ = this->get_parameter("sub_laser_topic").as_string();
			sub_robot_topic_ = this->get_parameter("sub_robot_topic").as_string();
			sub_semantic_topic_ = this->get_parameter("sub_semantic_topic").as_string();

			world_frame_id_ = this->get_parameter("world_frame_id").as_string();
			odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
			laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();
			laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();

			target_object_ = this->get_parameter("target_object").as_string();
			target_object_length_ = this->get_parameter("target_object_length").as_double();
			target_object_width_ = this->get_parameter("target_object_width").as_double();

			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), sub_laser_topic_);
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), sub_robot_topic_);
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), sub_semantic_topic_);

			RobotRadius_ = this->get_parameter("RobotRadius").as_double();
			ObstacleDilation_ = this->get_parameter("ObstacleDilation").as_double();
			WalkHeight_ = this->get_parameter("WalkHeight").as_double();
			AllowableRange_ = this->get_parameter("AllowableRange").as_double();
			CutoffRange_ = this->get_parameter("CutoffRange").as_double();
			ForwardLinCmdLimit_ = this->get_parameter("ForwardLinCmdLimit").as_double();
			BackwardLinCmdLimit_ = this->get_parameter("BackwardLinCmdLimit").as_double();
			AngCmdLimit_ = this->get_parameter("AngCmdLimit").as_double();
			RFunctionExponent_ = this->get_parameter("RFunctionExponent").as_double();
			Epsilon_ = this->get_parameter("Epsilon").as_double();
			VarEpsilon_ = this->get_parameter("VarEpsilon").as_double();
			Mu1_ = this->get_parameter("Mu1").as_double();
			Mu2_ = this->get_parameter("Mu2").as_double();
			WorkspaceMinX_ = this->get_parameter("WorkspaceMinX").as_double();
			WorkspaceMinY_ = this->get_parameter("WorkspaceMinY").as_double();
			WorkspaceMaxX_ = this->get_parameter("WorkspaceMaxX").as_double();
			WorkspaceMaxY_ = this->get_parameter("WorkspaceMaxY").as_double();
			DiffeoTreeUpdateRate_ = this->get_parameter("SemanticMapUpdateRate").as_double();

			LinearGain_ = this->get_parameter("LinearGain").as_double();
            AngularGain_ = this->get_parameter("AngularGain").as_double();
            AngularGain_ = this->get_parameter("AngularGain").as_double();
			Goal_x_ = this->get_parameter("Goal_x").as_double();
			Goal_y_ = this->get_parameter("Goal_y").as_double();
			Tolerance_ = this->get_parameter("Tolerance").as_double();

			LowpassCutoff_ = this->get_parameter("LowpassCutOff").as_double();
			LowpassSampling_ = this->get_parameter("LowpassSampling").as_double();
			LowpassOrder_ = this->get_parameter("LowpassOrder").as_double();
			LowpassSamples_ = this->get_parameter("LowpassSamples").as_double();

			DebugFlag_ = this->get_parameter("DebugFlag").as_bool();
			SimulationFlag_ = this->get_parameter("SimulationFlag").as_bool();

			RCLCPP_INFO(this->get_logger(), "robot radius: %f", RobotRadius_);
			RCLCPP_INFO(this->get_logger(), "walk height: %f", WalkHeight_);
			RCLCPP_INFO(this->get_logger(), "varepsilon: %f", VarEpsilon_);
			RCLCPP_INFO(this->get_logger(), "epsilon: %f", Epsilon_);
			RCLCPP_INFO(this->get_logger(), "angular gain: %f", AngularGain_);
			RCLCPP_INFO(this->get_logger(), "linear gain: %f", LinearGain_);
			RCLCPP_INFO(this->get_logger(), "workspace: (%f, %f) to (%f, %f)",
				WorkspaceMinX_, WorkspaceMinY_, WorkspaceMaxX_, WorkspaceMaxY_);

			DiffeoParams_ = DiffeoParamsClass(RFunctionExponent_, Epsilon_, VarEpsilon_, Mu1_, Mu2_, 
				{	{WorkspaceMinX_, WorkspaceMinY_},
					{WorkspaceMaxX_, WorkspaceMinY_},
					{WorkspaceMaxX_, WorkspaceMaxY_},
					{WorkspaceMinX_, WorkspaceMaxY_}, 
					{WorkspaceMinX_, WorkspaceMinY_}
				});

			// Initialize publishers
			Goal_.set<0>(Goal_x_);
			Goal_.set<1>(Goal_y_);
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Setting Up Publishers");
			pub_behaviorID_ = this->create_publisher<example_interfaces::msg::UInt32>("pub_behaviorID_topic_", 1);
		    pub_behaviorMode_ = this->create_publisher<example_interfaces::msg::UInt32>("pub_behaviorMode_topic_", 1);
		    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_twist_topic_, 1);
			pub_twist_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(pub_twist_stamped_topic_, 1);
			pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pub_marker_topic_", 1);

			// Register callbacks
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Registering Callback");
			
			sub_semantic = this->create_subscription<object_pose_interface_msgs::msg::SemanticMapObjectArray>(
				sub_semantic_topic_, 1,
				std::bind(&NavigationNode::diffeo_tree_update, this, std::placeholders::_1));

			sub_laser.subscribe(this, sub_laser_topic_);
			sub_robot.subscribe(this, sub_robot_topic_);

			sync = std::make_shared<message_filters::Synchronizer<
				message_filters::sync_policies::ApproximateTime<
					sensor_msgs::msg::LaserScan,
					nav_msgs::msg::Odometry
				>>>(
				message_filters::sync_policies::ApproximateTime<
					sensor_msgs::msg::LaserScan,
					nav_msgs::msg::Odometry
				>(10), sub_laser, sub_robot);

			sync->registerCallback(
				std::bind(
					&NavigationNode::control_callback, 
			    	this,
					std::placeholders::_1, 
					std::placeholders::_2
				)
			);

			// Publish zero commands
			//RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Publishing 0 command");

			publish_behavior_id(BEHAVIOR_STAND);
            rclcpp::sleep_for(std::chrono::nanoseconds(5000000000));
			publish_behavior_id(BEHAVIOR_WALK);
			publish_twist(0.0, 0.0);

			// Spin
			// rclcpp::executors::MultiThreadedExecutor spinner(2);
			// spinner.spin();
		    
			// transform listeners
			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		}

		void publish_twist(double LinearCmd, double AngularCmd) {
			geometry_msgs::msg::Twist commandTwist;
			commandTwist.linear.x = LinearCmd;
			commandTwist.angular.z = AngularCmd;
			this->pub_twist_->publish(commandTwist);

			// Stamped message for debugging
			geometry_msgs::msg::TwistStamped commandTwistStamped;
			commandTwistStamped.twist = commandTwist;
			commandTwistStamped.header.stamp = this->now();
			commandTwistStamped.header.frame_id = world_frame_id_;
			this->pub_twist_stamped_->publish(commandTwistStamped);

			RCLCPP_INFO(this->get_logger(), "[Navigation] Twist Cmd Published: (%f, %f)", LinearCmd, AngularCmd);
			return;
		}

		void publish_behavior_id(uint16_t BehaviorIdCmd) {
            example_interfaces::msg::UInt32 commandId;
			commandId.data = BehaviorIdCmd;
			this->pub_behaviorID_->publish(commandId);
			return;
		}

		void publish_behavior_mode(uint16_t BehaviorModeCmd) {
            example_interfaces::msg::UInt32 commandMode;
			commandMode.data = BehaviorModeCmd;
			this->pub_behaviorMode_->publish(commandMode);
			return;
		}

		void diffeoTrees_cout(std::vector<std::vector<PolygonClass>> diffeoTreeArray) {
			// Mapper
			std::ofstream svg("/home/neha/Desktop/tree.svg");
			bg::svg_mapper<point> mapper(svg, 1000, 1000);
			std::vector<polygon> polygon_vector, polygon_tilde_vector;
			std::vector<point> point_vector;

			// Print polygon information
			std::cout << "Number of polygons: " << diffeoTreeArray.size() << std::endl;
			for (size_t i = 0; i < diffeoTreeArray.size(); i++) {
				std::cout << "Now printing tree for polygon " << i << std::endl;
				for (size_t j = 0; j < diffeoTreeArray[i].size(); j++) {
					std::cout << "Polygon " << j << " index: " << diffeoTreeArray[i][j].get_index() << std::endl;
					std::cout << "Polygon " << j << " depth: " << diffeoTreeArray[i][j].get_depth() << std::endl;
					std::cout << "Polygon " << j << " predecessor: " << diffeoTreeArray[i][j].get_predecessor() << std::endl;
					std::cout << "Polygon " << j << " radius: " << diffeoTreeArray[i][j].get_radius() << std::endl;
					std::cout << "Polygon " << j << " center: " << bg::dsv(diffeoTreeArray[i][j].get_center()) << std::endl;
					std::vector<point> polygon_vertices = diffeoTreeArray[i][j].get_vertices();
					polygon_vertices.push_back(polygon_vertices[0]);
					std::vector<point> polygon_vertices_tilde = diffeoTreeArray[i][j].get_vertices_tilde();
					polygon_vertices_tilde.push_back(polygon_vertices_tilde[0]);
					std::vector<point> augmented_polygon_vertices = diffeoTreeArray[i][j].get_augmented_vertices();
					augmented_polygon_vertices.push_back(augmented_polygon_vertices[0]);
					polygon polygon_vertices_polygon = BoostPointToBoostPoly(polygon_vertices);
					polygon polygon_vertices_tilde_polygon = BoostPointToBoostPoly(polygon_vertices_tilde);
					polygon augmented_polygon_vertices_polygon = BoostPointToBoostPoly(augmented_polygon_vertices);
					polygon_vector.push_back(polygon_vertices_polygon);
					polygon_tilde_vector.push_back(polygon_vertices_tilde_polygon);
					point_vector.push_back(diffeoTreeArray[i][j].get_center());
					std::cout << "Polygon " << j << " vertices: " << bg::dsv(polygon_vertices_polygon) << std::endl;
					std::cout << "Polygon " << j << " augmented vertices: " << bg::dsv(augmented_polygon_vertices_polygon) << std::endl;
					std::cout << "Polygon " << j << " size of r_t " << diffeoTreeArray[i][j].get_r_t().size() << std::endl;
					std::cout << "Polygon " << j << " size of r_n " << diffeoTreeArray[i][j].get_r_n().size() << std::endl;
					std::cout << "Polygon " << j << " collar: " << bg::dsv(polygon_vertices_tilde_polygon) << std::endl;
					std::cout << "Polygon " << j << " size of r_tilde_t " << diffeoTreeArray[i][j].get_r_tilde_t().size() << std::endl;
					std::cout << "Polygon " << j << " size of r_tilde_n " << diffeoTreeArray[i][j].get_r_tilde_n().size() << std::endl;
					std::cout << "Polygon " << j << " polygons are valid: " << bg::is_valid(polygon_vertices_polygon) << " " << bg::is_valid(polygon_vertices_tilde_polygon) << std::endl;
					std::cout << " " << std::endl;
				}
			}

			// Plot
			std::cout << polygon_vector.size() << std::endl;
			for (size_t i = 0; i < polygon_vector.size(); i++) {
				mapper.add(polygon_vector[i]);
				mapper.add(polygon_tilde_vector[i]);
				mapper.add(point_vector[i]);
			}
			for (size_t i = 0; i < polygon_vector.size(); i++) {
				mapper.map(polygon_vector[i], "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:5", 5);
				mapper.map(polygon_tilde_vector[i], "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:3", 3);
				mapper.map(point_vector[i], "fill-opacity:0.3;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:5", 5);
			}
		}

		void diffeo_tree_update(const object_pose_interface_msgs::msg::SemanticMapObjectArray::ConstPtr& semantic_map_data) {
			/**
			 * Function that updates the semantic map polygons to be used by the control callback
			 * 
			 * Input:
			 * 	1) semantic_map_data: A SemanticMapObjectArray object
			 */
			RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Navigation] Updating Diffeo Trees");

			// Check if update is needed
			// std::cout << DiffeoTreeUpdateRate_ << std::endl;
			// std::cout << rclcpp::Time::now().toSec() - DiffeoTreeUpdateTime_ << std::endl;

			// Update for Foxglove visualization
			if (SimulationFlag_) {
				latest_map = *semantic_map_data;
			}

			// Assume map is static; do not recalculate once map is set.
			/*if (DiffeoTreeArray_.size() > 0) {
				return;
			}*/

            rclcpp::Time time = this->now();
		    if (DiffeoTreeUpdateRate_ <= 0) {
		        RCLCPP_WARN(this->get_logger(), "Invalid DiffeoTreeUpdateRate. It should be greater than zero.");
		        return;
		    }

            RCLCPP_INFO(this->get_logger(), "Received %zu semantic map objects.", semantic_map_data->objects.size());
			
			if (time.seconds() - DiffeoTreeUpdateTime_ < (1.0/DiffeoTreeUpdateRate_)) {
				return;
			} else {
				RCLCPP_INFO_STREAM(this->get_logger(), "Entering diffeo tree callback");
				// Count time
				double start_time = time.seconds();

				// Initialize polygon lists
				std::vector<polygon> polygon_list;
				std::vector<polygon> polygon_list_merged;

				// Span the incoming message to add all the polygons
				for (size_t i = 0; i < semantic_map_data->objects.size(); i++) {
					// Extract points of the polygon
					std::vector<point> polygon_in_points;

					const auto& obj = semantic_map_data->objects[i];
        			const auto& vertices = obj.polygon2d.polygon.points;

					if (vertices.size() < 3) {
						RCLCPP_WARN(this->get_logger(), "Object %zu has fewer than 3 points. Skipping.", i);
						continue;
					}

					for (size_t j = 0; j < vertices.size(); j++) {
						polygon_in_points.push_back(point(vertices[j].x, vertices[j].y));
					}
					polygon polygon_in = BoostPointToBoostPoly(polygon_in_points);

					if (!bg::is_valid(polygon_in)) {
						RCLCPP_ERROR(this->get_logger(), "Invalid polygon in object %zu. Skipping.", i);
						std::cout << bg::dsv(polygon_in) << std::endl;
						continue;
					}

					RCLCPP_INFO(this->get_logger(), "Extracted valid polygon %zu.", i);

					// Dilate the polygon by the robot radius and append it to the polygon list
					multi_polygon output;
					bg::strategy::buffer::distance_symmetric<double> distance_strategy(ObstacleDilation_);
					bg::buffer(
						polygon_in,
						output,
						distance_strategy,
						side_strategy_input,
						join_strategy_input,
						end_strategy_input,
						point_strategy_input
					);

					if (output.empty()) {
						RCLCPP_WARN(this->get_logger(), "Buffer output is empty for object %zu. Skipping.", i);
						continue;
					}
					
					polygon_list.push_back(output.front());
					RCLCPP_INFO(this->get_logger(), "Dilated and added polygon %zu.", i);
				}
				
				RCLCPP_INFO(this->get_logger(), "Added %zu polygons.", polygon_list.size());

				multi_polygon output_union;

				if (polygon_list.size() >= 1) {
					output_union.push_back(polygon_list.back());
					polygon_list.pop_back();
					
					while (!polygon_list.empty()) {
						// Find diffeomorphism trees for all merged polygons
						std::vector<std::vector<PolygonClass>> localDiffeoTreeArray;
						for (size_t i = 0; i < polygon_list_merged.size(); i++) {
							std::cout << bg::dsv(polygon_list_merged[i]) << std::endl;
							std::vector<PolygonClass> tree;
							diffeoTreeConvex(BoostPointToStd(BoostPolyToBoostPoint(polygon_list_merged[i])), DiffeoParams_, &tree);
							localDiffeoTreeArray.push_back(tree);
						}
						RCLCPP_INFO_STREAM(this->get_logger(), "Found trees");
						polygon next_polygon = polygon_list.back();
						polygon_list.pop_back();
						multi_polygon temp_result;
						bg::union_(output_union, next_polygon, temp_result);
						output_union = temp_result;
					}
				}
				RCLCPP_INFO_STREAM(this->get_logger(), "Found polygon unions");
				RCLCPP_INFO_STREAM(this->get_logger(), "output_union.size(): " << output_union.size());
				for (size_t i = 0; i < output_union.size(); i++) {
					// polygon ch_component;
					// bg::convex_hull(output_union[i], ch_component);
					polygon simplified_component;
					bg::simplify(output_union[i], simplified_component, 0.2);
					polygon_list_merged.push_back(simplified_component);
				}
				RCLCPP_INFO_STREAM(this->get_logger(), "Found simplified components");

				// Find diffeomorphism trees for all merged polygons
				std::vector<std::vector<PolygonClass>> localDiffeoTreeArray;
				for (size_t i = 0; i < polygon_list_merged.size(); i++) {
					std::cout << "Merging polygon " << i << std::endl;
					std::cout << bg::dsv(polygon_list_merged[i]) << std::endl;

					std::vector<PolygonClass> tree;
					std::vector<std::vector<double>> merged_poly_points;
					try {
						std::cout << "Getting merged polygon points." << std::endl;
						merged_poly_points = BoostPointToStd(BoostPolyToBoostPoint(polygon_list_merged[i]));
						std::cout << "Converted merged polygon points." << std::endl;
					} catch (const std::exception& e) {
						RCLCPP_ERROR(this->get_logger(), "Exception caught: %s", e.what());
					}

					if (merged_poly_points.empty()) {
						RCLCPP_WARN(this->get_logger(), "Converted point list is empty at index %zu. Skipping.", i);
						continue;
					}
/*
					if (bg::is_valid(BoostPointToBoostPoly(StdToBoostPoint(merged_poly_points)))) {
						std::cout << "Polygon is valid (no self-intersections)\n";
					} else {
						std::cout << "Polygon is NOT valid (may have self-intersections)\n";
					}*/

					try {
						std::cout << "Before diffeoTreeConvex()" << std::endl;
						diffeoTreeConvex(merged_poly_points, DiffeoParams_, &tree);
						std::cout << "After diffeoTreeConvex()" << std::endl;
					} catch (const std::exception& e) {
						RCLCPP_ERROR(this->get_logger(), "Exception caught in diffeoTreeConvex: %s", e.what());
					}

					if (tree.empty()) {
						RCLCPP_WARN(this->get_logger(), "Diffeomorphism tree is empty for polygon %zu.", i);
					}

					localDiffeoTreeArray.push_back(tree);
				}
				RCLCPP_INFO_STREAM(this->get_logger(), "Found trees");
				
				// Update
				{
					std::lock_guard<std::mutex> lock(mutex_);
					DiffeoTreeArray_.clear();
					PolygonList_.clear();
					DiffeoTreeArray_.assign(localDiffeoTreeArray.begin(), localDiffeoTreeArray.end());
					PolygonList_.assign(polygon_list_merged.begin(), polygon_list_merged.end());
				}

				if (DebugFlag_) {
					diffeoTrees_cout(DiffeoTreeArray_);
				}
				RCLCPP_WARN_STREAM(this->get_logger(), "[Navigation] Updated trees in " << time.seconds() - start_time << " seconds.");

				// Update time
				DiffeoTreeUpdateTime_ = time.seconds();
			}
			return;
		}

		void control_callback (const sensor_msgs::msg::LaserScan::ConstSharedPtr& lidar_data, const nav_msgs::msg::Odometry::ConstSharedPtr& robot_data) {
			/**
			 * Callback function that implements the main part of the reactive controller
			 * 
			 * Input:
				1) lidar_data: Data received from the LIDAR sensor
			 * 	2) robot_data: Data received from the robot odometry topic
			 */

			RCLCPP_INFO(this->get_logger(), "[Navigation] In control callback");
			diffeoTrees_cout(DiffeoTreeArray_);

			// For Foxglove visualization, at a set update rate.
			rclcpp::Time update_time = this->now();
			if (SimulationFlag_ &&
				(update_time.seconds() - TrajectoryUpdateTime_ > (1.0/TrajectoryUpdateRate_))) {
				double x = robot_data->pose.pose.position.x;
				double y = robot_data->pose.pose.position.y;
				std::array<double, 2> coord = {x, y};
				trajectory_.push_back(coord);
				
				publish_marker_array();

				TrajectoryUpdateTime_ = update_time.seconds();
			}

			// Make local copies
			std::vector<polygon> localPolygonList;
			std::vector<std::vector<PolygonClass>> localDiffeoTreeArray;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				localPolygonList.assign(PolygonList_.begin(), PolygonList_.end());
				localDiffeoTreeArray.assign(DiffeoTreeArray_.begin(), DiffeoTreeArray_.end());
			}
			RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Found diffeomorphism trees for control");

			// Compute before time
            rclcpp::Time time;
			double before_time = time.seconds();

			// Assuming the incoming odometry message is in the odom frame, transform to map frame
            geometry_msgs::msg::PoseStamped odomPose, mapPose;
			odomPose.header.stamp = rclcpp::Time(0);
			odomPose.header.frame_id = odom_frame_id_;
			odomPose.pose = robot_data->pose.pose;
			try {
				/* ROS1 version for comparison.
				listener_.waitForTransform(world_frame_id_, odom_frame_id_, rclcpp::Time(0), rclcpp::Duration(std::chrono::nanoseconds(1000000000));
				listener_.transformPose(world_frame_id_, odomPose, mapPose);
				tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(
					odomPose, mapPose, "world_frame_id_", std::chrono::seconds(1)); 
				*/
                geometry_msgs::msg::TransformStamped t;
                t = tf_buffer_->lookupTransform(world_frame_id_, odom_frame_id_, tf2::TimePointZero);
                tf2::doTransform(odomPose, mapPose, t);
			} catch (tf2::TransformException &ex) {
				RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "UH OH");
				RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
				return;
			}

			// Get robot position and orientation
			tf2::Quaternion rotation = tf2::Quaternion(mapPose.pose.orientation.x, 
													 mapPose.pose.orientation.y,
													 mapPose.pose.orientation.z,
													 mapPose.pose.orientation.w);
			tf2::Matrix3x3 m(rotation);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			double x_robot_position = mapPose.pose.position.x;
			double y_robot_position = mapPose.pose.position.y;

			// Register robot state - Compensate for LIDAR to camera transform
			double RobotPositionX = x_robot_position;
			double RobotPositionY = y_robot_position;
			RobotPitch_ = pitch;
			RobotPosition_.set<0>(RobotPositionX-RobotRadius_*cos(yaw));
			RobotPosition_.set<1>(RobotPositionY-RobotRadius_*sin(yaw));
			RobotOrientation_ = yaw;

			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Found robot state");

			// Construct LIDAR object
			LIDARClass LIDAR;
			constructLIDAR2D(lidar_data, CutoffRange_, AllowableRange_, RobotPitch_, &LIDAR);

			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Constructed LIDAR");

			// Complete LIDAR readings
			completeLIDAR2D(&LIDAR);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Completed LIDAR with " << LIDAR.RangeMeasurements.size() << " rays and " << LIDAR.Angle.size() << " angles.");

			// Set the LIDAR rays that hit known obstacles to the LIDAR range
			for (size_t i = 0; i < localPolygonList.size(); i++) {
				compensateObstacleLIDAR2D(RobotPosition_, RobotOrientation_, localPolygonList[i], &LIDAR);
			}
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Compensated for known obstacles.");

			// Find list of polygon objects in the model layer based on the known obstacles
			std::vector<polygon> KnownObstaclesModel;
			for (size_t i = 0; i < localDiffeoTreeArray.size(); i++) {
				std::vector<double> theta = linspace(-M_PI, M_PI, 15);
				std::vector<std::vector<double>> model_polygon_coords;
				point root_center = localDiffeoTreeArray[i].back().get_center();
				double root_radius = localDiffeoTreeArray[i].back().get_radius();
				for (size_t j = 0; j < theta.size(); j++) {
					model_polygon_coords.push_back({root_center.get<0>()+root_radius*cos(theta[j]), root_center.get<1>()+root_radius*sin(theta[j])});
				}
				KnownObstaclesModel.push_back(BoostPointToBoostPoly(StdToBoostPoint(model_polygon_coords)));
			}
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Constructed model space.");

			// Find the diffeomorphism and its jacobian at the robot position, along with the necessary second derivatives
			std::vector<double> RobotPositionTransformed = {RobotPosition_.get<0>(), RobotPosition_.get<1>()};
			std::vector<std::vector<double>> RobotPositionTransformedD = {{1.0, 0.0}, {0.0, 1.0}};
			std::vector<double> RobotPositionTransformedDD = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			for (size_t i = 0; i < localDiffeoTreeArray.size(); i++) {
				OutputStructVector TempTransformation = polygonDiffeoConvex(RobotPositionTransformed, localDiffeoTreeArray[i], DiffeoParams_);

				std::vector<double> TempPositionTransformed = TempTransformation.Value;
				std::vector<std::vector<double>> TempPositionTransformedD = TempTransformation.Jacobian;
				std::vector<double> TempPositionTransformedDD = TempTransformation.JacobianD;

				double res1 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[0] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[4] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][0]);
				double res2 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[1] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[5] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][1]);
				double res3 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[2] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[6] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][0]);
				double res4 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[3] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[7] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][1]);
				double res5 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[0] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[4] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][0]);
				double res6 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[1] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[5] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][1]);
				double res7 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[2] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[6] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][0]);
				double res8 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[3] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[7] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][1]);

				RobotPositionTransformedDD[0] = res1;
				RobotPositionTransformedDD[1] = res2;
				RobotPositionTransformedDD[2] = res3;
				RobotPositionTransformedDD[3] = res4;
				RobotPositionTransformedDD[4] = res5;
				RobotPositionTransformedDD[5] = res6;
				RobotPositionTransformedDD[6] = res7;
				RobotPositionTransformedDD[7] = res8;

				RobotPositionTransformedD = MatrixMatrixMultiplication(TempPositionTransformedD, RobotPositionTransformedD);

				RobotPositionTransformed = TempPositionTransformed;
			}

			// Make a point for the transformed robot position
			point RobotPositionTransformedPoint = point(RobotPositionTransformed[0],RobotPositionTransformed[1]);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Found diffeomorphism.");

			// Find alpha1, alpha2, beta1, beta2
			double alpha1 = -(RobotPositionTransformedD[1][0]*cos(RobotOrientation_) + RobotPositionTransformedD[1][1]*sin(RobotOrientation_));
			double beta1 = RobotPositionTransformedDD[0]*pow(cos(RobotOrientation_),2) + (RobotPositionTransformedDD[1]+RobotPositionTransformedDD[2])*sin(RobotOrientation_)*cos(RobotOrientation_) + RobotPositionTransformedDD[3]*pow(sin(RobotOrientation_),2);
			double alpha2 = RobotPositionTransformedD[0][0]*cos(RobotOrientation_) + RobotPositionTransformedD[0][1]*sin(RobotOrientation_);
			double beta2 = RobotPositionTransformedDD[4]*pow(cos(RobotOrientation_),2) + (RobotPositionTransformedDD[5]+RobotPositionTransformedDD[6])*sin(RobotOrientation_)*cos(RobotOrientation_) + RobotPositionTransformedDD[7]*pow(sin(RobotOrientation_),2);

			// Find transformed orientation
			double RobotOrientationTransformed = atan2(RobotPositionTransformedD[1][0]*cos(RobotOrientation_)+RobotPositionTransformedD[1][1]*sin(RobotOrientation_), 
				RobotPositionTransformedD[0][0]*cos(RobotOrientation_) +
				RobotPositionTransformedD[0][1]*sin(RobotOrientation_));

			// Read LIDAR data in the model space to account for the known obstacles
			LIDARClass LIDARmodel_known;
			LIDARmodel_known.RangeMeasurements = LIDAR.RangeMeasurements;
			LIDARmodel_known.Angle = LIDAR.Angle;
			LIDARmodel_known.Range = LIDAR.Range;
			LIDARmodel_known.Infinity = LIDAR.Infinity;
			LIDARmodel_known.MinAngle = LIDAR.MinAngle;
			LIDARmodel_known.MaxAngle = LIDAR.MaxAngle;
			LIDARmodel_known.Resolution = LIDAR.Resolution;
			LIDARmodel_known.NumSample = LIDAR.NumSample;
			readLIDAR2D(point(RobotPositionTransformed[0], RobotPositionTransformed[1]), 
					RobotOrientationTransformed, KnownObstaclesModel, LIDAR.Range, LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample, &LIDARmodel_known);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Constructed known model space LIDAR with " << LIDARmodel_known.RangeMeasurements.size() << " rays and " << LIDARmodel_known.Angle.size() << " angles.");

			// Translate LIDAR data from the unknown obstacles to the transformed robot state
			LIDARClass LIDARmodel_unknown;
			LIDARmodel_unknown.RangeMeasurements = LIDAR.RangeMeasurements;
			LIDARmodel_unknown.Angle = LIDAR.Angle;
			LIDARmodel_unknown.Range = LIDAR.Range;
			LIDARmodel_unknown.Infinity = LIDAR.Infinity;
			LIDARmodel_unknown.MinAngle = LIDAR.MinAngle;
			LIDARmodel_unknown.MaxAngle = LIDAR.MaxAngle;
			LIDARmodel_unknown.Resolution = LIDAR.Resolution;
			LIDARmodel_unknown.NumSample = LIDAR.NumSample;
			translateLIDAR2D(RobotPosition_, RobotOrientation_, point(RobotPositionTransformed[0], RobotPositionTransformed[1]), RobotOrientationTransformed, ObstacleDilation_, &LIDARmodel_unknown);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Constructed unknown model space LIDAR with " <<
				//LIDARmodel_unknown.RangeMeasurements.size() << " rays and " << LIDARmodel_unknown.Angle.size() << " angles.");

			// Build final model LIDAR object
			std::vector<double> newRangeMeasurements(LIDAR.RangeMeasurements.size(), 0.0);
			for (size_t i = 0; i < LIDAR.RangeMeasurements.size(); i++) {
				newRangeMeasurements[i] = std::min(LIDARmodel_known.RangeMeasurements[i], LIDARmodel_unknown.RangeMeasurements[i]);
			}
			LIDARClass LIDARmodel(newRangeMeasurements, LIDAR.Range-bg::distance(RobotPositionTransformedPoint, RobotPosition_), 
					LIDAR.Infinity, LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.Resolution);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Constructed model space LIDAR with " << LIDARmodel.RangeMeasurements.size() << " rays and " << LIDARmodel.Angle.size() << " angles.");

			// Find local freespace; the robot radius can be zero because we have already dilated the obstacles
			polygon LF_model = localfreespaceLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, 0.0, &LIDARmodel);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Computed local free space" << bg::dsv(LF_model));

			// Find projected goal
			point LGL_model = localgoal_linearLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, LF_model, Goal_);
			RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Computed linear local goal." << bg::dsv(LGL_model));
			point LGA1_model = localgoalLIDAR2D(LF_model, Goal_);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Computed angular local goal 1." << bg::dsv(LGA1_model));
			point LGA2_model = localgoal_angularLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, LF_model, Goal_);
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Computer angular local goal 2." << bg::dsv(LGA2_model));
			point LGA_model(LGA1_model.get<0>(), LGA1_model.get<1>()); // avoid division by zero
			// RCLCPP_INFO_STREAM(this->get_logger(), "[Navigation] Computed model space projections." << bg::dsv(LGA_model));

			if (SimulationFlag_) {
				marker_add_point(LGL_model.get<0>(), LGL_model.get<1>(), 1.0, 1.0, 0.0, 0.2); // yellow
				marker_add_point(LGA_model.get<0>(), LGA_model.get<1>(), 1.0, 0.5, 0.8, 0.2); // pink
			}

			// Plot debugging
			// if (DebugFlag_) {
			// 	std::ofstream svg("/home/kodlab-xavier/freespace.svg");
			// 	bg::svg_mapper<point> mapper(svg, 1000, 1000);
			// 	mapper.add(LF_model);
			// 	mapper.add(LGL_model);
			// 	mapper.add(LGA_model);
			// 	mapper.add(RobotPosition_);
			// 	mapper.add(RobotPositionTransformedPoint);
			// 	for (size_t i = 0; i < KnownObstaclesModel.size(); i++) {
			// 		mapper.add(KnownObstaclesModel[i]);
			// 	}
			// 	mapper.map(LF_model, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:5", 5);
			// 	mapper.map(LGL_model, "fill-opacity:0.3;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:5", 5);
			// 	mapper.map(LGA_model, "fill-opacity:0.3;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:5", 5);
			// 	mapper.map(RobotPosition_, "fill-opacity:0.3;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:5", 5);
			// 	mapper.map(RobotPositionTransformedPoint, "fill-opacity:0.3;fill:rgb(153,204,153);stroke:rgb(153,204,0);stroke-width:5", 5);
			// 	for (size_t i = 0; i < KnownObstaclesModel.size(); i++) {
			// 		mapper.map(KnownObstaclesModel[i], "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:3", 3);
			// 	}
			// }

			// Compute the basis for the virtual control inputs
			double tV = (LGL_model.get<0>()-RobotPositionTransformed[0])*cos(RobotOrientationTransformed) + (LGL_model.get<1>()-RobotPositionTransformed[1])*sin(RobotOrientationTransformed);
			double tW1 = (LGA_model.get<0>()-RobotPositionTransformed[0])*cos(RobotOrientationTransformed) + (LGA_model.get<1>()-RobotPositionTransformed[1])*sin(RobotOrientationTransformed);
			double tW2 = -(LGA_model.get<0>()-RobotPositionTransformed[0])*sin(RobotOrientationTransformed) + (LGA_model.get<1>()-RobotPositionTransformed[1])*cos(RobotOrientationTransformed);

			// Compute the basis for transforming to actual control inputs
			double e_norm = sqrt(pow((RobotPositionTransformedD[0][0]*cos(RobotOrientation_)+RobotPositionTransformedD[0][1]*sin(RobotOrientation_)),2) + pow((RobotPositionTransformedD[1][0]*cos(RobotOrientation_)+RobotPositionTransformedD[1][1]*sin(RobotOrientation_)),2));
			double dksi_dpsi = MatrixDeterminant(RobotPositionTransformedD)/pow(e_norm,2);
			double DksiCosSin = (alpha1*beta1 + alpha2*beta2)/pow(e_norm,2);

			// Compute commands by accounting for limits
			double LinearCtrlGain, AngularCtrlGain;
			std::vector<double> vector_to_check_1 = {LinearGain_, ForwardLinCmdLimit_*e_norm/fabsf(tV), 0.4*AngCmdLimit_*dksi_dpsi*e_norm/(fabsf(tV*DksiCosSin))};
			LinearCtrlGain = *std::min_element(vector_to_check_1.begin(), vector_to_check_1.end());

			std::vector<double> vector_to_check_2 = {AngularGain_, 0.6*AngCmdLimit_*dksi_dpsi/(fabsf(atan2(tW2,tW1)))};
			AngularCtrlGain = *std::min_element(vector_to_check_2.begin(), vector_to_check_2.end());

			// Compute virtual and actual inputs
			double dV_virtual = LinearCtrlGain*tV;
			double LinearCmd = dV_virtual/e_norm;
			double dW_virtual = AngularCtrlGain*atan2(tW2,tW1);
			double AngularCmd = (dW_virtual-LinearCmd*DksiCosSin)/dksi_dpsi;

			// Stop if the distance from the goal is less than delta
			RCLCPP_INFO_STREAM(this->get_logger(), "distance to goal: " << bg::distance(RobotPosition_, Goal_));
			if (bg::distance(RobotPosition_, Goal_) < Tolerance_) {
				LinearCmd = 0.0;
				AngularCmd = 0.0;
				publish_behavior_id(BEHAVIOR_STAND);
                // sleep for 5s
                rclcpp::sleep_for(std::chrono::nanoseconds(5000000000));
				publish_behavior_id(BEHAVIOR_SIT);

				RCLCPP_WARN_STREAM(this->get_logger(), "[Navigation] Successfully navigated to goal and stopped...");
			}

			// Apply limits
			if (LinearCmd > ForwardLinCmdLimit_) LinearCmd = ForwardLinCmdLimit_;
			if (LinearCmd < BackwardLinCmdLimit_) LinearCmd = BackwardLinCmdLimit_;
			if (AngularCmd < -AngCmdLimit_) AngularCmd = -AngCmdLimit_;
			if (AngularCmd > AngCmdLimit_) AngularCmd = AngCmdLimit_;

			RCLCPP_WARN_STREAM(this -> get_logger(),  "[Navigation] Current robot position = " << bg::dsv(RobotPosition_) << std::endl);

			// Publish twist
			publish_twist(LinearCmd, AngularCmd);

			// Print debug information
			if (DebugFlag_) {
				std::cout << "Global goal: " << bg::dsv(Goal_) << std::endl;
				std::cout << "Local linear goal in model space: " << bg::dsv(LGL_model) << std::endl;
				std::cout << "Local angular goal 1 in model space: " << bg::dsv(LGA1_model) << std::endl;
				std::cout << "Local angular goal 2 in model space: " << bg::dsv(LGA2_model) << std::endl;
				std::cout << "Local angular goal in model space: " << bg::dsv(LGA_model) << std::endl;
				std::cout << "Robot position in model space: " << bg::dsv(RobotPositionTransformedPoint) << std::endl;
			}

			// Print time
			RCLCPP_WARN_STREAM(this->get_logger(), "[Navigation] Linear: " << LinearCmd << " Angular: " << AngularCmd);
			// RCLCPP_WARN_STREAM(this->get_logger(), "[Navigation] Command update for " << int(localDiffeoTreeArray.size()) << " polygons in " << time.seconds()-before_time << " seconds.");

			return;
		}

		void publish_marker_array() {
			/// Add obstacle markers
			for (const auto &poly: latest_map.objects) {
				marker_add_poly(poly, 1.0, 0.0, 0.0);
			}

			// Add goal
			marker_add_point(Goal_x_, Goal_y_, 0.0, 1.0, 0.0, 0.2);

			// Add trajectory
			marker_add_path(trajectory_, 0.0, 0.0, 1.0, 0.2);

			pub_marker_->publish(marker_array_);
			RCLCPP_INFO(this->get_logger(), "published marker array");

			// Reset
			marker_array_.markers.clear();
			marker_id_counter_ = 0;
		}

		void marker_add_poly(const object_pose_interface_msgs::msg::SemanticMapObject &poly,
			float r, float g, float b) {

			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = world_frame_id_;
			marker.header.stamp = this->now();
			marker.ns = "obstacles";
			marker.id = marker_id_counter_++;
			marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.scale.x = 0.3;
		
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			marker.color.a = 1.0;

			auto& poly_points = poly.polygon2d.polygon.points;
			for (size_t i = 0; i < poly_points.size(); i++) {
				geometry_msgs::msg::Point p;
				p.x = poly_points[i].x;
				p.y = poly_points[i].y;
				p.z = 0.0;
				marker.points.push_back(p);
			}
			marker.points.push_back(marker.points.front()); // close polygon

			marker_array_.markers.push_back(marker);
		}

		void marker_add_path(std::vector<std::array<double, 2>> path,
			float r, float g, float b, float scale) {
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = world_frame_id_;
			marker.header.stamp = this->now();
			marker.ns = "path";
			marker.id = marker_id_counter_++;
			marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.scale.x = scale;
		
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			marker.color.a = 1.0;

			for (auto& path_pt: path) {
				geometry_msgs::msg::Point p;
				p.x = path_pt[0];
				p.y = path_pt[1];
				p.z = 0.0;
				marker.points.push_back(p);
			}

			marker_array_.markers.push_back(marker);
		}

		void marker_add_point(double x, double y, float r, float g, float b, float scale) {
			// treat point as short line segment
			std::array<double, 2> start = {x, y};
			std::array<double, 2> end = {x + 0.01, y + 0.01};
			std::vector<std::array<double, 2>> seg = {start, end};
			marker_add_path(seg, r, g, b, scale);
		}
	
	private:
		// Parameters
		std::string pub_twist_topic_;
		std::string pub_twist_stamped_topic_;

		std::string pub_behaviorID_topic_;
		std::string pub_behaviorMode_topic_;
		std::string pub_marker_topic_;

		std::string sub_laser_topic_;
		std::string sub_robot_topic_;
		std::string sub_semantic_topic_;

		std::string world_frame_id_;
		std::string odom_frame_id_;
		std::string laser_frame_id_;

        std::string target_object_;
        double target_object_length_;
        double target_object_width_;

        rclcpp::Publisher<example_interfaces::msg::UInt32>::SharedPtr pub_behaviorID_;
        rclcpp::Publisher<example_interfaces::msg::UInt32>::SharedPtr pub_behaviorMode_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_stamped_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

        rclcpp::Subscription<object_pose_interface_msgs::msg::SemanticMapObjectArray>::SharedPtr sub_semantic;
        std::shared_ptr<message_filters::Synchronizer<
                    message_filters::sync_policies::ApproximateTime<
                    sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>>> sync;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_laser;
		message_filters::Subscriber<nav_msgs::msg::Odometry> sub_robot;

		// For Foxglove simulation
		object_pose_interface_msgs::msg::SemanticMapObjectArray latest_map;
		std::vector<std::array<double, 2>> trajectory_;
		visualization_msgs::msg::MarkerArray marker_array_;
		int marker_id_counter_;

		double RobotRadius_;
		double ObstacleDilation_;
		double WalkHeight_;
		double AllowableRange_;
		double CutoffRange_;

		double ForwardLinCmdLimit_;
		double BackwardLinCmdLimit_;
		double AngCmdLimit_;

		double RFunctionExponent_;
		double Epsilon_;
		double VarEpsilon_;
		double Mu1_;
		double Mu2_;
		double WorkspaceMinX_;
		double WorkspaceMinY_;
		double WorkspaceMaxX_;
		double WorkspaceMaxY_;
		double DiffeoTreeUpdateRate_;
		DiffeoParamsClass DiffeoParams_;

		double LinearGain_;
		double AngularGain_;

		double Goal_x_;
		double Goal_y_;
		point Goal_;
		double Tolerance_;

		double LowpassCutoff_;
		double LowpassSampling_;
		double LowpassOrder_;
		double LowpassSamples_;

		point RobotPosition_;
		double RobotOrientation_;
		double RobotPitch_ ;

		std::vector<polygon> PolygonList_;
		std::vector<std::vector<PolygonClass>> DiffeoTreeArray_;

		double DiffeoTreeUpdateTime_ ;
		double last_control_callback_time_;

		bool DebugFlag_ = false;
		bool SimulationFlag_ = false;
		double TrajectoryUpdateRate_ = 1.0; // For visualization
		double TrajectoryUpdateTime_;

        std::shared_ptr<tf2_ros::TransformListener> listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		std::mutex mutex_;
};

int main(int argc, char** argv) {
	// ROS setups
	rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

	return 0;
}