/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

// SY Added
namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

// using apollo::canbus::Chassis;
// using apollo::common::ErrorCode;
// using apollo::common::Status;
// using apollo::common::VehicleStateProvider;
// using apollo::cyber::Clock;
// using apollo::localization::LocalizationEstimate;
// using apollo::planning::ADCTrajectory;

// SY Added
using apollo::prediction::PredictionObstacles;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::RoadInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::common::PointENU;


bool PlanningComponent::ChekWhetherInLaneArea(double t){
    std::vector<LaneInfoConstPtr> lanes;
    apollo::common::PointENU ego_position;
    t = t - past_ones -1;
    double x = trace_sy.position_x[t];
    double y = trace_sy.position_y[t];
    ego_position.set_x(x);
    ego_position.set_y(y);
    if (hdmap_sy->GetLanes(ego_position, 1.0, &lanes) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return false;
    }
    if (lanes.size() <= 0) {
      // AINFO << "SY Module - Not in lane";
      return false;
    }
    if (lanes.size() == 1) {
      // AINFO << "SY Module - In lane";
      return true;
    }
    if (lanes.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE lane";
      return true;
    }
    return false;
}

bool PlanningComponent::ChekWhetherInJunctionArea(double t, apollo::planning::ADCTrajectory *planning_context){
    std::vector<JunctionInfoConstPtr> junctions;
    apollo::common::PointENU ego_position;
    // double x = trace_sy.position_x[t];
    // double y = trace_sy.position_y[t];
    double x = planning_context->trajectory_point(t).path_point().x();
    double y = planning_context->trajectory_point(t).path_point().y();
    ego_position.set_x(x);
    ego_position.set_y(y);

    double heading = planning_context->trajectory_point(t).path_point().theta();
    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading);

    if (hdmap_sy->GetJunctions(ego_position, 50, &junctions) != 0) {
      AINFO << "SY Module - Fail to get junctions from base_map.";
      return false;
    }
    if (junctions.size() <= 0) {
      // AINFO << "SY Module - Not in junction";
      return false;
    }
    if (junctions.size() == 1) {
      // AINFO << "SY Module - ************In junction";
        const auto shape = junctions.front()->junction().polygon();
        bool negative = false;
        bool positive = false;
        if ( t <= 0 ){
            apollo::common::PointENU ego_position_next;
            ego_position_next.set_x(planning_context->trajectory_point(t+1).path_point().x());
            ego_position_next.set_y(planning_context->trajectory_point(t+1).path_point().y());
            double heading_next = planning_context->trajectory_point(t+1).path_point().theta();
            auto Polygon_of_ego_next = GetPolygonOfEgo(ego_position_next, heading_next);

            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                  double ego_x = Polygon_of_ego_next[aaa].x();
                  double ego_y = Polygon_of_ego_next[aaa].y();

                  double previousego_x = ego_polygon_point_x;
                  double previousego_y = ego_polygon_point_y;

                  double heading_x =  ego_x - previousego_x;
                  double heading_y =  ego_y - previousego_y;

                  double dir_to_ahead_x = point.x()- ego_x;
                  double dir_to_ahead_y = point.y()- ego_y;

                  double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                  if (result > 0){
                    positive = true;
                  }
                  else {
                    negative = true;
                  }
                  if (positive && negative) return true;
                }

            }       
        }
        else {
            apollo::common::PointENU ego_position_previous;
            ego_position_previous.set_x(planning_context->trajectory_point(t-1).path_point().x());
            ego_position_previous.set_y(planning_context->trajectory_point(t-1).path_point().y());
            double heading_previous = planning_context->trajectory_point(t-1).path_point().theta();
            auto Polygon_of_ego_previous = GetPolygonOfEgo(ego_position_previous, heading_previous);
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                    // auto point = shape.point(0);
                    double ego_x = ego_polygon_point_x;
                    double ego_y = ego_polygon_point_y;

                    double previousego_x = Polygon_of_ego_previous[aaa].x();
                    double previousego_y = Polygon_of_ego_previous[aaa].y();

                    double heading_x =  ego_x - previousego_x;
                    double heading_y =  ego_y - previousego_y;

                    double dir_to_ahead_x = point.x()- ego_x;
                    double dir_to_ahead_y = point.y()- ego_y;

                    double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                    if (result > 0){
                        positive = true;
                    }
                    else {
                        negative = true;
                    }
                    if (positive && negative) return true;
                }
            }
        }
    }


    if (junctions.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE junction";
      for (const auto junction: junctions){
        const auto shape = junction->junction().polygon();
        bool negative = false;
        bool positive = false;
        if ( t <= 0 ){
            apollo::common::PointENU ego_position_next;
            ego_position_next.set_x(planning_context->trajectory_point(t+1).path_point().x());
            ego_position_next.set_y(planning_context->trajectory_point(t+1).path_point().y());
            double heading_next = planning_context->trajectory_point(t+1).path_point().theta();
            auto Polygon_of_ego_next = GetPolygonOfEgo(ego_position_next, heading_next);

            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                  double ego_x = Polygon_of_ego_next[aaa].x();
                  double ego_y = Polygon_of_ego_next[aaa].y();

                  double previousego_x = ego_polygon_point_x;
                  double previousego_y = ego_polygon_point_y;

                  double heading_x =  ego_x - previousego_x;
                  double heading_y =  ego_y - previousego_y;

                  double dir_to_ahead_x = point.x()- ego_x;
                  double dir_to_ahead_y = point.y()- ego_y;

                  double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                  if (result > 0){
                    positive = true;
                  }
                  else {
                    negative = true;
                  }
                  if (positive && negative) return true;
                }

            }       
        }
        else {
            apollo::common::PointENU ego_position_previous;
            ego_position_previous.set_x(planning_context->trajectory_point(t-1).path_point().x());
            ego_position_previous.set_y(planning_context->trajectory_point(t-1).path_point().y());
            double heading_previous = planning_context->trajectory_point(t-1).path_point().theta();
            auto Polygon_of_ego_previous = GetPolygonOfEgo(ego_position_previous, heading_previous);
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                    // auto point = shape.point(0);
                    double ego_x = ego_polygon_point_x;
                    double ego_y = ego_polygon_point_y;

                    double previousego_x = Polygon_of_ego_previous[aaa].x();
                    double previousego_y = Polygon_of_ego_previous[aaa].y();

                    double heading_x =  ego_x - previousego_x;
                    double heading_y =  ego_y - previousego_y;

                    double dir_to_ahead_x = point.x()- ego_x;
                    double dir_to_ahead_y = point.y()- ego_y;

                    double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                    if (result > 0){
                        positive = true;
                    }
                    else {
                        negative = true;
                    }
                    if (positive && negative) return true;
                }
            }
        }
      }
      return false;
    }



    return false; 
}

bool PlanningComponent::ChekWhetherInCrosswalkArea(double t, apollo::planning::ADCTrajectory *planning_context){
    std::vector<CrosswalkInfoConstPtr> crosswalks;
    apollo::common::PointENU ego_position;
    // double x = trace_sy.position_x[t];
    // double y = trace_sy.position_y[t];
    double x = planning_context->trajectory_point(t).path_point().x();
    double y = planning_context->trajectory_point(t).path_point().y();
    ego_position.set_x(x);
    ego_position.set_y(y);

    double heading = planning_context->trajectory_point(t).path_point().theta();
    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading);

    if (hdmap_sy->GetCrosswalks(ego_position, 50, &crosswalks) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return false;
    }
    if (crosswalks.size() <= 0) {
      // AINFO << "SY Module - Not in lane";
      return false;
    }
    if (crosswalks.size() == 1) {
      // AINFO << "SY Module - In lane";
        const auto shape = crosswalks.front()->crosswalk().polygon();
        bool negative = false;
        bool positive = false;
        if ( t <= 0 ){
            apollo::common::PointENU ego_position_next;
            ego_position_next.set_x(planning_context->trajectory_point(t+1).path_point().x());
            ego_position_next.set_y(planning_context->trajectory_point(t+1).path_point().y());
            double heading_next = planning_context->trajectory_point(t+1).path_point().theta();
            auto Polygon_of_ego_next = GetPolygonOfEgo(ego_position_next, heading_next);

            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                  double ego_x = Polygon_of_ego_next[aaa].x();
                  double ego_y = Polygon_of_ego_next[aaa].y();

                  double previousego_x = ego_polygon_point_x;
                  double previousego_y = ego_polygon_point_y;

                  double heading_x =  ego_x - previousego_x;
                  double heading_y =  ego_y - previousego_y;

                  double dir_to_ahead_x = point.x()- ego_x;
                  double dir_to_ahead_y = point.y()- ego_y;

                  double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                  if (result > 0){
                    positive = true;
                  }
                  else {
                    negative = true;
                  }
                  if (positive && negative) return true;
                }

            }       
        }
        else {
            apollo::common::PointENU ego_position_previous;
            ego_position_previous.set_x(planning_context->trajectory_point(t-1).path_point().x());
            ego_position_previous.set_y(planning_context->trajectory_point(t-1).path_point().y());
            double heading_previous = planning_context->trajectory_point(t-1).path_point().theta();
            auto Polygon_of_ego_previous = GetPolygonOfEgo(ego_position_previous, heading_previous);
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                    // auto point = shape.point(0);
                    double ego_x = ego_polygon_point_x;
                    double ego_y = ego_polygon_point_y;

                    double previousego_x = Polygon_of_ego_previous[aaa].x();
                    double previousego_y = Polygon_of_ego_previous[aaa].y();

                    double heading_x =  ego_x - previousego_x;
                    double heading_y =  ego_y - previousego_y;

                    double dir_to_ahead_x = point.x()- ego_x;
                    double dir_to_ahead_y = point.y()- ego_y;

                    double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                    if (result > 0){
                        positive = true;
                    }
                    else {
                        negative = true;
                    }
                    if (positive && negative) return true;
                }
            }
        }
    }
    if (crosswalks.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE junction";
      for (const auto crosswalk: crosswalks){
        const auto shape = crosswalk->crosswalk().polygon();
        bool negative = false;
        bool positive = false;
        if ( t <= 0 ){
            apollo::common::PointENU ego_position_next;
            ego_position_next.set_x(planning_context->trajectory_point(t+1).path_point().x());
            ego_position_next.set_y(planning_context->trajectory_point(t+1).path_point().y());
            double heading_next = planning_context->trajectory_point(t+1).path_point().theta();
            auto Polygon_of_ego_next = GetPolygonOfEgo(ego_position_next, heading_next);

            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                  double ego_x = Polygon_of_ego_next[aaa].x();
                  double ego_y = Polygon_of_ego_next[aaa].y();

                  double previousego_x = ego_polygon_point_x;
                  double previousego_y = ego_polygon_point_y;

                  double heading_x =  ego_x - previousego_x;
                  double heading_y =  ego_y - previousego_y;

                  double dir_to_ahead_x = point.x()- ego_x;
                  double dir_to_ahead_y = point.y()- ego_y;

                  double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                  if (result > 0){
                    positive = true;
                  }
                  else {
                    negative = true;
                  }
                  if (positive && negative) return true;
                }

            }       
        }
        else {
            apollo::common::PointENU ego_position_previous;
            ego_position_previous.set_x(planning_context->trajectory_point(t-1).path_point().x());
            ego_position_previous.set_y(planning_context->trajectory_point(t-1).path_point().y());
            double heading_previous = planning_context->trajectory_point(t-1).path_point().theta();
            auto Polygon_of_ego_previous = GetPolygonOfEgo(ego_position_previous, heading_previous);
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double ego_polygon_point_x = Polygon_of_ego[aaa].x();
                double ego_polygon_point_y = Polygon_of_ego[aaa].y();
                for (const auto point: shape.point()){
                    // auto point = shape.point(0);
                    double ego_x = ego_polygon_point_x;
                    double ego_y = ego_polygon_point_y;

                    double previousego_x = Polygon_of_ego_previous[aaa].x();
                    double previousego_y = Polygon_of_ego_previous[aaa].y();

                    double heading_x =  ego_x - previousego_x;
                    double heading_y =  ego_y - previousego_y;

                    double dir_to_ahead_x = point.x()- ego_x;
                    double dir_to_ahead_y = point.y()- ego_y;

                    double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

                    if (result > 0){
                        positive = true;
                    }
                    else {
                        negative = true;
                    }
                    if (positive && negative) return true;
                }
            }
        }
      }
      return false;
    }    return false;
}

double PlanningComponent::GetLaneNumberOfRoad(double t){
    std::vector<RoadInfoConstPtr> roads;
    apollo::common::PointENU ego_position;
    t = t - past_ones - 1;
    double x = trace_sy.position_x[t];
    double y = trace_sy.position_y[t];
    ego_position.set_x(x);
    ego_position.set_y(y);
    if (hdmap_sy->GetRoads(ego_position, 1.0, &roads) != 0) {
      // AINFO << "SY Module - Fail to get roads from base_map.";
      return 0;
    }
    if (roads.size() == 1) {    
      if (roads.front()->road().section_size() > 0) {
          double size = 0;
          for(auto every_section: roads.front()->road().section()){
              size = size + every_section.lane_id_size();
          }
          // AINFO << "SY Module - road lanes: " << size;
          return size;
      }  
      return 0;     
    }

    if (roads.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE roads";
      if (roads.front()->road().section_size() > 0) {
          double size = 0;
          for(auto every_section: roads.front()->road().section()){
              size = size + every_section.lane_id_size();
          }
          // AINFO << "SY Module - road lanes: " << size;
          return size;
      }  
      return 0;  
    }
    return 0;
}

double PlanningComponent::GetLaneDirectionOfRoad(double t){
    std::vector<LaneInfoConstPtr> lanes;
    apollo::common::PointENU ego_position;

    t = t - past_ones - 1;

    double x = trace_sy.position_x[t];
    double y = trace_sy.position_y[t];
    ego_position.set_x(x);
    ego_position.set_y(y);
    if (hdmap_sy->GetLanes(ego_position, 1.0, &lanes) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return 0;
    }
    if (lanes.size() <= 0) {
      // AINFO << "SY Module - Not in lane";
      return 0;
    }
    if (lanes.size() == 1) {
      // AINFO << "SY Module - In lane";
      double direction =  lanes.front()->lane().turn() - 1;
      // AINFO << "SY Module - lane direction: "<< direction;
      return direction;
    }
    if (lanes.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE lane";
      double direction =  lanes.front()->lane().turn() - 1;
      // AINFO << "SY Module - lane direction: "<< direction;
      return direction;
    }
    return 0;
}

double PlanningComponent::GetLaneSideOfRoad(double t){
    std::vector<LaneInfoConstPtr> lanes;
    apollo::common::PointENU ego_position;

    t = t - past_ones - 1;

    double x = trace_sy.position_x[t];
    double y = trace_sy.position_y[t];
    ego_position.set_x(x);
    ego_position.set_y(y);
    if (hdmap_sy->GetLanes(ego_position, 1.0, &lanes) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return 0;
    }
    if (lanes.size() <= 0) {
      // AINFO << "SY Module - Not in lane";
      return 0;
    }
    if (lanes.size() == 1) {
      // AINFO << "SY Module - In lane";
      if (lanes.front()->lane().left_neighbor_forward_lane_id_size() > lanes.front()->lane().right_neighbor_forward_lane_id_size()){
        // AINFO << "SY Module - laneside: right";
        return 2; //right lane side
      }
      else {
        // AINFO << "SY Module - laneside: left";
        return 1; //left lane side
      }      
    }
    if (lanes.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE lane";
      if (lanes.front()->lane().left_neighbor_forward_lane_id_size() > lanes.front()->lane().right_neighbor_forward_lane_id_size()){
        // AINFO << "SY Module - laneside: right";
        return 2; //right lane side
      }
      else {
        // AINFO << "SY Module - laneside: left";
        return 1; //left lane side
      }  
    }
    return 0;
}

double PlanningComponent::CalculationDistanceToALine(double x, double y, double start_x, double start_y, double end_x, double end_y){
  double E_F = x; // ego_x
  double E_S = y; // ego_y

  double A_F = start_x; // start_x
  double A_S = start_y; // start_y
  // if (length_ <= kMathEpsilon) {
  //   return point.DistanceTo(start_);
  // }

  auto B_F = end_x; // end_x
  auto B_S = end_y; // end_y

  // A is start, B is end, E is ego
  // pair<double, double> AB;
  double AB_F = B_F - A_F;
  double AB_S = B_S - A_S;
 
  // vector BP
  // pair<double, double> BE;
  double BE_F = E_F - B_F;
  double BE_S = E_S - B_S;
 
  // vector AP
  // pair<double, double> AE;
  double AE_F = E_F - A_F;
  double AE_S = E_S - A_S;
 
  // Variables to store dot product
  double AB_BE, AB_AE;
 
  // Calculating the dot product
  AB_BE = (AB_F * BE_F + AB_S * BE_S);
  AB_AE = (AB_F * AE_F + AB_S * AE_S);
 
  // Minimum distance from
  // point E to the line segment
  double result = 0;
 
  // Case 1
  if (AB_BE > 0) {
  // Finding the magnitude
    double y = E_S - B_S;
    double x = E_F - B_F;
    result = std::sqrt(x * x + y * y);
  } 
  // Case 2
  else if (AB_AE < 0) {
    double y = E_S - A_S;
    double x = E_F - A_F;
    result = std::sqrt(x * x + y * y);
  }
  // Case 3
  else {
  // Finding the perpendicular distance
    double x1 = AB_F;
    double y1 = AB_S;
    double x2 = AE_F;
    double y2 = AE_S;
    double mod = std::sqrt(x1 * x1 + y1 * y1);
    result = std::abs(x1 * y2 - y1 * x2) / mod;
  }
  return result;
}

bool PlanningComponent::CheckWhetherCrosswalkisAhead(double t, apollo::hdmap::Id the_id){
  const auto shape = hdmap_sy->GetCrosswalkById(the_id)->crosswalk().polygon();
 
  t = t - past_ones - 1;

  if (shape.point_size() > 0){
    auto point = shape.point(0);
    if ( t == 0 ){
      double ego_x = second_x;
      double ego_y = second_y;

      double previousego_x = trace_sy.position_x[t];
      double previousego_y = trace_sy.position_y[t];

      double heading_x =  ego_x - previousego_x;
      double heading_y =  ego_y - previousego_y;

      double dir_to_ahead_x = point.x()- ego_x;
      double dir_to_ahead_y = point.y()- ego_y;

      double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

      if (result > 0){
        return true;
      }
      else {
        return false;
      }
    }
    else {
      double ego_x = trace_sy.position_x[t];
      double ego_y = trace_sy.position_y[t];

      double previousego_x = trace_sy.position_x[t-1];
      double previousego_y = trace_sy.position_y[t-1];

      double heading_x =  ego_x - previousego_x;
      double heading_y =  ego_y - previousego_y;

      double dir_to_ahead_x = point.x()- ego_x;
      double dir_to_ahead_y = point.y()- ego_y;

      double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

      if (result > 0){
        return true;
      }
      else {
        return false;
      }
    }
  }

  
  return true;
}

bool PlanningComponent::CheckWhetherJunctionisAhead(double t, apollo::hdmap::Id the_id){
  const auto shape = hdmap_sy->GetJunctionById(the_id)->junction().polygon();
  
  t = t - past_ones - 1;

  if (shape.point_size() > 0){
    auto point = shape.point(0);
    if ( t == 0 ){
      // for (const auto point: shape.point()){
        double ego_x = second_x;
        double ego_y = second_y;

        double previousego_x = trace_sy.position_x[t];
        double previousego_y = trace_sy.position_y[t];

        double heading_x =  ego_x - previousego_x;
        double heading_y =  ego_y - previousego_y;

        double dir_to_ahead_x = point.x()- ego_x;
        double dir_to_ahead_y = point.y()- ego_y;

        double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

        if (result > 0){
          return true;
        }
        else {
          return false;
        }
      // }
    }
    else {
      // for (const auto point: shape.point()){
          // auto point = shape.point(0);
          double ego_x = trace_sy.position_x[t];
          double ego_y = trace_sy.position_y[t];

          double previousego_x = trace_sy.position_x[t-1];
          double previousego_y = trace_sy.position_y[t-1];

          double heading_x =  ego_x - previousego_x;
          double heading_y =  ego_y - previousego_y;

          double dir_to_ahead_x = point.x()- ego_x;
          double dir_to_ahead_y = point.y()- ego_y;

          double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

          if (result <= 0){
            return false;
          }
      // }
    }
  }

  
  return true;
}

bool PlanningComponent::CheckWhetherStoplineisAhead(double t, apollo::hdmap::Id the_id){
  const auto shape = hdmap_sy->GetSignalById(the_id)->signal().stop_line(0).segment(0).line_segment();

  t = t - past_ones - 1;

  if (shape.point_size() > 0){
    auto point = shape.point(0);
    if ( t <= 0 ){
      double ego_x = second_x;
      double ego_y = second_y;

      double previousego_x = trace_sy.position_x[t];
      double previousego_y = trace_sy.position_y[t];

      double heading_x =  ego_x - previousego_x;
      double heading_y =  ego_y - previousego_y;

      double dir_to_ahead_x = point.x()- ego_x;
      double dir_to_ahead_y = point.y()- ego_y;

      double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

      if (result > 0){
        return true;
      }
      else {
        return false;
      }
    }
    else {
          double ego_x = trace_sy.position_x[t];
          double ego_y = trace_sy.position_y[t];

          double previousego_x = trace_sy.position_x[t-1];
          double previousego_y = trace_sy.position_y[t-1];

          double heading_x =  ego_x - previousego_x;
          double heading_y =  ego_y - previousego_y;

          double dir_to_ahead_x = point.x()- ego_x;
          double dir_to_ahead_y = point.y()- ego_y;

          double result = dir_to_ahead_x * heading_x + heading_y * dir_to_ahead_y;

          if (result <= 0){
            return false;
          }
    }
  }
  return true;
}

double PlanningComponent::CalculationDistanceToStopline(double t, apollo::planning::ADCTrajectory *planning_context){
    double distance = 50;
    std::vector<SignalInfoConstPtr> signals;
    apollo::common::PointENU ego_position;
    // double x = trace_sy.position_x[t];
    // double y = trace_sy.position_y[t];
    double x = planning_context->trajectory_point(t).path_point().x();
    double y = planning_context->trajectory_point(t).path_point().y();
    ego_position.set_x(x);
    ego_position.set_y(y);

    double heading = planning_context->trajectory_point(t).path_point().theta();
    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading);

    if (hdmap_sy->GetSignals(ego_position, 50.0, &signals) != 0) {
      AINFO << "SY Module - Fail to get signals from base_map.";
      return distance;
    }
    if (signals.size() <= 0) {
      // AINFO << "SY Module - No signals";
      return distance;
    }
    if (signals.size() == 1) {    
      if (CheckWhetherStoplineisAhead(t, signals.front()->signal().id())) {
        for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
            const auto stopline = signals.front()->signal().stop_line(0);
            double start_x = stopline.segment(0).line_segment().point(0).x(); // start_x
            double start_y = stopline.segment(0).line_segment().point(0).y(); // start_y
            double end_x = stopline.segment(0).line_segment().point(stopline.segment(0).line_segment().point_size() -1).x(); // end_x
            double end_y = stopline.segment(0).line_segment().point(stopline.segment(0).line_segment().point_size() -1).y(); // end_y

            double point_x = Polygon_of_ego[aaa].x();
            double point_y = Polygon_of_ego[aaa].y();
            double distance_temp = CalculationDistanceToALine(point_x, point_y, start_x, start_y, end_x, end_y);
            if (distance_temp < distance){
                distance = distance_temp;
            }
        }
      }
      // AINFO << "SY Module - ***********stopline distance: "<< distance;
      return distance;
    }
    if (signals.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE stopline";
        for (const auto signal: signals){
            if (CheckWhetherStoplineisAhead(t, signal->signal().id())) {
                for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                    const auto stopline = signal->signal().stop_line(0);
                    double start_x = stopline.segment(0).line_segment().point(0).x(); // start_x
                    double start_y = stopline.segment(0).line_segment().point(0).y(); // start_y

                    double end_x = stopline.segment(0).line_segment().point(stopline.segment(0).line_segment().point_size() -1).x(); // end_x
                    double end_y = stopline.segment(0).line_segment().point(stopline.segment(0).line_segment().point_size() -1).y(); // end_y

                    double point_x = Polygon_of_ego[aaa].x();
                    double point_y = Polygon_of_ego[aaa].y();
                    double distance_temp = CalculationDistanceToALine(point_x, point_y, start_x, start_y, end_x, end_y);
                    if (distance_temp < distance){
                        distance = distance_temp;
                    }
                }
            }
        }
      // AINFO << "SY Module - ***********stopline distance: "<< distance;
      return distance;
    }
    return distance;
}

double PlanningComponent::CalculateDistanceToCrosswalk(double t, apollo::planning::ADCTrajectory *planning_context){
    // const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
    double distance = 50;
    std::vector<CrosswalkInfoConstPtr> crosswalks;
    apollo::common::PointENU ego_position;
    // double x = trace_sy.position_x[t];
    // double y = trace_sy.position_y[t];
    double x = planning_context->trajectory_point(t).path_point().x();
    double y = planning_context->trajectory_point(t).path_point().y();
    ego_position.set_x(x);
    ego_position.set_y(y);

    double heading = planning_context->trajectory_point(t).path_point().theta();
    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading);

    if (hdmap_sy->GetCrosswalks(ego_position, 50.0, &crosswalks) != 0) {
      AINFO << "SY Module - Fail to get crosswalks from base_map.";
      return distance;
    }
    if (crosswalks.size() <= 0) {
      // AINFO << "SY Module - No crosswalks";
      return distance;
    }
    if (crosswalks.size() == 1) {    
      if (CheckWhetherCrosswalkisAhead(t, crosswalks.front()->crosswalk().id())) {
            const auto shape = crosswalks.front()->crosswalk().polygon(); 
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double polygon_x = Polygon_of_ego[aaa].x();
                double polygon_y = Polygon_of_ego[aaa].y();
                if (shape.point_size() > 0){
                    auto next_point = shape.point(0);
                    for (int i=0; i < shape.point_size(); i++){
                      auto every_point = shape.point(i);
                      if (i+1 < shape.point_size()) {
                        next_point = shape.point(i+1);
                      }
                      double temp = CalculationDistanceToALine(polygon_x,polygon_y,every_point.x(), every_point.y(), next_point.x(), next_point.y());
                      if (distance > temp){
                          distance = temp;
                      }
                    }
                }
            }
      }
      // else {
      //   return distance;
      // }
      return distance;
    }
    if (crosswalks.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE crosswalk";
      for (const auto crosswalk: crosswalks){
        if (CheckWhetherCrosswalkisAhead(t, crosswalk->crosswalk().id())){
            const auto shape = crosswalk->crosswalk().polygon(); 
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double polygon_x = Polygon_of_ego[aaa].x();
                double polygon_y = Polygon_of_ego[aaa].y();
                if (shape.point_size() > 0){
                    auto next_point = shape.point(0);
                    for (int i=0; i < shape.point_size(); i++){
                      auto every_point = shape.point(i);
                      if (i+1 < shape.point_size()) {
                        next_point = shape.point(i+1);
                      }
                      double temp = CalculationDistanceToALine(polygon_x,polygon_y,every_point.x(), every_point.y(), next_point.x(), next_point.y());
                      if (distance > temp){
                          distance = temp;
                      }
                    }
                }
            }
        }

      }
      // AINFO << "SY Module - ***********crosswalk distance: "<< distance;
      return distance;
    }
    return distance;
}

double PlanningComponent::CalculateDistanceToJunction(double t, apollo::planning::ADCTrajectory *planning_context){
    // const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
    double distance = 50;
    std::vector<JunctionInfoConstPtr> junctions;
    apollo::common::PointENU ego_position;
    // double x = trace_sy.position_x[t];
    // double y = trace_sy.position_y[t];
    double x = planning_context->trajectory_point(t).path_point().x();
    double y = planning_context->trajectory_point(t).path_point().y();
    ego_position.set_x(x);
    ego_position.set_y(y);

    double heading = planning_context->trajectory_point(t).path_point().theta();
    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading);



    if (hdmap_sy->GetJunctions(ego_position, 50.0, &junctions) != 0) {
      AINFO << "SY Module - Fail to get junctions from base_map.";
      return distance;
      // AINFO << "SY Module - ***********junction distance: "<< distance;
    }
    if (junctions.size() <= 0) {
      // AINFO << "SY Module - Not near junction";
      return distance;
    }
    if (junctions.size() == 1) {  
        if (CheckWhetherJunctionisAhead(t, junctions.front()->junction().id())) {
            const auto shape = junctions.front()->junction().polygon(); 
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double polygon_x = Polygon_of_ego[aaa].x();
                double polygon_y = Polygon_of_ego[aaa].y();
                if (shape.point_size() > 0){
                    auto next_point = shape.point(0);
                    for (int i=0; i < shape.point_size(); i++){
                      auto every_point = shape.point(i);
                      if (i+1 < shape.point_size()) {
                        next_point = shape.point(i+1);
                      }
                      double temp = CalculationDistanceToALine(polygon_x,polygon_y,every_point.x(), every_point.y(), next_point.x(), next_point.y());
                      if (distance > temp){
                          distance = temp;
                      }
                    }
                }
            }
        }
      // AINFO << "SY Module - ***********junction distance: "<< distance;
      return distance;
    }
    if (junctions.size() > 1) {
      // AINFO << "SY Module - In MORE THAN ONE junction";
      for (const auto junction: junctions){
        if (CheckWhetherJunctionisAhead(t, junction->junction().id())) {
            const auto shape = junctions.front()->junction().polygon(); 
            for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
                double polygon_x = Polygon_of_ego[aaa].x();
                double polygon_y = Polygon_of_ego[aaa].y();
                if (shape.point_size() > 0){
                    auto next_point = shape.point(0);
                    for (int i=0; i < shape.point_size(); i++){
                      auto every_point = shape.point(i);
                      if (i+1 < shape.point_size()) {
                        next_point = shape.point(i+1);
                      }
                      double temp = CalculationDistanceToALine(polygon_x,polygon_y,every_point.x(), every_point.y(), next_point.x(), next_point.y());
                      if (distance > temp){
                          distance = temp;
                      }
                    }
                }
            }
        }
      }

      // AINFO << "SY Module - ***********junction distance: "<< distance;
      return distance;
    }
    return distance;
}

double PlanningComponent::ProduceTraceDirection(ADCTrajectory *latest_trajectory_, double t){
    // double current_relative_time = latest_trajectory_->trajectory_point(t).relative_time();
    // double traget_relative_time = current_relative_time + 2; // within two seconds;

    // for (int temp = t; temp < latest_trajectory_->trajectory_point_size(), temp++){

    // }

    double current_theta = latest_trajectory_->trajectory_point(t).path_point().theta();
    if (t+1 < latest_trajectory_->trajectory_point_size()){
        double next_theta = latest_trajectory_->trajectory_point(t+1).path_point().theta();
        if (next_theta - current_theta < -0.01) {
            return 2;
        }
        else if (next_theta - current_theta > 0.01) {
            return 1;
        }
        else {
            return 0;
        }
    }
    else if (t-1 >= 0){
        double previous_theta = latest_trajectory_->trajectory_point(t-1).path_point().theta();
        if (current_theta - previous_theta < -0.01) {
            return 2;
        }
        else if (current_theta - previous_theta > 0.01) {
            return 1;
        }
        else {
            return 0;
        }
    }
    return 0;
}

apollo::common::PointENU PlanningComponent::AddHeadingToEgo(double angle, double x, double y, double pointx, double pointy){
    apollo::common::PointENU result;
    angle = -angle;
    double srx = (x-pointx)*std::cos(angle) + (y-pointy)*std::cos(angle)+pointx;

    double sry = (y-pointy)*std::cos(angle) - (x-pointx)*std::cos(angle)+pointy;

    result.set_x(srx);
    result.set_y(sry);

    // result = std::make_tuple(srx , sry);

    return result;
}
        
std::vector<apollo::common::PointENU> PlanningComponent::GetPolygonOfEgo(apollo::common::PointENU original_point, double heading_of_ego){
        //double zhouju = 2.71
        double zhouju = 2.697298;
        double lengthen_of_ego = 4.7;
        double width_of_ego = 2.06;

        std::vector<apollo::common::PointENU> result;

        apollo::common::PointENU point0; 
        double x = original_point.x() + (lengthen_of_ego - zhouju)/2 + zhouju;
        double y = original_point.y()+ width_of_ego/2;
        point0 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point0);

        apollo::common::PointENU point1;
        x = original_point.x() + (lengthen_of_ego - zhouju)/2 + zhouju;
        y = original_point.y() - width_of_ego/2;
        point1 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point1);

        apollo::common::PointENU point2;
        x = original_point.x() - (lengthen_of_ego - zhouju)/2 ;
        y = original_point.y() - width_of_ego/2;
        point2 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point2);

        apollo::common::PointENU point3;
        x = original_point.x() - (lengthen_of_ego - zhouju)/2;
        y = original_point.y() + width_of_ego/2;
        point3 = AddHeadingToEgo(heading_of_ego,x,y,original_point.x(),original_point.y());
        result.push_back(point3);

        return result;  
}

std::vector<apollo::common::PointENU> PlanningComponent::GetPolygonOfVehicle(apollo::common::PointENU original_point, double heading_of_ego, double lengthen_of_ego, double width_of_ego){
        std::vector<apollo::common::PointENU> result;

        double zhouju = 0;
        // double lengthen_of_ego = 4.7;
        // double width_of_ego = 2.06;

        apollo::common::PointENU point0; 
        double x = original_point.x() + (lengthen_of_ego - zhouju)/2 + zhouju;
        double y = original_point.y()+ width_of_ego/2;
        point0 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point0);

        apollo::common::PointENU point1;
        x = original_point.x() + (lengthen_of_ego - zhouju)/2 + zhouju;
        y = original_point.y() - width_of_ego/2;
        point1 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point1);

        apollo::common::PointENU point2;
        x = original_point.x() - (lengthen_of_ego - zhouju)/2 ;
        y = original_point.y() - width_of_ego/2;
        point2 = AddHeadingToEgo(heading_of_ego, x, y,original_point.x(),original_point.y());
        result.push_back(point2);

        apollo::common::PointENU point3;
        x = original_point.x() - (lengthen_of_ego - zhouju)/2;
        y = original_point.y() + width_of_ego/2;
        point3 = AddHeadingToEgo(heading_of_ego,x,y,original_point.x(),original_point.y());
        result.push_back(point3);
        return result;  
}

void PlanningComponent::ProduceTrajectorySignals(ADCTrajectory *latest_trajectory_){
    if (latest_trajectory_->trajectory_point().empty()) {
      AINFO << "SY Module - Planning has no trajectory point. ";
      return;
    }
    else{
      trace_sy.speed.clear();
      trace_sy.acceleration.clear();
      trace_sy.relativetime.clear();
      trace_sy.direction.clear();
      trace_sy.heading.clear(); 
      trace_sy.position_x.clear();
      trace_sy.position_y.clear();
      trace_sy.position_z.clear();
      trace_sy.position_path_id.clear();
      trace_sy.position_path_s.clear();
      trace_sy.gear.clear();

      trace_sy.lane_num.clear();
      trace_sy.lane_side.clear();
      trace_sy.lane_direction.clear();

      trace_sy.D_junction.clear();
      trace_sy.D_crosswalk.clear();
      trace_sy.D_stopline.clear();

      trace_sy.isChangingLane.clear();
      trace_sy.isOvertaking.clear();
      trace_sy.isTurningAround.clear();

      double gear = latest_trajectory_->gear();

      second_x = latest_trajectory_->trajectory_point(1).path_point().x(); // second position_x
      second_y = latest_trajectory_->trajectory_point(1).path_point().y(); // second position_y

      // for (auto every_trajectory_point: latest_trajectory_->trajectory_point()) {
      for (int i = 0; i < latest_trajectory_->trajectory_point_size(); i++) {
        const auto every_trajectory_point = latest_trajectory_->trajectory_point(i);
        double relativetime = every_trajectory_point.relative_time(); // relative_time

        if (relativetime >= 0){
            double speed = every_trajectory_point.v() * 3.6; // speed km/h
            double acceleration = every_trajectory_point.a(); // acceleration
            
            double heading = every_trajectory_point.path_point().theta(); // steer 
            double position_x = every_trajectory_point.path_point().x(); // position_x
            double position_y = every_trajectory_point.path_point().y(); // position_y
            double position_z = every_trajectory_point.path_point().z(); // position_z
            std::string position_path_id = every_trajectory_point.path_point().lane_id(); // lane_id
            double position_path_s = every_trajectory_point.path_point().s(); // the overall path lengthen

            double direction0 = ProduceTraceDirection(latest_trajectory_, i);

            double ischangingline = 0;
            double isovertaking = 0;
            double isturningaround = 0;

            if (direction0 != 0){
                ischangingline = 1;
            }
            else {
                ischangingline = 0;
                isovertaking = 0;
                isturningaround = 0;
            }
            

            trace_sy.speed.push_back(speed);
            trace_sy.direction.push_back(direction0);
            trace_sy.acceleration.push_back(acceleration);
            trace_sy.relativetime.push_back(relativetime);
            trace_sy.heading.push_back(heading);
            trace_sy.position_x.push_back(position_x);
            trace_sy.position_y.push_back(position_y);
            trace_sy.position_z.push_back(position_z);
            trace_sy.position_path_id.push_back(position_path_id);
            trace_sy.position_path_s.push_back(position_path_s);   

            trace_sy.isChangingLane.push_back(ischangingline);
            trace_sy.isOvertaking.push_back(isovertaking);
            trace_sy.isTurningAround.push_back(isturningaround);

            trace_sy.gear.push_back(gear); 

            // AINFO << "SY Module - ************lane_id: "<< trace_sy.position_path_id[i];
            // AINFO << "SY Module - ************direction: "<< trace_sy.direction[i];

            //double theta = every_trajectory_point.path_point().theta(); // the overall path lengthen

            if (ChekWhetherInJunctionArea(i, latest_trajectory_)) {
                trace_sy.lane_num.push_back(0);
                trace_sy.lane_direction.push_back(0);
                trace_sy.lane_side.push_back(0);
                trace_sy.D_junction.push_back(0);
                // AINFO << "SY Module - ***********junction distance: "<< 0;
            }
            else if (ChekWhetherInLaneArea(i)) {
                trace_sy.lane_num.push_back(GetLaneNumberOfRoad(i));
                trace_sy.lane_direction.push_back(GetLaneDirectionOfRoad(i));
                trace_sy.lane_side.push_back(GetLaneSideOfRoad(i));
                trace_sy.D_junction.push_back(CalculateDistanceToJunction(i, latest_trajectory_));
            }
            else {
                trace_sy.lane_num.push_back(trace_sy.lane_num[i-1]);
                trace_sy.lane_direction.push_back(trace_sy.lane_num[i-1]);
                trace_sy.lane_side.push_back(trace_sy.lane_num[i-1]);
                trace_sy.D_junction.push_back(CalculateDistanceToJunction(i, latest_trajectory_));
                // AINFO << "SY Module - ERROR: Not in lane or junction!";
            }

            // AINFO << "SY Module - ************D_junction: "<< trace_sy.D_junction[i];

            if (ChekWhetherInCrosswalkArea(i, latest_trajectory_)){
              trace_sy.D_crosswalk.push_back(0);
            }
            else {
              trace_sy.D_crosswalk.push_back(CalculateDistanceToCrosswalk(i, latest_trajectory_));
            }
            
            // AINFO << "SY Module - ************D_crosswalk: "<< trace_sy.D_crosswalk[i];

            trace_sy.D_stopline.push_back(CalculationDistanceToStopline(i, latest_trajectory_));
            // AINFO << "SY Module - ************D_stopline: "<< trace_sy.D_stopline[i];
        }
        else {
            past_ones = i;
        }
        

      }
      // AINFO << "SY Module - length of the trace(planning): "<< trace_sy.gear.size();      
    }
    return;
}

double PlanningComponent::CalculateDistanceFromObstacleToVehicle(double timestep, apollo::common::PointENU obstacle_pos, double obs_heading, double id_of_obstacle){
    double distance = 50;

    apollo::common::PointENU ego_position;
    double x = trace_sy.position_x[timestep];
    double y = trace_sy.position_y[timestep];
    ego_position.set_x(x);
    ego_position.set_y(y);
    double heading_of_ego = trace_sy.heading[timestep];

    auto Polygon_of_ego = GetPolygonOfEgo(ego_position, heading_of_ego);


    auto length_obstacle = local_view_.prediction_obstacles->prediction_obstacle(id_of_obstacle).perception_obstacle().length();
    auto width_obstacle = local_view_.prediction_obstacles->prediction_obstacle(id_of_obstacle).perception_obstacle().width();

    auto Polygon_of_obstacle = GetPolygonOfVehicle(obstacle_pos, obs_heading, length_obstacle, width_obstacle);

    for (unsigned int aaa = 0; (unsigned int) aaa < Polygon_of_ego.size(); aaa++ ){
        double polygon_x = Polygon_of_ego[aaa].x();
        double polygon_y = Polygon_of_ego[aaa].y();
        
        auto next_point = Polygon_of_obstacle[0];
        for (unsigned int i=0; (unsigned int) i < Polygon_of_obstacle.size(); i++){
            auto every_point = Polygon_of_obstacle[i];
            if (i+1 < Polygon_of_obstacle.size()) {
                next_point = Polygon_of_obstacle[i+1];
            }
            double temp = CalculationDistanceToALine(polygon_x,polygon_y,every_point.x(), every_point.y(), next_point.x(), next_point.y());
            if (distance > temp){
                distance = temp;
            }
        }
        
    }

    return distance;
}

bool PlanningComponent::CheckWhetherNPCAhead(double timestep, double obstacle_x, double obstacle_y){
    // auto the_obstacle = local_view_.prediction_obstacles->prediction_obstacle(vehicle_id);

    // double ego_heading = trace_sy.heading[timestep];


    // Direction vector to judge whether obstacle is ahead
    double ego_x = trace_sy.position_x[timestep];
    double ego_y = trace_sy.position_y[timestep];

    double vector0_x = obstacle_x - ego_x;
    double vector0_y = obstacle_y - ego_y;

    double vector1_x = 0;
    double vector1_y = 0;

    if (timestep + 1 < trace_sy.speed.size()) {
        vector1_x = trace_sy.heading[timestep+1] - ego_x;
        vector1_y = trace_sy.heading[timestep+1] - ego_y;
    }
    else if (trace_sy.speed.size() > 1) {
        vector1_x = ego_x - trace_sy.heading[timestep-1];
        vector1_y = ego_y - trace_sy.heading[timestep-1] ;
    }
    double direction_flag = vector0_x * vector1_x + vector0_y * vector1_y;

    // should be in same lane
    std::vector<LaneInfoConstPtr> lanes_0;
    apollo::common::PointENU ego_position;
    ego_position.set_x(ego_x);
    ego_position.set_y(ego_y);
    if (hdmap_sy->GetLanes(ego_position, 1.0, &lanes_0) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return false;
    }

    std::vector<LaneInfoConstPtr> lanes_1;
    apollo::common::PointENU obs_position;
    obs_position.set_x(obstacle_x);
    obs_position.set_y(obstacle_y);
    if (hdmap_sy->GetLanes(obs_position, 1.0, &lanes_1) != 0) {
      AINFO << "SY Module - Fail to get lanes from base_map.";
      return false;
    }

    // Check whether in same lane
    bool WhetherInSameLane = false;
    for (const auto lane_00: lanes_0){
        for (const auto lane_11: lanes_1){
            auto id0 = lane_00->lane().id().id();
            auto id1 = lane_11->lane().id().id();
            if (id0 == id1){
                WhetherInSameLane = true;
            }
        }
    }


    if (direction_flag > 0 && WhetherInSameLane) {
        return true;
    }

    return false; 
    //0: Null, 1: NPCAhead, 2: NPCBack, 3: left, 4:right, 5:nearest
}

void PlanningComponent::ProducePredictionObstaclesSignals(){
    // AINFO << "SY Module -  debug00";
    if (local_view_.prediction_obstacles->prediction_obstacle_size() <= 0) {
      // AINFO << "SY Module -  debug01";
        trace_sy.scenario.clear();
        trace_sy.PriorityVehicleAhead.clear();
        trace_sy.PriorityPedAhead.clear();

        trace_sy.NPCAhead_Speed.clear();
        trace_sy.NPCAhead_Distance.clear();
        trace_sy.NPCAhead_Dirtection.clear();
        trace_sy.NPCAhead_type.clear();

        trace_sy.NPCNearest_Speed.clear();
        trace_sy.NPCNearest_Distance.clear();
        trace_sy.NPCNearest_Dirtection.clear();
        trace_sy.NPCNearest_type.clear();

        for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.speed.size(); timestep++){
            // Prepare the predicted obstacle here
            trace_sy.scenario.push_back(0);
            trace_sy.PriorityVehicleAhead.push_back(0);
            trace_sy.PriorityPedAhead.push_back(0);

            trace_sy.NPCAhead_Speed.push_back(0);
            trace_sy.NPCAhead_Distance.push_back(50);
            trace_sy.NPCAhead_Dirtection.push_back(0);
            trace_sy.NPCAhead_type.push_back(0);

            trace_sy.NPCNearest_Speed.push_back(0);
            trace_sy.NPCNearest_Distance.push_back(50);
            trace_sy.NPCNearest_Dirtection.push_back(0);
            trace_sy.NPCNearest_type.push_back(0);


        }
        // AINFO << "SY Module - Prediction has no Obstacles. ";
        return;
    }
    else{  //If has prediction obstacles
        trace_sy.scenario.clear();
        trace_sy.PriorityVehicleAhead.clear();
        trace_sy.PriorityPedAhead.clear();

        trace_sy.NPCAhead_Speed.clear();
        trace_sy.NPCAhead_Distance.clear();
        trace_sy.NPCAhead_Dirtection.clear();
        trace_sy.NPCAhead_type.clear();

        trace_sy.NPCNearest_Speed.clear();
        trace_sy.NPCNearest_Distance.clear();
        trace_sy.NPCNearest_Dirtection.clear();
        trace_sy.NPCNearest_type.clear();

        const double scenario = local_view_.prediction_obstacles->scenario().type();

        // exam every obstacle
        // for (const auto every_obstacle: local_view_.prediction_obstacles->prediction_obstacle()) {
        double whether_priority_vehicle = 0;
        double whether_priority_ped = 0;
        for (int i = 0; i < local_view_.prediction_obstacles->prediction_obstacle_size(); i++){
            auto every_obstacle = local_view_.prediction_obstacles->prediction_obstacle(i);

            // Check whether the obstace is with priority
            const auto priority = every_obstacle.priority().priority(); // whether priority
            const auto type_of_obstacle = every_obstacle.perception_obstacle().type(); // type of the obstacle

            // Check whether there is priority vehicle or pedestrain
            if (priority == 1 && type_of_obstacle == 5){ //vehicle
                whether_priority_vehicle = 1;
            } else if (priority == 1 && (type_of_obstacle == 3 || type_of_obstacle == 4) ){ //pedestrain or bicycle
                whether_priority_ped = 1;
            }

            std::vector<double> obstacle_speed;
            std::vector<double> obstacle_x;
            std::vector<double> obstacle_y;
            std::vector<double> obstacle_theta; 
            std::vector<double> obstacle_distance; 
            std::vector<double> obstacle_direction; 
            // raw predition trace data, will process to something like NPCAhead.speed later

            // Obstacle has trajectories, the following extend the trace for it (same lengthen as the planning signals)
            int Choosen_Trajectory = 0;
            double temp_probability = 0;
            if (every_obstacle.trajectory_size() > 0) {
                // for every obstacle, choose the trjectory with highest probability
                for (int j = 0; j < every_obstacle.trajectory_size(); j++){
                    if (temp_probability < every_obstacle.trajectory(j).probability()){
                        Choosen_Trajectory = j;
                        temp_probability = every_obstacle.trajectory(j).probability();
                    }
                }
                const auto the_Choosen_Trajectory = every_obstacle.trajectory(Choosen_Trajectory); // choose the most possible predicted trajectory

                // Extend the prediction to the lengthen of the planned trajectory
                for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.speed.size(); timestep++){
                    // Prepare the speed, direction, distance to ego vechicle, of the obstacle here
                    // Extend them to the same lengthen of the trjectory signal
                    for (int j = 0; j< the_Choosen_Trajectory.trajectory_point_size(); j++){
                        if (the_Choosen_Trajectory.trajectory_point(j).relative_time() >= trace_sy.relativetime[timestep]){
                            if (j == 0 && the_Choosen_Trajectory.trajectory_point_size() > 1){
                                double temp_speed = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j).relative_time())* (the_Choosen_Trajectory.trajectory_point(j+1).v() - the_Choosen_Trajectory.trajectory_point(j).v())/ (the_Choosen_Trajectory.trajectory_point(j+1).relative_time() - the_Choosen_Trajectory.trajectory_point(j).relative_time()) + the_Choosen_Trajectory.trajectory_point(j).v();
                                double temp_x = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j).relative_time())* (the_Choosen_Trajectory.trajectory_point(j+1).path_point().x() - the_Choosen_Trajectory.trajectory_point(j).path_point().x())/ (the_Choosen_Trajectory.trajectory_point(j+1).relative_time() - the_Choosen_Trajectory.trajectory_point(j).relative_time()) + the_Choosen_Trajectory.trajectory_point(j).path_point().x();
                                double temp_y = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j).relative_time())* (the_Choosen_Trajectory.trajectory_point(j+1).path_point().y() - the_Choosen_Trajectory.trajectory_point(j).path_point().y())/ (the_Choosen_Trajectory.trajectory_point(j+1).relative_time() - the_Choosen_Trajectory.trajectory_point(j).relative_time()) + the_Choosen_Trajectory.trajectory_point(j).path_point().y();
                                double temp_theta = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j).relative_time())* (the_Choosen_Trajectory.trajectory_point(j+1).path_point().theta() - the_Choosen_Trajectory.trajectory_point(j).path_point().theta())/ (the_Choosen_Trajectory.trajectory_point(j+1).relative_time() - the_Choosen_Trajectory.trajectory_point(j).relative_time()) + the_Choosen_Trajectory.trajectory_point(j).path_point().theta();
                                double temp_direction = 0;

                                if ( timestep > 0 ){
                                    double previous_theta = obstacle_theta[timestep - 1];
                                    if (temp_theta - previous_theta < -0.01) {
                                        temp_direction = 2;
                                    }
                                    else if (temp_theta - previous_theta > 0.01) {
                                        temp_direction = 1;
                                    }
                                    else {
                                        temp_direction = 0;
                                    }
                                    if (timestep == 1){
                                        obstacle_direction[0] = temp_direction;
                                    }
                                }

                                obstacle_x.push_back(temp_x);
                                obstacle_y.push_back(temp_y);
                                obstacle_speed.push_back(temp_speed);
                                obstacle_theta.push_back(temp_theta);
                                obstacle_direction.push_back(temp_direction);
                                // AINFO << "SY Module - ***********Obstacle Prediction Speed: " << temp_speed;
                                // AINFO << "SY Module - ***********Obstacle Prediction x: " << temp_x;
                                // AINFO << "SY Module - ***********Obstacle Prediction y: " << temp_y;
                                // AINFO << "SY Module - ***********Obstacle Prediction theta: " << temp_theta;                                                         
                            }
                            else {
                                double temp_speed = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(j).v() - the_Choosen_Trajectory.trajectory_point(j-1).v())/ (the_Choosen_Trajectory.trajectory_point(j).relative_time() - the_Choosen_Trajectory.trajectory_point(j-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(j-1).v();
                                double temp_x = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(j).path_point().x() - the_Choosen_Trajectory.trajectory_point(j-1).path_point().x())/ (the_Choosen_Trajectory.trajectory_point(j).relative_time() - the_Choosen_Trajectory.trajectory_point(j-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(j-1).path_point().x();
                                double temp_y = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(j).path_point().y() - the_Choosen_Trajectory.trajectory_point(j-1).path_point().y())/ (the_Choosen_Trajectory.trajectory_point(j).relative_time() - the_Choosen_Trajectory.trajectory_point(j-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(j-1).path_point().y();
                                double temp_theta = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(j-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(j).path_point().theta() - the_Choosen_Trajectory.trajectory_point(j-1).path_point().theta())/ (the_Choosen_Trajectory.trajectory_point(j).relative_time() - the_Choosen_Trajectory.trajectory_point(j-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(j-1).path_point().theta();
                                double temp_direction = 0;
                                if ( timestep > 0 ){
                                    double previous_theta = obstacle_theta[timestep - 1];
                                    if (temp_theta - previous_theta < -0.01) {
                                        temp_direction = 2;
                                    }
                                    else if (temp_theta - previous_theta > 0.01) {
                                        temp_direction = 1;
                                    }
                                    else {
                                        temp_direction = 0;
                                    }
                                    if (timestep == 1){
                                        obstacle_direction[0] = temp_direction;
                                    }
                                }

                                obstacle_x.push_back(temp_x);
                                obstacle_y.push_back(temp_y);
                                obstacle_speed.push_back(temp_speed);
                                obstacle_theta.push_back(temp_theta);
                                obstacle_direction.push_back(temp_direction);
                                // AINFO << "SY Module - ***********Obstacle Prediction theta: " << temp_theta;
                                // AINFO << "SY Module - ***********Obstacle Prediction Speed: " << temp_speed;
                                // AINFO << "SY Module - ***********Obstacle Prediction x: " << temp_x;
                                // AINFO << "SY Module - ***********Obstacle Prediction y: " << temp_y;
                            }  
                            break;                      
                        }
                    }
                    if (timestep == obstacle_speed.size()){ // Trajectory lengthen larger than prediction
                        double temp_speed = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).v() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).v())/ (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).relative_time() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).v();
                        double temp_x = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).path_point().x() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().x())/ (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).relative_time() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().x();
                        double temp_y = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).path_point().y() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().y())/ (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).relative_time() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().y();
                        double temp_theta = (trace_sy.relativetime[timestep] - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time())* (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).path_point().theta() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().theta())/ (the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()).relative_time() - the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).relative_time()) + the_Choosen_Trajectory.trajectory_point(the_Choosen_Trajectory.trajectory_point_size()-1).path_point().theta();
                        double temp_direction = 0;
                        if ( timestep > 0 ){
                            double previous_theta = obstacle_theta[timestep - 1];
                            if (temp_theta - previous_theta < -0.01) {
                                temp_direction = 2;
                            }
                            else if (temp_theta - previous_theta > 0.01) {
                                temp_direction = 1;
                            }
                            else {
                                temp_direction = 0;
                            }
                            if (timestep == 1){
                                obstacle_direction[0] = temp_direction;
                            }
                        }

                        obstacle_x.push_back(temp_x);
                        obstacle_y.push_back(temp_y);
                        obstacle_speed.push_back(temp_speed);
                        obstacle_theta.push_back(temp_theta);
                        obstacle_direction.push_back(temp_direction);
                        // AINFO << "SY Module - ***********Obstacle Prediction theta: " << temp_theta;
                        // AINFO << "SY Module - ***********Obstacle Prediction Speed: " << temp_speed;
                        // AINFO << "SY Module - ***********Obstacle Prediction x: " << temp_x;
                        // AINFO << "SY Module - ***********Obstacle Prediction y: " << temp_y;
                    }

                    // Get the distance from the background vehicle to the ego vehicle
                    apollo::common::PointENU point_obstcale;
                    point_obstcale.set_x(obstacle_x[timestep]);
                    point_obstcale.set_y(obstacle_y[timestep]);
                    double distance = CalculateDistanceFromObstacleToVehicle(timestep, point_obstcale, obstacle_theta[timestep], i);
                    obstacle_distance.push_back(distance);

                    // AINFO << "SY Module - ***********Obstacle Prediction distance: " << distance;

                    // Judge which type the vehicle belongs to
                    // 0: Null, 1: NPCAhead, 2: NPCBack, 3: left, 4:right

                    // Check whether the vehicle is NPCAhead;
                    if (CheckWhetherNPCAhead(timestep, obstacle_x[timestep], obstacle_y[timestep])){
                        // AINFO << "SY Module - ***********NPCAhead Detecdted" ;
                        if (trace_sy.NPCAhead_Speed.size() == timestep) {
                            trace_sy.NPCAhead_Speed.push_back(obstacle_speed[timestep]);
                            trace_sy.NPCAhead_Distance.push_back(obstacle_distance[timestep]);
                            trace_sy.NPCAhead_Dirtection.push_back(obstacle_direction[timestep]);
                            trace_sy.NPCAhead_type.push_back(type_of_obstacle);
                        }
                        else if (trace_sy.NPCAhead_Speed.size() >= timestep + 1) {
                            if (trace_sy.NPCAhead_Distance[timestep] > obstacle_distance[timestep]){
                                trace_sy.NPCAhead_Speed[timestep] = obstacle_speed[timestep];
                                trace_sy.NPCAhead_Distance[timestep] = obstacle_distance[timestep];
                                trace_sy.NPCAhead_Dirtection[timestep] = obstacle_direction[timestep];
                                trace_sy.NPCAhead_type[timestep] = type_of_obstacle;
                            }
                            
                        }
                        else{
                            AINFO << "SY Module - ***********NPCAhead Lengthen error" ;
                        }
                    }
                    else {
                        if (trace_sy.NPCAhead_Speed.size() == timestep) {
                            trace_sy.NPCAhead_Speed.push_back(0);
                            trace_sy.NPCAhead_Distance .push_back(50);
                            trace_sy.NPCAhead_Dirtection.push_back(0);
                            trace_sy.NPCAhead_type.push_back(0);
                        }
                        else if (trace_sy.NPCAhead_Speed.size() >= timestep + 1) {}
                        else{
                            AINFO << "SY Module - ***********NPCAhead Lengthen error";
                        }                   
                    }


                    if (trace_sy.NPCNearest_Speed.size() == timestep) {
                        trace_sy.NPCNearest_Speed.push_back(obstacle_speed[timestep]);
                        trace_sy.NPCNearest_Distance .push_back(obstacle_distance[timestep]);
                        trace_sy.NPCNearest_Dirtection.push_back(obstacle_direction[timestep]);
                        trace_sy.NPCNearest_type.push_back(type_of_obstacle);
                    }
                    else if (trace_sy.NPCNearest_Speed.size() >= timestep + 1) {
                        if (trace_sy.NPCNearest_Distance[timestep] > obstacle_distance[timestep]) {
                            trace_sy.NPCNearest_Speed[timestep] = obstacle_speed[timestep];
                            trace_sy.NPCNearest_Distance[timestep] = obstacle_distance[timestep];
                            trace_sy.NPCNearest_Dirtection[timestep] = obstacle_direction[timestep];
                            trace_sy.NPCNearest_type[timestep] = type_of_obstacle;
                        }
                    }
                    else{
                        AINFO << "SY Module - ***********NPCNearest Lengthen error";
                    }
                }
            }
            else {
                // AINFO << "SY Module - **************This obstacle doesn't have trajectory"; 
                // if (every_obstacle.is_static()) {
                //     AINFO << "SY Module - **************This obstacle is static!!!"; 
                // }
                for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.speed.size(); timestep++){
                    // Prepare the speed, direction, distance to ego vechicle, of the obstacle here
                    // Extend them to the same lengthen of the trjectory signal
                    
                    obstacle_x.push_back(every_obstacle.perception_obstacle().position().x());
                    obstacle_y.push_back(every_obstacle.perception_obstacle().position().y());
                    obstacle_speed.push_back(0);
                    obstacle_theta.push_back(every_obstacle.perception_obstacle().theta());
                    obstacle_direction.push_back(0);

                    // Get the distance from the background vehicle to the ego vehicle
                    apollo::common::PointENU point_obstcale;
                    point_obstcale.set_x(obstacle_x[timestep]);
                    point_obstcale.set_y(obstacle_y[timestep]);
                    double distance = CalculateDistanceFromObstacleToVehicle(timestep, point_obstcale, obstacle_theta[timestep], i);
                    obstacle_distance.push_back(distance);

                    // AINFO << "SY Module - ***********Obstacle Prediction distance: " << distance;

                    // Judge which type the vehicle belongs to
                    // 0: Null, 1: NPCAhead, 2: NPCBack, 3: left, 4:right

                    // Check whether the vehicle is NPCAhead;
                    if (CheckWhetherNPCAhead(timestep, obstacle_x[timestep], obstacle_y[timestep])){
                        // AINFO << "SY Module - ***********NPCAhead Detecdted" ;
                        if (trace_sy.NPCAhead_Speed.size() == timestep) {
                            trace_sy.NPCAhead_Speed.push_back(obstacle_speed[timestep]);
                            trace_sy.NPCAhead_Distance.push_back(obstacle_distance[timestep]);
                            trace_sy.NPCAhead_Dirtection.push_back(obstacle_direction[timestep]);
                            trace_sy.NPCAhead_type.push_back(type_of_obstacle);
                        }
                        else if (trace_sy.NPCAhead_Speed.size() >= timestep + 1) {
                            if (trace_sy.NPCAhead_Distance[timestep] > obstacle_distance[timestep]){
                                trace_sy.NPCAhead_Speed[timestep] = obstacle_speed[timestep];
                                trace_sy.NPCAhead_Distance[timestep] = obstacle_distance[timestep];
                                trace_sy.NPCAhead_Dirtection[timestep] = obstacle_direction[timestep];
                                trace_sy.NPCAhead_type[timestep] = type_of_obstacle;
                            }
                            
                        }
                        else{
                            AINFO << "SY Module - ***********NPCAhead Lengthen error" ;
                        }
                    }
                    else {
                        if (trace_sy.NPCAhead_Speed.size() == timestep) {
                            trace_sy.NPCAhead_Speed.push_back(0);
                            trace_sy.NPCAhead_Distance .push_back(50);
                            trace_sy.NPCAhead_Dirtection.push_back(0);
                            trace_sy.NPCAhead_type.push_back(0);
                        }
                        else if (trace_sy.NPCAhead_Speed.size() >= timestep + 1) {}
                        else{
                            AINFO << "SY Module - ***********NPCAhead Lengthen error";
                        }                   
                    }


                    if (trace_sy.NPCNearest_Speed.size() == timestep) {
                        trace_sy.NPCNearest_Speed.push_back(obstacle_speed[timestep]);
                        trace_sy.NPCNearest_Distance .push_back(obstacle_distance[timestep]);
                        trace_sy.NPCNearest_Dirtection.push_back(obstacle_direction[timestep]);
                        trace_sy.NPCNearest_type.push_back(type_of_obstacle);
                    }
                    else if (trace_sy.NPCNearest_Speed.size() >= timestep + 1) {
                        if (trace_sy.NPCNearest_Distance[timestep] > obstacle_distance[timestep]) {
                            trace_sy.NPCNearest_Speed[timestep] = obstacle_speed[timestep];
                            trace_sy.NPCNearest_Distance[timestep] = obstacle_distance[timestep];
                            trace_sy.NPCNearest_Dirtection[timestep] = obstacle_direction[timestep];
                            trace_sy.NPCNearest_type[timestep] = type_of_obstacle;
                        }
                    }
                    else{
                        AINFO << "SY Module - ***********NPCNearest Lengthen error";
                    }

                } 
            }        
            // AINFO << "SY Module - **************length of the trace(prediction): "<< obstacle_speed.size(); 
        }

        
        for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.speed.size(); timestep++){
            // Prepare the predicted obstacle here
            trace_sy.scenario.push_back(scenario);
            trace_sy.PriorityVehicleAhead.push_back(whether_priority_vehicle);
            trace_sy.PriorityPedAhead.push_back(whether_priority_ped);
            // AINFO << "SY Module - NPCNearest_Distance: "<< trace_sy.NPCNearest_Distance[timestep]; 
        }
        // AINFO << "SY Module - length of the trace(prediction): "<< trace_sy.PriorityPedAhead.size(); 
        // AINFO << "SY Module - length of the trace(NPCAhead): "<< trace_sy.NPCAhead_type.size(); 
        // AINFO << "SY Module - length of the trace(NPCNearest): "<< trace_sy.NPCNearest_Distance.size(); 
    }

    return;
}

void PlanningComponent::ProduceTrafficLightRelatedSignals(){
    trace_sy.TL_color.clear();
    // trace_sy.TL_distance_to_stopline.clear();
    trace_sy.TL_isblinking.clear();
    if (!traffic_light_.contain_lights()) {
      ADEBUG << "SY Module - has no Traffic Light. ";
      trafficlight_tracking_time = 0;
      trafficlight_tracking_color = apollo::perception::TrafficLight::UNKNOWN;

      for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.relativetime.size(); timestep++){
        // Prepare the default signal here
        trace_sy.TL_color.push_back(3);
        // trace_sy.TL_distance_to_stopline.push_back(50);
        trace_sy.TL_isblinking.push_back(0);
      }
    }
    else{
        auto every_traffic_light = traffic_light_.traffic_light(0);
        auto TrafficLight_Color = every_traffic_light.color(); // current color

        if (trafficlight_tracking_color != TrafficLight_Color){ // if color changes, start tracking
            trafficlight_tracking_time = 0;
            trafficlight_tracking_color = TrafficLight_Color;
            trafficlight_previous_timestep = traffic_light_.header().timestamp_sec();
        }
        else { // if color not change, add tracking time
            trafficlight_tracking_time += traffic_light_.header().timestamp_sec() - trafficlight_previous_timestep;
            trafficlight_previous_timestep = traffic_light_.header().timestamp_sec();
        }

        // AINFO << "SY Module - *******trackingtime of traffic light: "<< trafficlight_tracking_time;
        // AINFO << "SY Module - *******track color of traffic light: "<< trafficlight_tracking_color;

        double THE_remainingtime;

        if (TrafficLight_Color == apollo::perception::TrafficLight::RED){
            THE_remainingtime = 15 - trafficlight_tracking_time;
        }
        else if (TrafficLight_Color == apollo::perception::TrafficLight::YELLOW){
            THE_remainingtime = 5 - trafficlight_tracking_time;
        }
        else if (TrafficLight_Color == apollo::perception::TrafficLight::GREEN){
            THE_remainingtime = 15 - trafficlight_tracking_time;
        }
        else {
            THE_remainingtime = 0;
        }

        // AINFO << "SY Module - *******THE_remainingtime of traffic light: "<< THE_remainingtime;
          
        for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.relativetime.size(); timestep++){
              // Prepare the predicted traffic light here

              //double trackingtime = every_traffic_light.tracking_time();
              // auto remainingtime = every_traffic_light.remaining_time();

              if (every_traffic_light.blink()){
                trace_sy.TL_isblinking.push_back(1);
              }
              else{
                trace_sy.TL_isblinking.push_back(0);
              }

              // auto THE_remainingtime = fmod(remainingtime, 35.0); // 35 = 15 red + 15 green + 5 yellow

              // predicated color based on remaining time
              if (TrafficLight_Color == apollo::perception::TrafficLight::RED){
                if (THE_remainingtime >= trace_sy.relativetime[timestep] ){
                  trace_sy.TL_color.push_back(2);
                }  
                else if (THE_remainingtime <= trace_sy.relativetime[timestep] - 15) {
                  trace_sy.TL_color.push_back(1);
                } 
                else {
                  trace_sy.TL_color.push_back(0);
                }       
              }
              else if (TrafficLight_Color == apollo::perception::TrafficLight::YELLOW){
                if (THE_remainingtime >= trace_sy.relativetime[timestep] ){
                  trace_sy.TL_color.push_back(0);
                }  
                else if (THE_remainingtime <= trace_sy.relativetime[timestep] - 15) {
                  trace_sy.TL_color.push_back(2);
                } 
                else {
                  trace_sy.TL_color.push_back(1);
                }   
              }
              else if (TrafficLight_Color == apollo::perception::TrafficLight::GREEN){
                if (THE_remainingtime >= trace_sy.relativetime[timestep] ){
                  trace_sy.TL_color.push_back(1);
                }  
                else if (THE_remainingtime <= trace_sy.relativetime[timestep] - 5) {
                  trace_sy.TL_color.push_back(0);
                } 
                else {
                  trace_sy.TL_color.push_back(2);
                } 
              }
              else {
                trace_sy.TL_color.push_back(3);
              }
              // AINFO << "SY Module - Distance to stop line: "<< distance_to_stop_line;
          }   
    }
    // AINFO << "SY Module - length of the trace(traffic light): "<< trace_sy.TL_color.size(); 
    return;
}

void PlanningComponent::PrepareTrace(ADCTrajectory *planning_context) {
    // Planning Trajetory 
    ProduceTrajectorySignals(planning_context);

    // Prediction Obstacles
    ProducePredictionObstaclesSignals();

    // Traffic Light
    ProduceTrafficLightRelatedSignals();

    // trace_sy.s1 = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 6, 0, 6 };

    // trace_sy.s2 = { 11 ,10, 9 ,8 ,7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 6 };

    // trace_sy.s3 = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 6, 0, 6 };

    // trace_sy.s4 = { 11 ,10, 9 ,8 ,7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 6 };

    // trace_sy.relativetime.clear();
    // trace_sy.relativetime = { 0, 2, 4, 6, 8 }; 
    // trace_sy.speed.clear();
    // trace_sy.speed = { 7.01, 6.13, 5.44, 5.09, 3.89 }; // 0

    // trace_sy.direction.clear();
    // trace_sy.direction = { 0, 0, 0, 0, 0 };  // 1
    // trace_sy.lane_num.clear();
    // trace_sy.lane_num = {3, 3, 3, 3, 3}; // 
    // trace_sy.D_crosswalk.clear();
    // trace_sy.D_crosswalk = { 45, 31.66, 20.17, 9.15, 0.25 };
    // trace_sy.D_stopline.clear();
    // trace_sy.D_stopline = { 44, 30.66, 19.17, 8.15, -0.75 };
    // trace_sy.D_junction.clear();
    // trace_sy.D_junction = { 44, 30.66, 19.17, 8.15, -0.75 };

    // trace_sy.TL_color.clear();
    // trace_sy.TL_color = {1, 0, 0, 0, 2};

    // trace_sy.PriorityPedAhead.clear();
    // trace_sy.PriorityPedAhead = { 0 ,0, 0, 1, 1 };
    // trace_sy.PriorityVehicleAhead.clear();
    // trace_sy.PriorityVehicleAhead = { 0 ,0, 0, 0, 0 };

    return;
    // Now Ready Info:
    // 1. traffic_light_: Traffic light INFO
    // 2. local_view_.prediction_obstacles: Prediction of Obstacles' motion
    // 3. local_view_.localization_estimate: the position, pose
    // 4. latest_chassis_: Chassis control from the preious loop 
    // 5. latest_trajectory_: Information of the planning trajetory 
    // 6. hdmap_sy: Map information 
}

void PlanningComponent::PrepareControlRelatedTrace(){

    trace_sy.engineOn.clear(); //37
    trace_sy.highBeamOn.clear();
    trace_sy.lowBeamOn.clear();
    trace_sy.turnSignal.clear();
    trace_sy.fogLightOn.clear();
    trace_sy.warningFlashOn.clear();
    trace_sy.toManual.clear();

    for (unsigned int timestep = 0; (unsigned int) timestep < trace_sy.speed.size(); timestep++){
        trace_sy.engineOn.push_back(1);
        trace_sy.highBeamOn.push_back(0);
        trace_sy.lowBeamOn.push_back(0);
        trace_sy.turnSignal.push_back(0);
        trace_sy.fogLightOn.push_back(0);
        trace_sy.warningFlashOn.push_back(0);
        trace_sy.toManual.push_back(0);
    }

    return;
}


}  // namespace planning
}  // namespace apollo