/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"

#include "modules/planning/REDriver.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning {

class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();
                                
  //SY Added
  //prepare specs
  void Prepare_Law38();
  void Prepare_Law51_45();
  void Prepare_Law46_2();
  void Prepare_Law44();
  void PrepareSpec();
  
  //prepare the trace
  void PrepareTrace(apollo::planning::ADCTrajectory *planning_context);
  void PrepareControlRelatedTrace();
  void ProduceTrajectorySignals(apollo::planning::ADCTrajectory *planning_context);
  void ProducePredictionObstaclesSignals();
  void ProduceTrafficLightRelatedSignals();
  double ProduceTraceDirection(apollo::planning::ADCTrajectory *planning_context, double t);
  bool CheckWhetherNPCAhead(double timestep, double obstacle_x, double obstacle_y);
  apollo::common::PointENU AddHeadingToEgo(double angle, double x, double y, double pointx, double pointy);
  std::vector<apollo::common::PointENU> GetPolygonOfEgo(apollo::common::PointENU original_point, double heading_of_ego);
  std::vector<apollo::common::PointENU> GetPolygonOfVehicle(apollo::common::PointENU original_point, double heading_of_ego, double lengthen_of_ego, double width_of_ego);

  bool ChekWhetherInLaneArea(double t);
  bool ChekWhetherInJunctionArea(double t, apollo::planning::ADCTrajectory *planning_context);
  bool ChekWhetherInCrosswalkArea(double t, apollo::planning::ADCTrajectory *planning_context);
  double GetLaneNumberOfRoad(double t);
  double GetLaneDirectionOfRoad(double t);
  double GetLaneSideOfRoad(double t);
  double CalculateDistanceFromObstacleToVehicle(double timestep, apollo::common::PointENU obstacle_pos, double obs_heading, double id_of_obstacle);

  double CalculateDistanceToCrosswalk(double t, apollo::planning::ADCTrajectory *planning_context);
  double CalculateDistanceToJunction(double t, apollo::planning::ADCTrajectory *planning_context);
  double CalculationDistanceToStopline(double t, apollo::planning::ADCTrajectory *planning_context);
  double CalculationDistanceToALine(double x, double y, double start_x, double start_y, double end_x, double end_y);
  bool CheckWhetherCrosswalkisAhead(double t, apollo::hdmap::Id the_id);
  bool CheckWhetherJunctionisAhead(double t, apollo::hdmap::Id the_id);
  bool CheckWhetherStoplineisAhead(double t, apollo::hdmap::Id the_id);

  
  //prepare the algorithm
  double GetSingalByTypeByTimestep(int type, int t);
  double CalculationOfDistanceByTimeStep(GeneralSpec* spec0, int t, int L);
  void CalculateGradient(GeneralSpec* spec0, int t, int L, double previousgradient); 
  int BinarySearchForTimestep(int minL, int maxL);

  void FixTheTrajectory(apollo::planning::ADCTrajectory *planning_context, int type, int timestep, double modification_value);

  void PlanningCheckForSpec(apollo::planning::ADCTrajectory *planning_context);
  //end

 private:
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
                                
  // SY - Added
  // std::shared_ptr<cyber::Reader<prediction::PredictionObstacles>> prediction_obstacles_reader_;
  // prediction::PredictionObstacles latest_prediction_obstacles_;
 
  GeneralSpec* TheGeneralSpec = new GeneralSpec();
  Trace trace_sy;
  
  // double threshold = 0.0;
  // double threshold = 0.3;
  // double threshold = 0.6;
  double threshold = 0.9;
  // double threshold = 1.2;

  double second_x;
  double second_y;

  double trafficlight_tracking_time;
  double trafficlight_previous_timestep;
  apollo::perception::TrafficLight::Color trafficlight_tracking_color;

  // std::vector<GeneralSpec*> SpecsTodelete;

  const hdmap::HDMap *hdmap_sy = nullptr;

  double past_ones = -1;
  double number_of_fixes = 0;


  double number_of_rounds = 0;

  double max_validaion_time = 0;
  double avg_validaion_time = 0;
  double avg_running_time = 0;

  double k_for_soft = 10;
  CradientMap gradientmap;

  double count = 0;

  double distance_for_repair = 0;
  //end
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
