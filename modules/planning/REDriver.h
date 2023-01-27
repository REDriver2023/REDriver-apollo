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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"


// SY - Add
// #include "modules/perception/proto/traffic_light_detection.pb.h"
// #include "modules/prediction/proto/prediction_obstacle.pb.h"
// #include "modules/map/hdmap/hdmap_util.h"


// #include <iostream>
// #include <assert.h> 
// #include <cmath>
// #include <math.h>
// #include <algorithm>


namespace apollo {
namespace planning {

enum SpecType{
  SPECTYPE_Null = 0,
  SPECTYPE_NOT = 1,
  SPECTYPE_AND = 2,
  SPECTYPE_OR = 3,
  SPECTYPE_IMPLY = 4,
  SPECTYPE_ALWAYS = 5,
  SPECTYPE_FINALLY = 6,
  SPECTYPE_UNTIL = 7,
  SPECTYPE_NEXT = 8,

  SPECTYPE_ATOMSMALLER = 10,
  SPECTYPE_ATOMGREATER = 11,
  SPECTYPE_ATOMEQUAL = 12,
  SPECTYPE_ATOMDIFFERENT = 13,
  SPECTYPE_ATOMSMALLEREQUAL = 14,
  SPECTYPE_ATOMGREATEREQUAL = 15,
};

// enum TraceType{
//   TRACE_speed = 0,
//   TRACE_acceleration = 1,
//   TRACE_relativetime = 2,
//   TRACE_steer = 3,
//   TRACE_gear = 4,
//   TRACE_scenario = 5,
// };


class Trace
{
public:
    virtual void dosomething() {}
    virtual ~Trace() {}
    // List all the signals contain in the trace
    // std::vector<double> s1;
    // std::vector<double> s2;
    // std::vector<double> s3;
    // std::vector<double> s4;
    // std::vector<double> s5;
    // std::vector<double> s6;

    // The Planning Trajectory
    // std::vector<double> speed;  // type = 0
    // std::vector<double> acceleration; // type = 1
    // std::vector<double> relativetime; // type = 2
    // std::vector<double> steer; // type = 3
    // std::vector<double> position_x; 
    // std::vector<double> position_y; 
    // std::vector<double> position_z; 
    // std::vector<std::string> position_path_id; 
    // std::vector<double> position_path_s;

    // std::vector<apollo::canbus::Chassis::GearPosition> gear; // type = 4

    // // The prediction
    // std::vector<apollo::prediction::Scenario> scenario; // type = 5
    // std::vector<bool> PriorityVehicleAhead;
    // std::vector<bool> PriorityPedAhead;

    // // Traffic Light
    // std::vector<double> TL_distance_to_stopline;
    // std::vector<apollo::perception::TrafficLight::Color> TL_color;
    // std::vector<bool> TL_isblinking;


    // The Planning Trajectory
    std::vector<double> speed;  // type = 1  ***
    std::vector<double> acceleration; // type = 2 ***
    std::vector<double> relativetime; // type = 3 ***
    std::vector<double> direction; // type = 4 ***
    std::vector<double> heading; // type = 5 ***
    std::vector<double> position_x;
    std::vector<double> position_y;
    std::vector<double> position_z;
    std::vector<std::string> position_path_id;
    std::vector<double> position_path_s;

    std::vector<double> lane_num; //6 ***
    std::vector<double> lane_direction; //7 ***
    std::vector<double> lane_side; //8 ***
    std::vector<double> gear;   // type = 9 ***
    // The prediction
    // std::vector<double> scenario; // 
    std::vector<double> scenario;
    std::vector<double> PriorityVehicleAhead; // 10 **
    std::vector<double> PriorityPedAhead; // 11 **
    // Traffic Light
    // std::vector<double> TL_distance_to_stopline; 
    std::vector<double> TL_color; // 12 ***
    std::vector<double> TL_isblinking; // 13 ***
    // map
    std::vector<double> D_stopline; // 14 ***
    std::vector<double> D_crosswalk; // 15 ***
    std::vector<double> D_junction; // 16 ***


    std::vector<double> NPCAhead_Speed; // 17 ***
    std::vector<double> NPCAhead_Distance; // 18 ***
    std::vector<double> NPCAhead_Dirtection; // 19 ***
    std::vector<double> NPCAhead_type; // 20 ***

    std::vector<double> NPCLeft_Speed; // 21
    std::vector<double> NPCLeft_Distance; // 22
    std::vector<double> NPCLeft_Dirtection; // 23
    std::vector<double> NPCLeft_type; // 24

    std::vector<double> NPCRight_Speed; // 25
    std::vector<double> NPCRight_Distance; // 26
    std::vector<double> NPCRight_Dirtection; // 27
    std::vector<double> NPCRight_type; // 28


    std::vector<double> NPCNearest_Speed; // 29 ***
    std::vector<double> NPCNearest_Distance; // 30 ***
    std::vector<double> NPCNearest_Dirtection; // 31 ***
    std::vector<double> NPCNearest_type; // 32 ***


    std::vector<double> NPCOppo_Speed; // 33
    std::vector<double> NPCOppo_Distance; // 34
    std::vector<double> NPCOppo_Dirtection; // 35
    std::vector<double> NPCOppo_type; // 36


    std::vector<double> engineOn; //37
    std::vector<double> highBeamOn; //38
    std::vector<double> lowBeamOn; //39
    std::vector<double> turnSignal; //40
    std::vector<double> fogLightOn; //41
    std::vector<double> warningFlashOn; //42
    std::vector<double> toManual; //43


    std::vector<double> isChangingLane; //44
    std::vector<double> isOvertaking; //45
    std::vector<double> isTurningAround; //46
    



    Trace(){
      // The Planning Trajectory
      speed = {};           //1 ***
      acceleration = {};    //2 ***
      relativetime = {};    //3 ***
      direction = {};       //4 ***
      heading = {};         //5 ***
      position_x = {};      
      position_y = {};      
      position_z = {};      
      position_path_id = {};
      position_path_s = {};

      lane_num = {};        //6 ***
      lane_direction = {};  //7 ***
      lane_side = {};       //8 ***

      gear = {};            //9 ***

      scenario = {};
      PriorityVehicleAhead = {}; //10 **
      PriorityPedAhead = {};    //11 **
      
      TL_color = {};            //12 ***
      TL_isblinking = {};       //13 ***

      D_stopline = {};          //14 ***
      D_crosswalk = {};         //15 ***
      D_junction = {};          //16 ***


      NPCAhead_Speed = {}; // 17 ***
      NPCAhead_Distance = {}; // 18 ***
      NPCAhead_Dirtection = {}; // 19 ***
      NPCAhead_type = {}; // 20 ***

      NPCLeft_Speed = {}; // 21
      NPCLeft_Distance = {}; // 22
      NPCLeft_Dirtection = {}; // 23
      NPCLeft_type = {}; // 24

      NPCRight_Speed = {}; // 25
      NPCRight_Distance = {}; // 26
      NPCRight_Dirtection = {}; // 27
      NPCRight_type = {}; // 28


      NPCNearest_Speed = {}; // 29 ***
      NPCNearest_Distance = {}; // 30 ***
      NPCNearest_Dirtection = {}; // 31 ***
      NPCNearest_type = {}; // 32 ***


      NPCOppo_Speed = {}; // 33
      NPCOppo_Distance = {}; // 34
      NPCOppo_Dirtection = {}; // 35
      NPCOppo_type = {}; // 36

      engineOn = {}; //37
      highBeamOn = {}; //38
      lowBeamOn = {}; //39
      turnSignal = {}; //40
      fogLightOn = {}; //41
      warningFlashOn = {}; //42
      toManual = {}; //43

      isChangingLane = {}; //44
      isOvertaking = {}; //45
      isTurningAround = {}; //46
    }
    // double speed;   
};

class SpecBase {
public:
    virtual void dosomething() {}
    virtual ~SpecBase() {}
};

class GeneralSpec : public SpecBase
{
public:
    GeneralSpec() {
        type = 0;
        LeftNumber = 0;
        RightNumber = 0;

        whetherbounded = 0;

        LeftType = 0; // used for atom specs
        RightType = 0;

        spec = new SpecBase();
        LeftSpec = new SpecBase();
        RightSpec = new SpecBase();
    }

    GeneralSpec(int type0)
    {
        type = type0;
        LeftNumber = 0;
        RightNumber = 0;

        whetherbounded = 0;

        LeftType = 0; // used for atom specs
        RightType = 0;

        spec = new SpecBase();
        LeftSpec = new SpecBase();
        RightSpec = new SpecBase();

    }

    // contain all the possible elements
    SpecBase* spec;
    SpecBase* LeftSpec;
    SpecBase* RightSpec;

    int type;
    double LeftNumber; // For atomspec, it's the number. For G/F/U, it means the bounded number
    double RightNumber;

    int whetherbounded; // 0 means just G/F/U, 1 means G/F/U[LeftNumber, RightNumber]

    int LeftType; // used for atom specs
    int RightType;
    // TODO - THE TYPE OF SIGNALS IS LIMITED.
    // 0 - the number, in this case, use the leftnumber and rightnumber
    // 1 ... - the other signals...

    virtual void dosomething() {};
    // 0 - Null
    // 1 - not
    // 2 - and 
    // 3 - or
    // 4 - imply
    // 5 - always
    // 6 - finally
    // 7 - until
    // 8 - next

   // 10 - atomstep - "<" or "<="
   // 11 - atomstep - ">" or ">="
   // 12 - atomstep - "==" 
   // 13 - atomstep - "!="
};

class CradientMap
{
public:
    std::vector<double> speed;  // type = 1
    std::vector<double> acceleration; // type = 2
    std::vector<double> direction; // type = 4
    std::vector<double> heading; // type = 5


    std::vector<double> lane_num; //6
    std::vector<double> lane_direction; //7
    std::vector<double> lane_side; //8

    std::vector<double> gear; // type = 9

    // map
    std::vector<double> D_stopline; // 14
    std::vector<double> D_crosswalk; // 15
    std::vector<double> D_junction; // 16
};

// class StoredRobustness
// {
// public:
//     double value;
//     int timestep;
//     SpecBase* formula;
// };


// class InternalValueNode : public SpecBase {
//     public:
//         int timestep;
//         GeneralSpec* spec;
//         double robustness;
//         std::vector<InternalValueNode*> nodes;
//         virtual void dosomething() {};
// };



}  // namespace planning
}  // namespace apollo