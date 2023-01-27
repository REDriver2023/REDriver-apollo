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


void PlanningComponent::PlanningCheckForSpec(ADCTrajectory *planning_context) {
  // AINFO << "SY Module - Planning Check For Spec";

    PrepareTrace(planning_context);

    // PrepareControlRelatedTrace();

    // number_of_rounds = number_of_rounds + 1;

    if (trace_sy.speed.size() > 0) { // if there is signal
        auto distance = CalculationOfDistanceByTimeStep(TheGeneralSpec, 0, trace_sy.speed.size());
        //AINFO << "SY Module - The overall distance: " << distance;    

        if (distance <= threshold) {
            auto choosen_timestep = BinarySearchForTimestep(1, trace_sy.speed.size());

            gradientmap.speed.clear();
            gradientmap.D_stopline.clear();
            gradientmap.D_crosswalk.clear();
            gradientmap.D_junction.clear();
            gradientmap.lane_num.clear();
            for (int i = 0; i < choosen_timestep; i++) {
                gradientmap.speed.push_back(0);
                gradientmap.D_stopline.push_back(0);
                gradientmap.D_crosswalk.push_back(0);
                gradientmap.D_junction.push_back(0);
                gradientmap.lane_num.push_back(0);
            }

            CalculateGradient(TheGeneralSpec, 0, choosen_timestep, 1);

            if (distance_for_repair < distance){
                distance_for_repair = distance;
            }
            for (int i = 0; i < choosen_timestep; i++) {
                if (std::abs(gradientmap.speed[i]) > 0.1){ //speed
                    FixTheTrajectory(planning_context, 1, i, (threshold - distance_for_repair)/gradientmap.speed[i]);
                }
                if (std::abs(gradientmap.D_stopline[i]) > 0.1){ //D_stopline
                    FixTheTrajectory(planning_context, 14, i, (threshold - distance_for_repair)/gradientmap.D_stopline[i]);
                }
                if (std::abs(gradientmap.D_crosswalk[i]) > 0.1){ //D_crosswalk
                    FixTheTrajectory(planning_context, 15, i, (threshold - distance_for_repair)/gradientmap.D_crosswalk[i]);
                }
                if (std::abs(gradientmap.D_junction[i]) > 0.1){ //D_junction
                    FixTheTrajectory(planning_context, 16, i, (threshold - distance_for_repair)/gradientmap.D_junction[i]);
                }
                if (std::abs(gradientmap.lane_num[i]) > 0.1){ //lane_num
                    FixTheTrajectory(planning_context, 6, i, (threshold - distance_for_repair)/gradientmap.lane_num[i]);
                }
            }

            // number_of_fixes = number_of_fixes + 1;

            
            // AINFO << "SY Module - choosen_timestep: " << choosen_timestep << " original size: " << trace_sy.speed.size();
            // for (int i = 0; (unsigned)i < gradientmap.D_stopline.size(); i++){
            //     if (gradientmap.D_stopline[i] != 0){
            //         AINFO << "SY Module - The gradientmap D_stopline timestep: " << i << " the value: " << gradientmap.D_stopline[i];    
            //     }
            // }
        }
    }



    return;
}


void PlanningComponent::Prepare_Law38() {
    // Green Light
    GeneralSpec* a_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_0->LeftType = 12; //the signal type: TL(color)
    a_0->RightNumber = 1; //"TL(color) == green"
    //SpecsTodelete.push_back(a_0);

    GeneralSpec* a_1 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    a_1->LeftType = 14; //the signal type: D(stopline)
    a_1->RightNumber = 2; //"D(stopline) < 2"
    //SpecsTodelete.push_back(a_1);

    GeneralSpec* a_2 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    a_2->LeftType = 16; //the signal type: D(junction)
    a_2->RightNumber = 2; //"D(junction) < 2"
    //SpecsTodelete.push_back(a_2);

    GeneralSpec* a_3 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_3->LeftType = 10; //the signal type: PriorityVehicleAhead
    a_3->RightNumber = 0; //"PriorityVehicleAhead == false"
    //SpecsTodelete.push_back(a_3);

    GeneralSpec* a_4 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_4->LeftType = 11; //the signal type: PriorityPedAhead
    a_4->RightNumber = 0; //"PriorityPedAhead == false"
    //SpecsTodelete.push_back(a_4);

    GeneralSpec* a_5 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    a_5->LeftType = 1; //the signal type: speed
    a_5->RightNumber = 0.5; //"speed > 0.5"
    //SpecsTodelete.push_back(a_5);

    GeneralSpec* a_00 = new GeneralSpec(SPECTYPE_OR);
    a_00->LeftSpec = a_1;
    a_00->RightSpec = a_2; //"D(stopline) < 2 or D(junction) < 2"
    //SpecsTodelete.push_back(a_00);

    GeneralSpec* a_01 = new GeneralSpec(SPECTYPE_AND);
    a_01->LeftSpec = a_0;
    a_01->RightSpec = a_00; //"TL(color) == green and (D(junction) < 2 or D(stopline) < 2)"
    //SpecsTodelete.push_back(a_01);

    GeneralSpec* a_02 = new GeneralSpec(SPECTYPE_AND);
    a_02->LeftSpec = a_3;
    a_02->RightSpec = a_4; //"PriorityVehicleAhead == false and PriorityPedAhead == false"
    //SpecsTodelete.push_back(a_02);

    GeneralSpec* law38_sub1_1 = new GeneralSpec(SPECTYPE_AND);
    law38_sub1_1->LeftSpec = a_01;
    law38_sub1_1->RightSpec = a_02; //"TL(color) == green and (D(junction) < 2 or D(stopline) < 2) and PriorityVehicleAhead == false and PriorityPedAhead == false"
    //SpecsTodelete.push_back(law38_sub1_1);

    GeneralSpec* law38_sub1_2 = new GeneralSpec(SPECTYPE_FINALLY);
    law38_sub1_2->spec = a_5; //"F[0, 2] (speed > 0.5)"
    law38_sub1_2->whetherbounded = 1;
    law38_sub1_2->LeftNumber = 0;
    law38_sub1_2->RightNumber = 2;
    //SpecsTodelete.push_back(law38_sub1_2);

    GeneralSpec* a_03 = new GeneralSpec(SPECTYPE_IMPLY);
    a_03->LeftSpec = law38_sub1_1;
    a_03->RightSpec = law38_sub1_2; //"law38_sub1_1 -> law38_sub1_2"
    //SpecsTodelete.push_back(a_03);

    GeneralSpec* law38_sub1 = new GeneralSpec(SPECTYPE_ALWAYS);
    law38_sub1->spec = a_03; //"G (law38_sub1_1 -> law38_sub1_2)"
    //SpecsTodelete.push_back(law38_sub1);





    // Yellow Light
    GeneralSpec* b_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    b_0->LeftType = 12; //the signal type: TL(color)
    b_0->RightNumber = 0; //"TL(color) == yellow"
    //SpecsTodelete.push_back(b_0);

    GeneralSpec* b_1 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    b_1->LeftType = 6; //the signal type: lane(num)
    b_1->RightNumber = 0; //"lane(num) == 0"
    //SpecsTodelete.push_back(b_1);

    GeneralSpec* b_2 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    b_2->LeftType = 14; //the signal type: D(stopline)
    b_2->RightNumber = 3.5; //"D(stopline) < 3.5"
    //SpecsTodelete.push_back(b_2);

    GeneralSpec* b_3 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    b_3->LeftType = 1; //the signal type: speed
    b_3->RightNumber = 0.5; //"speed < 0.5"
    //SpecsTodelete.push_back(b_3);

    GeneralSpec* b_4 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    b_4->LeftType = 6; //the signal type: lane(num)
    b_4->RightNumber = 1; //"lane(num) >= 1"
    //SpecsTodelete.push_back(b_4);

    GeneralSpec* b_00 = new GeneralSpec(SPECTYPE_OR);
    b_00->LeftSpec = a_1;
    b_00->RightSpec = b_1; //"D(stopline) < 2 or lane(num) == 0"
    //SpecsTodelete.push_back(b_00);

    GeneralSpec* b_01 = new GeneralSpec(SPECTYPE_AND);
    b_01->LeftSpec = b_0;
    b_01->RightSpec = b_00; //"TL(color) == yellow and (D(stopline) < 2 or lane(num) == 0)"
    //SpecsTodelete.push_back(b_01);

    GeneralSpec* law38_sub2_1 = new GeneralSpec(SPECTYPE_IMPLY);
    law38_sub2_1->LeftSpec = b_01;
    law38_sub2_1->RightSpec = law38_sub1_2; //"(TL(color) == yellow and (D(stopline) < 2 or lane(num) == 0)) -> F[0, 2] (speed > 0.5)"
    //SpecsTodelete.push_back(law38_sub2_1);

    GeneralSpec* b_10 = new GeneralSpec(SPECTYPE_AND);
    b_10->LeftSpec = b_2;
    b_10->RightSpec = b_4; //"D(stopline) < 3.5 and lane(num) >= 1"
    //SpecsTodelete.push_back(b_10);

    GeneralSpec* b_11 = new GeneralSpec(SPECTYPE_AND);
    b_11->LeftSpec = b_0;
    b_11->RightSpec = b_10; //"(TL(color) == yellow and (D(stopline) < 3.5 and lane(num) >= 1)"
    //SpecsTodelete.push_back(b_11);

    GeneralSpec* b_12 = new GeneralSpec(SPECTYPE_FINALLY);
    b_12->spec = b_3; //"F[0, 3] (speed < 0.5)"
    b_12->whetherbounded = 1;
    b_12->LeftNumber = 0;
    b_12->RightNumber = 3;
    //SpecsTodelete.push_back(b_12);

    GeneralSpec* law38_sub2_2 = new GeneralSpec(SPECTYPE_IMPLY);
    law38_sub2_2->LeftSpec = b_11;
    law38_sub2_2->RightSpec = b_12; //"(TL(color) == yellow and (D(stopline) < 3.5 and lane(num) >= 1) -> F[0, 3] (speed < 0.5)"
    //SpecsTodelete.push_back(law38_sub2_2);

    GeneralSpec* b_13 = new GeneralSpec(SPECTYPE_AND);
    b_13->LeftSpec = law38_sub2_1;
    b_13->RightSpec = law38_sub2_2; //"law38_sub2_1 and law38_sub2_1"
    //SpecsTodelete.push_back(b_13);

    GeneralSpec* law38_sub2 = new GeneralSpec(SPECTYPE_ALWAYS);
    law38_sub2->spec = b_13; //"G (law38_sub2_1 and law38_sub2_1)"
    //SpecsTodelete.push_back(law38_sub2);

    // Red Light
    GeneralSpec* c_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    c_0->LeftType = 12; //the signal type: TL(color)
    c_0->RightNumber = 2; //"TL(color) == red"
    //SpecsTodelete.push_back(c_0);

    GeneralSpec* c_1 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    c_1->LeftType = 4; //the signal type: direction
    c_1->RightNumber = 2; //"direction == 2"
    //SpecsTodelete.push_back(c_1);

    GeneralSpec* c_2 = new GeneralSpec(SPECTYPE_ATOMDIFFERENT);
    c_2->LeftType = 4; //the signal type: direction
    c_2->RightNumber = 2; //"direction !== 2"
    //SpecsTodelete.push_back(c_2);

    GeneralSpec* c_00 = new GeneralSpec(SPECTYPE_AND);
    c_00->LeftSpec = c_0;
    c_00->RightSpec = a_00; //"TL(color) == red and (D(junction) < 2 or D(stopline) < 2)"
    //SpecsTodelete.push_back(c_00);

    GeneralSpec* c_01 = new GeneralSpec(SPECTYPE_AND);
    c_01->LeftSpec = c_00;
    c_01->RightSpec = c_2; //"TL(color) == red and (D(junction) < 2 or D(stopline) < 2) and direction !== 2"
    //SpecsTodelete.push_back(c_01);

    GeneralSpec* law38_sub3_1 = new GeneralSpec(SPECTYPE_IMPLY);
    law38_sub3_1->LeftSpec = c_01;
    law38_sub3_1->RightSpec = b_12; //"(TL(color) == red and (D(junction) < 2 or D(stopline) < 2) and direction !== 2) -> F[0, 3] (speed < 0.5)"
    //SpecsTodelete.push_back(law38_sub3_1);

    GeneralSpec* c_02 = new GeneralSpec(SPECTYPE_AND);
    c_02->LeftSpec = c_00;
    c_02->RightSpec = c_1; //"TL(color) == red and (D(junction) < 2 or D(stopline) < 2) and direction == 2"
    //SpecsTodelete.push_back(c_02);

    GeneralSpec* c_03 = new GeneralSpec(SPECTYPE_AND);
    c_03->LeftSpec = c_02;
    c_03->RightSpec = a_02; //"TL(color) == red and (D(junction) < 2 or D(stopline) < 2) and direction == 2 and PriorityVehicleAhead == false and PriorityPedAhead == false"
    //SpecsTodelete.push_back(c_03);

    GeneralSpec* law38_sub3_2 = new GeneralSpec(SPECTYPE_IMPLY);
    law38_sub3_2->LeftSpec = c_03;
    law38_sub3_2->RightSpec = law38_sub1_2; //"(TL(color) == red and (D(junction) < 2 or D(stopline) < 2) and direction !== 2) -> F[0, 2] (speed > 0.5)"
    //SpecsTodelete.push_back(law38_sub3_2);

    GeneralSpec* c_04 = new GeneralSpec(SPECTYPE_AND);
    c_04->LeftSpec = law38_sub3_1;
    c_04->RightSpec = law38_sub3_2; //"law38_sub3_1 and law38_sub3_2"
    //SpecsTodelete.push_back(c_04);

    GeneralSpec* law38_sub3 = new GeneralSpec(SPECTYPE_ALWAYS);
    law38_sub3->spec = c_04; //"G (law38_sub2_1 and law38_sub2_1)"
    //SpecsTodelete.push_back(law38_sub3);


    // law38
    GeneralSpec* temp0 = new GeneralSpec(SPECTYPE_AND);
    temp0->LeftSpec = law38_sub1;
    temp0->RightSpec = law38_sub2; //"law38_sub1 and law38_sub2"
    //SpecsTodelete.push_back(temp0);


    TheGeneralSpec->type = SPECTYPE_AND;
    TheGeneralSpec->LeftSpec = temp0;
    TheGeneralSpec->RightSpec = law38_sub3; //"law38_sub1 and law38_sub2 and law38_sub3"

    //TheGeneralSpec->type = SPECTYPE_ALWAYS;
    //TheGeneralSpec->spec = c_04;

    //delete a_0;

    //delete a_1;

    //delete a_2;

    //delete a_3;

    //delete a_4;

    //delete a_5;

    //delete a_00;

    //delete a_01;

    //delete a_02;

    //delete law38_sub1_1;

    //delete law38_sub1_2;

    //delete a_03;

    //delete law38_sub1;

    //delete b_0;

    //delete b_1;

    //delete b_2;

    //delete b_3;

    //delete b_4;

    //delete b_00;

    //delete b_01;

    //delete law38_sub2_1;

    //delete b_10;

    //delete b_11;

    //delete b_12;

    //delete law38_sub2_2;

    //delete b_13;

    //delete law38_sub2;

    //delete c_0;

    //delete c_1;

    //delete c_2;

    //delete c_00;

    //delete c_01;

    //delete law38_sub3_1;

    //delete c_02;

    //delete c_03;

    //delete law38_sub3_2;

    //delete c_04;

    //delete law38_sub3;
}

void PlanningComponent::Prepare_Law51_45() {
    // law51_sub4_1 = trafficLightAhead.color == green & (~NPCAhead(8) | (( NPCAhead(8) -> F[0,2](NPCAhead.speed > 0.5)  ) & NPCAhead(8)));
    // law51_sub4_2 = (F[0, 3](speed > 0.5)) & ~NPCAhead(0.5);
    // law51_sub4 = G(law51_sub4_1 -> law51_sub4_2);

    GeneralSpec* a_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_0->LeftType = 12; //the signal type: TL(color)
    a_0->RightNumber = 1; //"TL(color) == green"

    GeneralSpec* a_1 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    a_1->LeftType = 18; //the signal type: NPCAhead_Distance
    a_1->RightNumber = 8; //"NPCAhead_Distance < 8"

    GeneralSpec* a_2 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    a_2->LeftType = 16; //the signal type: NPCAhead_Speed
    a_2->RightNumber = 0.5; //"NPCAhead_Speed > 0.5"

    GeneralSpec* a_3 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    a_3->LeftType = 18; //the signal type: NPCAhead_Distance
    a_3->RightNumber = 8; //"NPCAhead_Distance > 8"


    GeneralSpec* a_4 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    a_4->LeftType = 1; //the signal type: speed
    a_4->RightNumber = 0.5; //"speed > 0.5"


    GeneralSpec* a_5 = new GeneralSpec(SPECTYPE_ATOMGREATER);
    a_5->LeftType = 18; //the signal type: NPCAhead_Distance
    a_5->RightNumber = 0.5; //"NPCAhead_Distance > 0.5"



    GeneralSpec* a_6 = new GeneralSpec(SPECTYPE_FINALLY);
    a_6->spec = a_2; //"F[0, 2] (NPCAhead_Speed > 0.5)"
    a_6->whetherbounded = 1;
    a_6->LeftNumber = 0;
    a_6->RightNumber = 2;


    GeneralSpec* a_7 = new GeneralSpec(SPECTYPE_IMPLY);
    a_7->LeftSpec = a_1;
    a_7->RightSpec = a_6; //"NPCAhead(8) -> F[0,2](NPCAhead.speed > 0.5)"


    GeneralSpec* a_8 = new GeneralSpec(SPECTYPE_AND);
    a_8->LeftSpec = a_7;
    a_8->RightSpec = a_1; //"(( NPCAhead(8) -> F[0,2](NPCAhead.speed > 0.5)  ) & NPCAhead(8))"


    GeneralSpec* a_9 = new GeneralSpec(SPECTYPE_OR);
    a_9->LeftSpec = a_3;
    a_9->RightSpec = a_8; //"(~NPCAhead(8) | (( NPCAhead(8) -> F[0,2](NPCAhead.speed > 0.5)  ) & NPCAhead(8)))"


    GeneralSpec* law51_sub4_1 = new GeneralSpec(SPECTYPE_AND);
    law51_sub4_1->LeftSpec = a_0;
    law51_sub4_1->RightSpec = a_9; //"law51_sub4_1"



    GeneralSpec* a_11 = new GeneralSpec(SPECTYPE_FINALLY);
    a_11->spec = a_4; //"F[0, 3] (speed > 0.5)"
    a_11->whetherbounded = 1;
    a_11->LeftNumber = 0;
    a_11->RightNumber = 3;

    GeneralSpec* law51_sub4_2 = new GeneralSpec(SPECTYPE_AND);
    law51_sub4_2->LeftSpec = a_11;
    law51_sub4_2->RightSpec = a_5; //"law51_sub4_2"


    GeneralSpec* a_12 = new GeneralSpec(SPECTYPE_IMPLY);
    a_12->LeftSpec = law51_sub4_1;
    a_12->RightSpec = law51_sub4_2; //"law51_sub4_1 -> law51_sub4_2"


    GeneralSpec* law51_sub4 = new GeneralSpec(SPECTYPE_ALWAYS);
    law51_sub4->spec = a_12;


    // law51_sub5_1 = trafficLightAhead.color == red & (stoplineAhead(2) | junctionAhead(2) | NPCAhead(0.5));
    // law51_sub5_2 = F[0, 2](speed < 0.5);
    // law51_sub5 = G(law51_sub5_1 -> law51_sub5_2);  

    GeneralSpec* c_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    c_0->LeftType = 12; //the signal type: TL(color)
    c_0->RightNumber = 2; //"TL(color) == red"

    GeneralSpec* c_1 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    c_1->LeftType = 18; //the signal type: NPCAhead_Distance
    c_1->RightNumber = 0.5; //"NPCAhead_Distance < 0.5"

    GeneralSpec* c_2 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    c_2->LeftType = 14; //the signal type: D(stopline)
    c_2->RightNumber = 2; //"D(stopline) < 2"

    GeneralSpec* c_3= new GeneralSpec(SPECTYPE_ATOMSMALLER);
    c_3->LeftType = 16; //the signal type: D(junction)
    c_3->RightNumber = 2; //"D(junction) < 2"

    GeneralSpec* c_4 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    c_4->LeftType = 1; //the signal type: speed
    c_4->RightNumber = 0.5; //"speed < 0.5"



    GeneralSpec* c_5 = new GeneralSpec(SPECTYPE_OR);
    c_5->LeftSpec = c_2;
    c_5->RightSpec = c_3; //"(stoplineAhead(2) | junctionAhead(2))"


    GeneralSpec* c_6 = new GeneralSpec(SPECTYPE_OR);
    c_6->LeftSpec = c_5;
    c_6->RightSpec = c_1; //"(stoplineAhead(2) | junctionAhead(2) | NPCAhead(0.5))"

    GeneralSpec* law51_sub5_1 = new GeneralSpec(SPECTYPE_AND);
    law51_sub5_1->LeftSpec = c_0;
    law51_sub5_1->RightSpec = c_6; //"law51_sub5_1"


    GeneralSpec* law51_sub5_2 = new GeneralSpec(SPECTYPE_FINALLY);
    law51_sub5_2->spec = c_4; //"F[0, 2] (speed < 0.5)"
    law51_sub5_2->whetherbounded = 1;
    law51_sub5_2->LeftNumber = 0;
    law51_sub5_2->RightNumber = 2;

    GeneralSpec* c_7 = new GeneralSpec(SPECTYPE_IMPLY);
    c_7->LeftSpec = law51_sub5_1;
    c_7->RightSpec = law51_sub5_2; //"law51_sub5_1 -> law51_sub5_2"


    GeneralSpec* law51_sub5 = new GeneralSpec(SPECTYPE_ALWAYS);
    law51_sub5->spec = c_7;


    TheGeneralSpec->type = SPECTYPE_AND;
    TheGeneralSpec->LeftSpec = law51_sub4;
    TheGeneralSpec->RightSpec = law51_sub5; //"law51_sub4 and law51_sub5"
}

void PlanningComponent::Prepare_Law46_2() {
    // law_46_sub2 = G( (direction == left | direction == right | isTurningAround) -> (speed <= 30 ));
    
    GeneralSpec* a_0 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_0->LeftType = 4; //the signal type: direction
    a_0->RightNumber = 1; //"direction == left "



    GeneralSpec* a_1 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_1->LeftType = 4; //the signal type: direction
    a_1->RightNumber = 2; //"direction == right "


    GeneralSpec* a_2 = new GeneralSpec(SPECTYPE_OR);
    a_2->LeftSpec = a_0;
    a_2->RightSpec = a_1; //"(direction == left | direction == right)"


    GeneralSpec* a_3 = new GeneralSpec(SPECTYPE_ATOMSMALLER);
    a_3->LeftType = 1; //the signal type: speed
    a_3->RightNumber = 30; //"speed < 30"


    GeneralSpec* a_4 = new GeneralSpec(SPECTYPE_IMPLY);
    a_4->LeftSpec = a_2;
    a_4->RightSpec = a_3; //"(direction == left | direction == right) -> (speed <= 30 )"

    TheGeneralSpec->type = SPECTYPE_ALWAYS;
    TheGeneralSpec->spec = a_4;
}


void PlanningComponent::Prepare_Law44() {
    //law44
    // law44_sub1 = currentLane.number >= 2;
    // law44_sub2 = (speed >= speedLimit.lowerLimit) & (speed <= speedLimit.upperLimit);
    // law44_sub3 = isChangingLane & currentLane.number >= 2;
    // law44_sub4 = G(law44_sub1 -> law44_sub2);
    // law44_sub5 = G(law44_sub3 -> ~PriorityNPCAhead);
    // law44 = law44_sub4 & law44_sub5;
    
    GeneralSpec* law44_sub1 = new GeneralSpec(SPECTYPE_ATOMGREATEREQUAL);
    law44_sub1->LeftType = 6; //the signal type: lane_num
    law44_sub1->RightNumber = 2; //"lane_num >= 2 "


    GeneralSpec* a_0 = new GeneralSpec(SPECTYPE_ATOMSMALLEREQUAL);
    a_0->LeftType = 1; //the signal type: speed
    a_0->RightNumber = 40; //"speed <= 40"


    GeneralSpec* a_1 = new GeneralSpec(SPECTYPE_ATOMGREATEREQUAL);
    a_1->LeftType = 1; //the signal type: speed
    a_1->RightNumber = 0; //"speed >= 0"


    GeneralSpec* law44_sub2 = new GeneralSpec(SPECTYPE_AND);
    law44_sub2->LeftSpec = a_0;
    law44_sub2->RightSpec = a_1; //"(speed >= speedLimit.lowerLimit) & (speed <= speedLimit.upperLimit)"


    GeneralSpec* a_2 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_2->LeftType = 44; //the signal type: isChangingLane
    a_2->RightNumber = 1; //"isChangingLane == true"


    GeneralSpec* law44_sub3 = new GeneralSpec(SPECTYPE_AND);
    law44_sub3->LeftSpec = a_2;
    law44_sub3->RightSpec = law44_sub1; //"isChangingLane & currentLane.number >= 2;"


    GeneralSpec* a_3 = new GeneralSpec(SPECTYPE_IMPLY);
    a_3->LeftSpec = law44_sub1;
    a_3->RightSpec = law44_sub2; //"(law44_sub1 -> law44_sub2)"


    GeneralSpec* law44_sub4 = new GeneralSpec(SPECTYPE_ALWAYS);
    law44_sub4->spec = a_3; //"G(law44_sub1 -> law44_sub2)"


    GeneralSpec* a_4 = new GeneralSpec(SPECTYPE_ATOMEQUAL);
    a_4->LeftType = 10; //the signal type: PriorityVehicleAhead
    a_4->RightNumber = 0; //"isChangingLane == false"


    GeneralSpec* a_5 = new GeneralSpec(SPECTYPE_IMPLY);
    a_5->LeftSpec = law44_sub3;
    a_5->RightSpec = a_4; //"(law44_sub3 -> ~PriorityNPCAhead)"


    GeneralSpec* law44_sub5 = new GeneralSpec(SPECTYPE_ALWAYS);
    law44_sub5->spec = a_5; //"G(law44_sub3 -> ~PriorityNPCAhead)"

    
    TheGeneralSpec->type = SPECTYPE_AND;
    TheGeneralSpec->LeftSpec = law44_sub4;
    TheGeneralSpec->RightSpec = law44_sub5; //"law44"
}

void PlanningComponent::PrepareSpec() {
    if (TheGeneralSpec->type == 0){
      // Prepare_Law38();
      // Prepare_Law51_45();
      // Prepare_Law46_2();
        Prepare_Law44();
        return;
    }
    else {
      AINFO << "SY Module - *****law already defined";
      return;
    }

    // Prepare the 'GeneralSpec' here
    // 0 - null 
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
}


double PlanningComponent::GetSingalByTypeByTimestep(int type, int t) {
    double result = 0;
    if (type == 0) {
        return result;
    }
    else if (type == 1) {
        return trace_sy.speed[t];
    }
    else if (type == 2) {
        return trace_sy.acceleration[t];
    }
    else if (type == 3) {
        return trace_sy.relativetime[t];
    }
    else if (type == 4) {
        return trace_sy.direction[t];
    }
    else if (type == 5) {
        return trace_sy.heading[t];
    }
    else if (type == 6) {
        return trace_sy.lane_num[t];
    }
    else if (type == 7) {
        return trace_sy.lane_direction[t];
    }
    else if (type == 8) {
        return trace_sy.lane_side[t];
    }
    else if (type == 9) {
        return trace_sy.gear[t];
    }
    else if (type == 10) {
        return trace_sy.PriorityVehicleAhead[t];
    }
    else if (type == 11) {
        return trace_sy.PriorityPedAhead[t];
    }
    else if (type == 12) {
        return trace_sy.TL_color[t];
    }
    else if (type == 13) {
        return trace_sy.TL_isblinking[t];
    }
    else if (type == 14) {
        return trace_sy.D_stopline[t];
    }
    else if (type == 15) {
        return trace_sy.D_crosswalk[t];
    }
    else if (type == 16) {
        return trace_sy.D_junction[t];
    }
    else if (type == 17) {
        return trace_sy.NPCAhead_Speed[t];
    }
    else if (type == 18) {
        return trace_sy.NPCAhead_Distance[t];
    }
    else if (type == 19) {
        return trace_sy.NPCAhead_Dirtection[t];
    }
    else if (type == 20) {
        return trace_sy.NPCAhead_type[t];
    }
    else if (type == 29) {
        return trace_sy.NPCNearest_Speed[t];
    }
    else if (type == 30) {
        return trace_sy.NPCNearest_Distance[t];
    }
    else if (type == 31) {
        return trace_sy.NPCNearest_Dirtection[t];
    }
    else if (type == 32) {
        return trace_sy.NPCNearest_type[t];
    }
    else if (type == 37) {
        return trace_sy.engineOn[t];
    }
    else if (type == 38) {
        return trace_sy.highBeamOn[t];
    }
    else if (type == 39) {
        return trace_sy.lowBeamOn[t];
    }
    else if (type == 40) {
        return trace_sy.turnSignal[t];
    }
    else if (type == 41) {
        return trace_sy.fogLightOn[t];
    }
    else if (type == 42) {
        return trace_sy.warningFlashOn[t];
    }
    else if (type == 43) {
        return trace_sy.toManual[t];
    }
    else if (type == 44) {
        return trace_sy.isChangingLane[t];
    }
    else if (type == 45) {
        return trace_sy.isOvertaking[t];
    }
    else if (type == 46) {
        return trace_sy.isTurningAround[t];
    }
    else {
        AINFO << "UNKNOWN SIGNAL TYPE: " << type;
    }
    return result;
}

double PlanningComponent::CalculationOfDistanceByTimeStep(GeneralSpec* spec0, int t, int L) {
    double result = 0;
    assert(t < L);

    if (spec0->type == 1) { //not
        result = -1 * CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), t, L);

        //StoredRobustness temp;
        ////temp.layertype = 1;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 2) { //and 
        //std::cout << "Distance type2: and \n";

        auto s1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto s2 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);

        result = std::min(s1, s2);

        //StoredRobustness temp;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 3) { //or
        //std::cout << "Distance type3: or \n";
        auto s1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto s2 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);

        result = std::max(s1, s2);

        //StoredRobustness temp;
        ////temp.layertype = 1;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 4) { //imply
        //std::cout << "Distance type4: imply \n";  

        auto s1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto s2 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);


        

        result = std::max(-1 * s1, s2);

        //StoredRobustness temp;
        ////temp.layertype = 1;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 5) { //always
        //AINFO << "SY Module - Calculation Distance Always";
        //std::cout << "Distance type5: always \n";
        //auto s0 = CalculationOfDistance(dynamic_cast<GeneralSpec*>(spec0->spec));
        if (spec0->whetherbounded == 0) {
            //AINFO << "SY Module - Distance type6: always";
            std::vector<double> temp;
            for (int a = t; a < L; a = a + 1) {
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), a, L));
            }
            result = *min_element(temp.begin(), temp.end());
        }
        else if (spec0->whetherbounded == 1) {
            //std::cout << "Distance type6: always [left, right] \n";
            //AINFO << "SY Module - Distance type6: always[0,1]";
            std::vector<double> temp;
            result = 10000;
            double LeftNumber = spec0->LeftNumber;
            double RightNumber = spec0->RightNumber;

            double the_lower = trace_sy.relativetime[t] + LeftNumber;
            double the_upper = trace_sy.relativetime[t] + RightNumber;

            bool flag_0 = false;
            bool flag_1 = false;

            int low_bound = 0;
            int upper_bound = L;

            for (int jjj = 0; jjj < L; jjj++) {
                if (trace_sy.relativetime[jjj] >= the_lower && flag_0 == false) {
                    flag_0 = true;
                    low_bound = jjj;
                }

                if (trace_sy.relativetime[jjj] > the_upper && flag_1 == false) {
                    flag_1 = true;
                    upper_bound = jjj;
                }
            }
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                result = std::min(result, CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
            }
        }
        else {
            std::cout << "SY Module - Unexpected type of whetherbounded";
        }

        //StoredRobustness temp;
        ////temp.layertype = 1;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 6) { //finally
        //std::cout << "type6: finally \n";
        if (spec0->whetherbounded == 0) {
            //AINFO << "SY Module - Distance type6: finally";
            std::vector<double> temp;
            for (int i = t; i < L; i = i + 1) {
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), i, L));
            }
            result = *max_element(temp.begin(), temp.end());
        }
        else if (spec0->whetherbounded == 1) {
            //std::cout << "Distance type6: finally [left, right] \n";
            //AINFO << "SY Module - Distance type6: finally[0,1]";
            std::vector<double> temp;
            result = -10000;
            double LeftNumber = spec0->LeftNumber;
            double RightNumber = spec0->RightNumber;

            double the_lower = trace_sy.relativetime[t] + LeftNumber;
            double the_upper = trace_sy.relativetime[t] + RightNumber;

            bool flag_0 = false;
            bool flag_1 = false;

            int low_bound = 0;
            int upper_bound = L;

            for (int jjj = 0; jjj < L; jjj++) {
                if (trace_sy.relativetime[jjj] >= the_lower && flag_0 == false) {
                    flag_0 = true;
                    low_bound = jjj;
                }

                if (trace_sy.relativetime[jjj] > the_upper && flag_1 == false) {
                    flag_1 = true;
                    upper_bound = jjj;
                }
            }
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                result = std::max(result, CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
            }
        }
        else {
            std::cout << "SY Module - Unexpected type of whetherbounded";
        }

        //StoredRobustness temp;
        ////temp.layertype = 1;
        //temp.value = result;
        //temp.timestep = t;
        //temp.formula = spec0;
        //storedrobustness.push_back(temp);

        return result;
    }
    else if (spec0->type == 7) { //until
        //std::cout << "Distance type7: until \n";
        //auto s1 = CalculationOfDistance(dynamic_cast<GeneralSpec*>(spec0->LeftSpec));
        //auto s2 = CalculationOfDistance(dynamic_cast<GeneralSpec*>(spec0->RightSpec));
        //result = MergeSpec(s1, s2, 7);

        //GradientMemory temp;
        ////temp.layertype = 1;
        //temp.values.push_back(s1);
        //temp.values.push_back(s2);
        //temp.spec = spec0;
        //gradient_memory.push_back(temp);
        return result;
    }
    else if (spec0->type == 8) { //next
        //std::cout << "Distance type8: next \n";
        //auto s0 = CalculationOfDistance(dynamic_cast<GeneralSpec*>(spec0->spec));
        //result = MergeSpec(s0, {}, 8);

        //GradientMemory temp;
        ////temp.layertype = 1;
        //temp.values.push_back(s0);
        //temp.spec = spec0;
        //gradient_memory.push_back(temp);

        return result;
    }
    else if (spec0->type == 10) { //atomstep - "<"
        //std::cout << "Distance type10: < <= \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = s1 - s0;
        return result;
    }
    else if (spec0->type == 11) { //atomstep - ">" 
        //std::cout << "Distance type11: > >= \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = s0 - s1;
        return result;
    }
    else if (spec0->type == 12) { //atomstep - "==" 
        //std::cout << "Distance type12 == \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = -1 * std::abs(s0 - s1);
        return result;
    }
    else if (spec0->type == 13) { //atomstep - "!="
        //std::cout << "Distance type13 != \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = std::abs(s0 - s1);
        return result;
    }
    else if (spec0->type == 14) { //atomstep - "<="
        //std::cout << "Distance type10: < <= \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = s1 - s0;
        return result;
    }
    else if (spec0->type == 15) { //atomstep - ">=" 
        //std::cout << "Distance type11: > >= \n";
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        auto s1 = spec0->RightNumber;

        result = s0 - s1;
        return result;
    }
    //std::cout << "s1 = { ";
    //for (double n : s1) {
    //    std::cout << n << ", ";
    //}
    //std::cout << "}; \n";

    // Prepare the 'GeneralSpec' here
    // 0 - atomstep 
    // 1 - not
    // 2 - and 
    // 3 - or
    // 4 - imply
    // 5 - always
    // 6 - finally
    // 7 - until
    // 8 - next

    // 10 - atomstep - "<"
    // 11 - atomstep - ">"
    // 12 - atomstep - "==" 
    // 13 - atomstep - "!="
    // 14 - atomstep - "<="
    // 15 - atomstep - ">="

    return result;
}

void PlanningComponent::CalculateGradient(GeneralSpec* spec0, int t, int L, double previousgradient) {
    if (spec0->type == 1) { //not
        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->spec), t, L, -1 * previousgradient);

        return;
    }
    else if (spec0->type == 2) { //and 
        auto x0 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto x1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);

        long double g_1 = std::exp(-k_for_soft * x0) / (std::exp(-k_for_soft * x0) + std::exp(-k_for_soft * x1));
        long double g_2 = std::exp(-k_for_soft * x1) / (std::exp(-k_for_soft * x0) + std::exp(-k_for_soft * x1));

        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L, previousgradient * g_1);
        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L, previousgradient * g_2);
        return;
    }
    else if (spec0->type == 3) { //or
        auto x0 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto x1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);

        long double g_1 = std::exp(k_for_soft * x0) / (std::exp(k_for_soft * x0) + std::exp(k_for_soft * x1));
        long double g_2 = std::exp(k_for_soft * x1) / (std::exp(k_for_soft * x0) + std::exp(k_for_soft * x1));

        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L, previousgradient * g_1);
        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L, previousgradient * g_2);
        return;
    }
    else if (spec0->type == 4) { //imply
        auto x0 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L);
        auto x1 = CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L);

        long double g_1 = std::exp(-k_for_soft * x0) / (std::exp(-k_for_soft * x0) + std::exp(k_for_soft * x1));
        long double g_2 = std::exp(k_for_soft * x1) / (std::exp(-k_for_soft * x0) + std::exp(k_for_soft * x1));

        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->LeftSpec), t, L, -previousgradient * g_1);
        CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->RightSpec), t, L, previousgradient * g_2);
        return;
    }
    else if (spec0->type == 5) { //always
        if (spec0->whetherbounded == 0) {
            std::vector<double> temp;
            long double sum = 0;
            for (int a = t; a < L; a = a + 1) {
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), a, L));  
                sum = sum + std::exp(-k_for_soft * temp[a - t]);
            }

            for (int a = t; a < L; a = a + 1) {
                long double g_1 = std::exp(-k_for_soft * temp[a - t]) / sum;
                CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->spec), a, L, previousgradient * g_1);
            }
        }
        else if (spec0->whetherbounded == 1) {
            std::vector<double> temp;
            //result = 10000;
            double LeftNumber = spec0->LeftNumber;
            double RightNumber = spec0->RightNumber;

            double the_lower = trace_sy.relativetime[t] + LeftNumber;
            double the_upper = trace_sy.relativetime[t] + RightNumber;

            bool flag_0 = false;
            bool flag_1 = false;

            int low_bound = 0;
            int upper_bound = L;

            for (int jjj = 0; jjj < L; jjj++) {
                if (trace_sy.relativetime[jjj] >= the_lower && flag_0 == false) {
                    flag_0 = true;
                    low_bound = jjj;
                }

                if (trace_sy.relativetime[jjj] > the_upper && flag_1 == false) {
                    flag_1 = true;
                    upper_bound = jjj;
                }
            }
            long double sum = 0;
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                //result = std::min(result, CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
                sum = sum + std::exp(-k_for_soft * temp.back());
            }
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                long double g_1 = std::exp(-k_for_soft * temp[j- std::max(low_bound, 0)]) / sum;
                CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->spec), j, L, previousgradient * g_1);
            }
        }
        else {
            std::cout << "SY Module - Unexpected type of whetherbounded";
        }

        return;

    }
    else if (spec0->type == 6) { //finally
        if (spec0->whetherbounded == 0) {
            //AINFO << "SY Module - Distance type6: always";
            std::vector<double> temp;
            long double sum = 0;
            for (int a = t; a < L; a = a + 1) {
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), a, L));
                sum = sum + std::exp(k_for_soft * temp[a]);
            }

            for (int a = t; a < L; a = a + 1) {
                long double g_1 = std::exp(k_for_soft * temp[a]) / sum;
                CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->spec), a, L, previousgradient * g_1);
            }
            //result = *min_element(temp.begin(), temp.end());
        }
        else if (spec0->whetherbounded == 1) {
            //std::cout << "Distance type6: always [left, right] \n";
            //AINFO << "SY Module - Distance type6: always[0,1]";
            std::vector<double> temp;
            //result = 10000;
            double LeftNumber = spec0->LeftNumber;
            double RightNumber = spec0->RightNumber;

            double the_lower = trace_sy.relativetime[t] + LeftNumber;
            double the_upper = trace_sy.relativetime[t] + RightNumber;

            bool flag_0 = false;
            bool flag_1 = false;

            int low_bound = 0;
            int upper_bound = L;

            for (int jjj = 0; jjj < L; jjj++) {
                if (trace_sy.relativetime[jjj] >= the_lower && flag_0 == false) {
                    flag_0 = true;
                    low_bound = jjj;
                }

                if (trace_sy.relativetime[jjj] > the_upper && flag_1 == false) {
                    flag_1 = true;
                    upper_bound = jjj;
                }
            }
            long double sum = 0;
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                //result = std::min(result, CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
                temp.push_back(CalculationOfDistanceByTimeStep(dynamic_cast<GeneralSpec*>(spec0->spec), j, L));
                sum = sum + std::exp(k_for_soft * temp[j - std::max(low_bound, 0)]);
            }
            for (int j = std::max(low_bound, 0); j < std::min(upper_bound, L); j++) {
                long double g_1 = std::exp(k_for_soft * temp[j - std::max(low_bound, 0)]) / sum;
                CalculateGradient(dynamic_cast<GeneralSpec*>(spec0->spec), j, L, previousgradient * g_1);
            }
        }
        else {
            std::cout << "SY Module - Unexpected type of whetherbounded";
        }

        return;
    }
    else if (spec0->type == 7) { //until
        return;
    }
    else if (spec0->type == 8) { //next
        return;

    }
    else if (spec0->type == 10) { //atomstep - "<" or "<="
        if (spec0->LeftType == 1) { //speed
            gradientmap.speed[t] = gradientmap.speed[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 14) { //D_stopline
            gradientmap.D_stopline[t] = gradientmap.D_stopline[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 15) { //D_crosswalk
            gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 16) { //D_junction
            gradientmap.D_junction[t] = gradientmap.D_junction[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 6) { //lane_num
            gradientmap.lane_num[t] = gradientmap.lane_num[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        return;
    }
    else if (spec0->type == 11) { //atomstep - ">" or ">="
        if (spec0->LeftType == 1) { //speed
            gradientmap.speed[t] = gradientmap.speed[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 14) { //D_stopline
            gradientmap.D_stopline[t] = gradientmap.D_stopline[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 15) { //D_crosswalk
            gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 16) { //D_junction
            gradientmap.D_junction[t] = gradientmap.D_junction[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 6) { //lane_num
            gradientmap.lane_num[t] = gradientmap.lane_num[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        return;
    }
    else if (spec0->type == 12) { //atomstep - "==" 
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        if (s0 > spec0->RightNumber) {
            if (spec0->LeftType == 1) { //speed
                gradientmap.speed[t] = gradientmap.speed[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 14) { //D_stopline
                gradientmap.D_stopline[t] = gradientmap.D_stopline[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 15) { //D_crosswalk
                gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 16) { //D_junction
                gradientmap.D_junction[t] = gradientmap.D_junction[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 6) { //lane_num
                gradientmap.lane_num[t] = gradientmap.lane_num[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
        }
        else {
            if (spec0->LeftType == 1) { //speed
                gradientmap.speed[t] = gradientmap.speed[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 14) { //D_stopline
                gradientmap.D_stopline[t] = gradientmap.D_stopline[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 15) { //D_crosswalk
                gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 16) { //D_junction
                gradientmap.D_junction[t] = gradientmap.D_junction[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 6) { //lane_num
                gradientmap.lane_num[t] = gradientmap.lane_num[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }   
        }
        return;
    }
    else if (spec0->type == 13) { //atomstep - "!="
        auto s0 = GetSingalByTypeByTimestep(spec0->LeftType, t);
        if (s0 > spec0->RightNumber) {
            if (spec0->LeftType == 1) { //speed
                gradientmap.speed[t] = gradientmap.speed[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 14) { //D_stopline
                gradientmap.D_stopline[t] = gradientmap.D_stopline[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 15) { //D_crosswalk
                gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 16) { //D_junction
                gradientmap.D_junction[t] = gradientmap.D_junction[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 6) { //lane_num
                gradientmap.lane_num[t] = gradientmap.lane_num[t] + previousgradient;
                // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
            }
        }
        else {
            if (spec0->LeftType == 1) { //speed
                gradientmap.speed[t] = gradientmap.speed[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 14) { //D_stopline
                gradientmap.D_stopline[t] = gradientmap.D_stopline[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 15) { //D_crosswalk
                gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 16) { //D_junction
                gradientmap.D_junction[t] = gradientmap.D_junction[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
            if (spec0->LeftType == 6) { //lane_num
                gradientmap.lane_num[t] = gradientmap.lane_num[t] - previousgradient;
                // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
            }
        }
        return;
    }
    else if (spec0->type == 14) { //atomstep - "<" or "<="
        if (spec0->LeftType == 1) { //speed
            gradientmap.speed[t] = gradientmap.speed[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 14) { //D_stopline
            gradientmap.D_stopline[t] = gradientmap.D_stopline[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 15) { //D_crosswalk
            gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 16) { //D_junction
            gradientmap.D_junction[t] = gradientmap.D_junction[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 6) { //lane_num
            gradientmap.lane_num[t] = gradientmap.lane_num[t] - previousgradient;
            // std::cout << "value: " << -previousgradient << "    timestep: " << t << "\n";
        }
        return;
    }
    else if (spec0->type == 15) { //atomstep - ">" or ">="
        if (spec0->LeftType == 1) { //speed
            gradientmap.speed[t] = gradientmap.speed[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 14) { //D_stopline
            gradientmap.D_stopline[t] = gradientmap.D_stopline[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 15) { //D_crosswalk
            gradientmap.D_crosswalk[t] = gradientmap.D_crosswalk[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 16) { //D_junction
            gradientmap.D_junction[t] = gradientmap.D_junction[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        if (spec0->LeftType == 6) { //lane_num
            gradientmap.lane_num[t] = gradientmap.lane_num[t] + previousgradient;
            // std::cout << "value: " << previousgradient << "    timestep: " << t << "\n";
        }
        return;
    }
     //Prepare the 'GeneralSpec' here
     //0 - atomstep 
     //1 - not
     //2 - and 
     //3 - or
     //4 - imply
     //5 - always
     //6 - finally
     //7 - until
     //8 - next

     //10 - atomstep - "<" or "<="
     //11 - atomstep - ">" or ">="
     //12 - atomstep - "==" 
     //13 - atomstep - "!="
    return;
}

int PlanningComponent::BinarySearchForTimestep(int minL, int maxL) {
    int result = maxL;
    int middleL = (minL + maxL) / 2;
    double temp = CalculationOfDistanceByTimeStep(TheGeneralSpec, 0, middleL);

    if (temp <= threshold) {
        if (middleL == minL) {
            result = middleL;
            distance_for_repair = temp;
            return result;
        }
        else {
            result = BinarySearchForTimestep(minL, middleL);
        }      
    }
    else {
        result = BinarySearchForTimestep(middleL + 1, maxL);
    }
    return result;
}

void PlanningComponent::FixTheTrajectory(apollo::planning::ADCTrajectory *planning_context, int type, int timestep, double modification_value) {
    auto chosen_trajectory_point = planning_context->mutable_trajectory_point(timestep);
    if (type == 1){ // speed
        // AINFO << "SY Module - **********Fix speed with value: " << modification_value;
        if (planning_context->trajectory_point(timestep + past_ones + 1).v() + modification_value/3.6 > 0){
            chosen_trajectory_point->set_v(planning_context->trajectory_point(timestep+ past_ones + 1).v() + modification_value/3.6);
        }
        else{
            chosen_trajectory_point->set_v(0.01);
        }     
    }
    else if (type == 4){ // direction
        // AINFO << "SY Module - **********I prefer not to set direction ";
        // AINFO << "SY Module - **********Fix direction with value: " << modification_value;
        double estimated = trace_sy.direction[timestep] + modification_value;

        double to_0 = std::abs(estimated - 0);
        double to_1 = std::abs(estimated - 1);
        double to_2 = std::abs(estimated - 2);

        if (to_0 <= to_1 && to_0 <= to_2){
            if (timestep != 0){
                chosen_trajectory_point->mutable_path_point()->set_theta(planning_context->trajectory_point(timestep-1 + past_ones + 1).path_point().theta());
            }             
        }
        else if (to_1 < to_0 && to_1 < to_2){
            if (timestep != 0){
                chosen_trajectory_point->mutable_path_point()->set_theta(planning_context->trajectory_point(timestep-1 + past_ones + 1).path_point().theta() + 0.05);
            }             
        }
        else if (to_2 < to_0 && to_2 < to_1){
            if (timestep != 0){
                chosen_trajectory_point->mutable_path_point()->set_theta(planning_context->trajectory_point(timestep-1 + past_ones + 1).path_point().theta() - 0.05);
            }             
        }    
    }
    else if (type == 6){ // lane_num
        // AINFO << "SY Module - **********Fix lane_num with value: " << modification_value;
        if (trace_sy.lane_num[timestep] == 0) {
            double temp_t = timestep - 1;
            while (temp_t >= 0){
                if (trace_sy.lane_num[temp_t] > 0){
                    double new_x = trace_sy.position_x[temp_t];
                    double new_y = trace_sy.position_y[temp_t];
                    chosen_trajectory_point->mutable_path_point()->set_x(new_x);
                    chosen_trajectory_point->mutable_path_point()->set_y(new_y);
                    break;
                }
                temp_t = temp_t - 1;
            }
        }
    }
    else if (type == 7){ // lane_direction           
    }
    else if (type == 8){ // lane_side            
    }
    else if (type == 14){ // D_stopline
        // if (modification_value < 0){
        // AINFO << "SY Module - **********Fix D_stopline with value: " << modification_value;
        if (timestep + 1 + past_ones + 1 <  planning_context->trajectory_point_size()){
            double d0 = trace_sy.D_stopline[timestep];
            double d1 = trace_sy.D_stopline[timestep+1];

            double d2 = d0 + modification_value;

            double x1 = trace_sy.position_x[timestep + 1];
            double x0 = trace_sy.position_x[timestep];
            double y1 = trace_sy.position_y[timestep + 1];
            double y0 = trace_sy.position_y[timestep];                   

            double new_x = x0 + (d2 - d0)* (x1-x0)/(d1-d0);
            double new_y = y0 + (d2 - d0)* (y1-y0)/(d1-d0);
            chosen_trajectory_point->mutable_path_point()->set_x(new_x);
            chosen_trajectory_point->mutable_path_point()->set_y(new_y);
        }
        // }
    }
    else if (type == 15){ // D_crosswalk
        // AINFO << "SY Module - **********Fix D_crosswalk with value: " << modification_value;
        if (timestep + 1 + past_ones + 1 <  planning_context->trajectory_point_size()){
            double d0 = trace_sy.D_crosswalk[timestep];
            double d1 = trace_sy.D_crosswalk[timestep+1];

            double d2 = d0 + modification_value;

            double x1 = trace_sy.position_x[timestep + 1];
            double x0 = trace_sy.position_x[timestep];
            double y1 = trace_sy.position_y[timestep + 1];
            double y0 = trace_sy.position_y[timestep];                   

            double new_x = x0 + (d2 - d0)* (x1-x0)/(d1-d0);
            double new_y = y0 + (d2 - d0)* (y1-y0)/(d1-d0);
            chosen_trajectory_point->mutable_path_point()->set_x(new_x);
            chosen_trajectory_point->mutable_path_point()->set_y(new_y);
        }
    }
    else if (type == 16){ // D_junction
        // AINFO << "SY Module - **********Fix D_junction with value: " << modification_value;
        if (timestep + 1 + past_ones + 1 <  planning_context->trajectory_point_size()){
            double d0 = trace_sy.D_junction[timestep];
            double d1 = trace_sy.D_junction[timestep+1];

            double d2 = d0 + modification_value;

            double x1 = trace_sy.position_x[timestep + 1];
            double x0 = trace_sy.position_x[timestep];
            double y1 = trace_sy.position_y[timestep + 1];
            double y0 = trace_sy.position_y[timestep];                   

            double new_x = x0 + (d2 - d0)* (x1-x0)/(d1-d0);
            double new_y = y0 + (d2 - d0)* (y1-y0)/(d1-d0);
            chosen_trajectory_point->mutable_path_point()->set_x(new_x);
            chosen_trajectory_point->mutable_path_point()->set_y(new_y);
        }
    }


    // AINFO << "SY Module - **********Speed Value After: " << planning_context->trajectory_point(timestep).v() * 3.6;
   

    // A test of the effects on actual control commands
    // for (int i = 0; i < planning_context->trajectory_point_size(); i++) {
    //     const auto every_trajectory_point = planning_context->mutable_trajectory_point(i);
    //     // // Try directly set v
    //     AINFO << "SY Module - !!!Speed Before: " << planning_context->trajectory_point(i).v();  
    //     if (i >= 10 ){
    //         every_trajectory_point->set_v(planning_context->trajectory_point(i).v() + 10);
    //     } 
    //     AINFO << "SY Module - !!!Speed After: " << planning_context->trajectory_point(i).v();  

    //     // // Try directly set x
    //     // AINFO << "SY Module - !!!X Before: " << planning_context->trajectory_point(i).path_point().x();  
    //     // every_trajectory_point->mutable_path_point()->set_x(planning_context->trajectory_point(i).path_point().x() + 1);
    //     // AINFO << "SY Module - !!!X After: " << planning_context->trajectory_point(i).path_point().x();  


    //     // // Try directly set a
    //     AINFO << "SY Module - !!!Acc Before: " << planning_context->trajectory_point(i).a();  
    //     if (i == 9 ){
    //         every_trajectory_point->set_a(planning_context->trajectory_point(i).a() + 10/ (planning_context->trajectory_point(i+1).relative_time() -  planning_context->trajectory_point(i).relative_time()) );
    //     }
    //     AINFO << "SY Module - !!!Acc After: " << planning_context->trajectory_point(i).a();  
    // }
    return;
}


}  // namespace planning
}  // namespace apollo