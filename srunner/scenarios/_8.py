#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Cut in scenario:

The scenario realizes a driving behavior on the highway.
The user-controlled ego vehicle is driving straight and keeping its velocity at a constant level.
Another car is cutting just in front, coming from left or right lane.

The ego vehicle may need to brake to avoid a collision.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      ActorDestroy,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      ChangeActorTargetSpeed
                                                                       
                                                                      )
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                                InTriggerDistanceToNextIntersection,
                                                                                RelativeVelocityToOtherActor,
                                                                                  DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class _8(BasicScenario):

    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._velocity = 35
        self._delta_velocity = 10
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._other_actor_stop_in_front_intersection = 10
        point = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(point)
        super(_8, self).__init__("_8",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)

    
    def _initialize_actors(self, config):

        # Spawn actor on other lane


        # transform visible
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        
        waypoint_left = waypoint.get_left_lane()
        transform = waypoint_left.transform
        transform.location.z += 0.5
        transform.location.x += 20
        tranform_ped = waypoint.transform
        tranform_ped.location.z += 0.5
        tranform_ped.location.x -= 5
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform)

        self.other_actors.append(first_vehicle)
        self.adversary = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', tranform_ped)
        self.other_actors.append(self.adversary)
    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """

        # car_visible
          # let the other actor drive until next intersection
        DriveStraight = py_trees.composites.Parallel(
            "DriveStraight",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        DriveStraight.add_child(WaypointFollower(self.other_actors[0], 15))
        DriveStraight.add_child(WaypointFollower(self.other_actors[1], 2))
        DriveStraight.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))
        DriveStraight.add_child(RelativeVelocityToOtherActor(self.ego_vehicles[0],
            self.adversary, 20))
        #DriveStraight.add_child(ChangeActorTargetSpeed(self.other_actors[0],20))
        
        
        # stop vehicle
        # stop = StopVehicle(self.other_actors[0], 1.0)

        # end condition
        end = py_trees.composites.Parallel("ego reached intersection",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        end.add_child(InTriggerDistanceToNextIntersection(
            self.ego_vehicles[0], 1))
        # end condition
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(DriveStraight)
        sequence.add_child(end)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence
    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria
    
    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
