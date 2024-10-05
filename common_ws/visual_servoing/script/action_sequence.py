#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import Enum
from action import Action
import time
import rclpy
ParkingBodyCameraSequence = Enum( 'ParkingBodyCameraSequence', \
                        'init_fork \
                        changing_direction \
                        move_nearby_parking_lot \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
ParkingForkCameraSequence = Enum( 'ParkingForkCameraSequence', \
                        'init_fork \
                        changing_direction \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
RaisePalletSequence = Enum( 'RaisePalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
DropPalletSequence = Enum( 'DropPalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
FruitSequence = Enum( 'FruitSequence', \
                        'fork_updown \
                        dead_reckoning \
                        dead_reckoning_x \
                        back \
                        stop \
                        error')
class ActionSequence():
    def __init__(self, VisualServoingActionServer):
        self.visual_servoing_action_server = VisualServoingActionServer  # access function {SpinOnce(), SpinOnce_Fork()} and parameter and Node{ROS2 API}
        self.action = Action(VisualServoingActionServer)
        self.is_sequence_finished = False

    def parking_bodycamera(self, goal_handle, layer):
        current_sequence = ParkingBodyCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingBodyCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingBodyCameraSequence.init_fork.value):
                self.is_sequence_finished = self.action.fnForkUpdown()
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changing_direction.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.changing_direction.value):
                self.is_sequence_finished = self.action.fnSeqChangingDirection(self.visual_servoing_action_server.bodycamera_ChangingDirection_threshold)

                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value):
                self.is_sequence_finished = self.action.fnSeqMovingNearbyParkingLot(self.visual_servoing_action_server.bodycamera_desired_dist_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.parking.value):
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.bodycamera_parking_stop, 2.0)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.bodycamera_Changingtheta_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.bodycamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingBodyCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.bodycamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingBodyCameraSequence.stop.value):
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
            
    def parking_forkcamera(self, goal_handle, layer):
        current_sequence = ParkingForkCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingForkCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingForkCameraSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingForkCameraSequence.parking.value):
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.forkcamera_parking_stop, 1.0)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.forkcamera_Changingtheta_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.forkcamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingForkCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.forkcamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingForkCameraSequence.stop.value):
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return

    def raise_pallet(self, goal_handle, layer):
        current_sequence = RaisePalletSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(RaisePalletSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == RaisePalletSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer1)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.fork_updown.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.back.value):
                self.is_sequence_finished = self.action.fnseqDeadReckoning(self.visual_servoing_action_server.raise_pallet_back_dist)

                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
              
    def drop_pallet(self, goal_handle, layer):
        current_sequence = DropPalletSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)

            if(current_sequence == DropPalletSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.dead_reckoning.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_dead_reckoning_dist)
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.fork_updown.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_drop_height_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_drop_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_back_distance)

                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return

    def fruit_docking(self, goal_handle):  #製作一個用於水果對接的函數
        current_sequence = FruitSequence.dead_reckoning_x.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            self.visual_servoing_action_server.get_logger().info(f"Current Sequence: {current_sequence}")

            # if current_sequence == FruitSequence.dead_reckoning.value:
            #     self.is_sequence_finished = self.action.fnForkFruit(self.visual_servoing_action_server.forkcamera_x_pose_hreshold)
            #     if self.is_sequence_finished:
            #         current_sequence = FruitSequence.dead_reckoning_x.value
            #         self.is_sequence_finished = False

            if current_sequence == FruitSequence.dead_reckoning_x.value:
                self.is_sequence_finished = self.action.fnForkFruit_approach(self.visual_servoing_action_server.fruit_dead_reckoning_dist_x)
                if self.is_sequence_finished:
                    return

            # else:
            #     self.visual_servoing_action_server.get_logger().info(f"Error: {current_sequence} does not exist")
            #     return
