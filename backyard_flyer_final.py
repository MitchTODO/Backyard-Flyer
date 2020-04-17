import argparse
import time
from enum import Enum
import math

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class Tools():

    """
     calculates bearing (heading)

     inputs: start_north: float
             start_east: float
             end_north: float
             end_east: float
     output: float
     Formula:	θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
     where	φ1,λ1 is the start point, φ2,λ2 the end point (Δλ is the difference in longitude)
    """
    def bearing(start_north,start_east,end_north,end_east):
        # convert long, lat to radians
        lat1 = math.radians(start_north)
        lat2 = math.radians(end_north)

        diffLong = math.radians(end_east - start_east)

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(x, y)
        return initial_bearing


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        #(north, east, altitude)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        altitude = -1.0 * self.local_position[2]
        altitude_acc = 0.95 * self.local_position[2]

        # rounding allows for small but noticable corrections near a waypoint
        north = round(self.local_position[0])
        east = round(self.local_position[1])

        if(self.flight_state == States.TAKEOFF):
            if(altitude > altitude_acc): # check for stable altitude
                # ready for flight moving to first waypoint
                self.waypoint_transition()

        elif(self.flight_state == States.WAYPOINT):
            # check if at waypoint
            if(north == self.target_position[0] and east == self.target_position[1]) and altitude > altitude_acc:
                if(north == 0.0 and east == 0.0):
                    # at home coords, now landing
                    self.landing_transition()
                else:
                # move to next waypoint
                    self.waypoint_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()




    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return # leave me alone im flying
        # drone is ready to fly a mission
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        # drone is ready for take off
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        # drone is ready to be disarmed in manual mode
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()



    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        coords = [
                  [15.0, 0.0, 3.0],
                  [15.0, 15.0, 3.0],
                  [0.0, 15.0, 3.0]
                 ]

        for coord in coords:
            self.all_waypoints.append(coord)

        return coords

    def arming_transition(self):
        """TODO: Fill out this method

        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING
        
        # assume when armed mission starts
        self.in_mission = True

        # set home position
        self.set_home_position(
                                self.global_position[0],
                                self.global_position[1],
                                self.global_position[2]
                              )

        self.calculate_box()
        print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method

        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")


    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        if(len(self.all_waypoints) == 0): # end of waypoints
            # going to home coords
            self.target_position[0] = 0.0
            self.target_position[1] = 0.0


        else:
            # moving to next waypoint
            self.target_position = self.all_waypoints.pop(0)
        #(north, east, altitude, heading)
        bearing = Tools.bearing(self.local_position[0],self.local_position[1],self.target_position[0], self.target_position[1])
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], bearing)
        self.flight_state = States.WAYPOINT
        print("waypoint transition")

    def landing_transition(self):
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method

        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        print("manual transition")

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
