import sys
sys.path.append('C:/Program Files/RoboDK/Python/')

import numpy as np
from threading import Thread
import time
import signal
import sys

import robolink
import robodk as rdk

DESCEND_DISTANCE = -20
LIFT_DISTANCE = -50


class RobotChoreographer:
    def __init__(self, run_on_real=False, acceleration=1000, speed=500):
        self.robodk = robolink.Robolink()  # Link with simulator
        self.robot = self.robodk.Item('UR5')
        self.camera = self.robodk.Item('Camera')

        self.draw_frame = self.robodk.Item('Canvas')
        self.draw_tool = self.robodk.Item('Tool')

        self.robodk.setRunMode(robolink.RUNMODE_SIMULATE)

        if run_on_real:
            self.connect_to_real_robot()

        self.robot.setAcceleration(acceleration)
        self.robot.setSpeed(speed)

        self.last_pos_x = 0
        self.last_pos_y = 0
        self.last_pos_z = 0

    def connect_to_real_robot(self):
        if self.robodk.RunMode() != robolink.RUNMODE_SIMULATE:
            raise Exception('Cannot run on robot outside of simulation mode')

        # Connect to the robot using default IP
        self.robot.Connect()
        status, status_msg = self.robot.ConnectedState()

        if status != robolink.ROBOTCOM_READY:
            # Stop if the connection did not succeed
            print(status_msg)
            raise Exception("Failed to connect: " + status_msg)

        # This will set to run the API programs on the robot and the simulator
        # (online programming)
        self.robodk.setRunMode(robolink.RUNMODE_RUN_ROBOT)

    def movel_to_keyframe(self, keyframe_name):
        target = self.robodk.Item(keyframe_name)
        self.robot.MoveL(target)

    def movej_to_keyframe(self, keyframe_name):
        target = self.robodk.Item(keyframe_name)
        self.robot.MoveJ(target)

    def move_to(self, x, y, z):
        self.robot.setPoseFrame(self.draw_frame)
        self.robot.setPoseTool(self.draw_tool)

        orient_frame2tool = self.draw_tool.Pose()
        orient_frame2tool[0:3, 3] = rdk.Mat([0, 0, 0])

        pos = rdk.transl(x, y, z) * orient_frame2tool

        self.robot.MoveL(pos)

        self.last_pos_x = x
        self.last_pos_y = y
        self.last_pos_z = z

    def fire(self):
        self.robot.setDO('11', 0)
        self.robot.setDO('11', 1)


def tufting_speed(speed):
    if speed < 0 or speed > 1:
        raise Exception('Tufting speed must be between 0 and 1')

    # TODO


def tufting_enable(enabled):
    waldo.robot.setDO('0', 0 if enabled else 1)


def load_points_csv(filename):
    with open(filename, 'r') as csv_file:
        csv_lines = csv_file.readlines()
        header = csv_lines[0]
        print(header)

        data_lines = csv_lines[1:]

        points = []
        for line in data_lines:
            parts = line.split(',')
            x, y, z, cmd = parts[1:]
            s = 0.8 #0.5
            points += [(float(x) * s, float(y) * s, float(z) * s, int(cmd))]

    return points


if __name__ == '__main__':
    waldo = RobotChoreographer(True, 1000, 50)

    def signal_handler(sig, frame):
        print('Safing robot')
        tufting_enable(False)
        waldo.move_to(waldo.last_pos_x, waldo.last_pos_y, waldo.last_pos_z + LIFT_DISTANCE)
        print('Done! Goodbye')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    # signal.pause()

    tufting_enable(False)
    tufting_speed(1)

    # time.sleep(1)
    # tufting_enable(False)
    # print('Done')

    csv_file = sys.argv[1]
    points = load_points_csv(csv_file)

    def move_to(x, y, z):
        print('Moving to ({}, {}, {})'.format(x, y, z))

    row_cnt = 0

    waldo.move_to(0,0,LIFT_DISTANCE)

    for x, y, z, cmd in points:
        if row_cnt > 83:
            if cmd == -1:
                waldo.move_to(x, z, y + LIFT_DISTANCE)
                waldo.move_to(x, z, y + DESCEND_DISTANCE)
                tufting_enable(True)

            waldo.move_to(x, z, y)

            # if cmd == -1:
            #     tufting_enable(True)

            if cmd == 1:
                tufting_enable(False)
                waldo.move_to(x, z, y + LIFT_DISTANCE)
                # time.sleep(2)
            print(row_cnt)
        row_cnt+=1

    waldo.move_to(0, 0, LIFT_DISTANCE)
