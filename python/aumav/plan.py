"Planning -- the final frontier"

import sys
import logging
import logcolor
from time import time as clock
from contextlib import closing
from itertools import chain

import rospy
import numpy as np
from actionlib import SimpleActionClient, SimpleGoalState, GoalStatus
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from actionlib.msg import TestAction, TestGoal
from mavros_msgs.srv import CommandTOL


log = logging.getLogger('aumav.plan')


class LandingFailed(Exception): pass
class ActionFailed(Exception): pass
class ActionAborted(ActionFailed): pass
class ActionPreempted(ActionFailed): pass


def timed(timeout, it):
    t0 = clock()
    for v in it:
        try:
            yield
        except:
            it.throw(*sys.exc_info())
        if clock() - t0 >= timeout:
            break


def action_client_generator(client, goal):
    assert client.simple_state == SimpleGoalState.DONE
    client.send_goal(goal)
    try:
        while True:
            yield
            status = client.get_state()
            if status in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                continue
            elif status == GoalStatus.SUCCEEDED:
                break
            elif status == GoalStatus.ABORTED:
                raise ActionAborted(client.get_goal_status_text())
            elif status == GoalStatus.PREEMPTED:
                raise ActionPreempted(client.get_goal_status_text())
            else:
                raise ActionFailed(GoalStatus.to_string(status))
    finally:
        client.cancel_goal()


class Robot(object):

    world_frame = 'world'
    control_frequency = 50

    def __init__(self):

        self.position = None
        self.orientation = None
        self._sub_local_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._receive_local_pose)

        self.is_tracking = False
        self._sub_tracking_status = rospy.Subscriber('/slam/tracking_status', Bool, self._receive_tracking_status)

        self._land_rpc = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self._pub_setpoint_position = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        #self._path_planner_client = SimpleActionClient('/path_planner', PoseStamped)
        self._path_planner_client = SimpleActionClient('/path_planner', TestAction)

    def _receive_local_pose(self, msg):
        pos, ori = msg.pose.position, msg.pose.orientation
        self.position = np.r_[pos.x, pos.y, pos.z]
        self.orientation = np.r_[ori.x, ori.y, ori.z, ori.w]
        self.last_pose_time = msg.header.stamp
        if msg.header.frame_id != self.world_frame:
            log.warn('received pose in frame %r but expected it to be in %r',
                     msg.header.frame_id, self.world_frame)

    def _receive_tracking_status(self, msg):
        self.is_tracking = msg.data

    @property
    def orientation_euler(self):
        return euler_from_quaternion(self.orientation)

    def coerce_to_pose(self, target, frame_id=None):
        if hasattr(target, 'pose') and hasattr(target, 'header'):
            return target
        px, py, pz = self.position
        qx, qy, qz, qw = self.orientation
        if len(target) == 3:
            px, py, pz = target
        elif len(target) == 2:
            (px, py, pz), (qx, qy, qz, qw) = target
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = px, py, pz
        pose.orientation = Quaternion(qx, qy, qz, qw)
        if frame_id is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose = pose
            return pose_stamped
        else:
            return pose

    def wait_for_init(self):
        log.info('waiting for pose from mavros')
        while self.position is None:
            yield
        log.info('waiting for path planning action server')
        while not self._path_planner_client.wait_for_server(rospy.Duration(-1.0)):
            yield

    def run(self, it):
        with closing(it):
            rate = rospy.Rate(self.control_frequency)
            for v in it:
                if rospy.is_shutdown():
                    break
                if rate.remaining() < -rate.sleep_dur*0.1:
                    log.warn('control runs slow, deadline missed by %.3g',
                             rate.remaining().to_sec())
                rate.sleep()

    def publish_setpoint(self, target):
        target_msg = self.coerce_to_pose(target, frame_id=self.world_frame)
        while True:
            target_msg.header.stamp = rospy.Time.now()
            self._pub_setpoint_position.publish(target_msg)
            yield

    def until_at(self, target, it, tolerance=5e-2):
        "Execute it until within *tolerance* of target"
        with it:
            for v in it:
                if np.linalg.norm(self.position - target) <= tolerance:
                    break
                yield

    def move_to(self, target, tolerance=5e-2):
        return self.until_at(target, self.publish_setpoint(target))

    def hover(self):
        "Publish current position as setpoint"
        return self.publish_setpoint(self.position)

    def land(self):
        nan = float('nan')
        resp = self._land_rpc(nan, nan, nan, nan, nan)
        if not resp.success:
            raise LandingFailed(resp.result)
        yield

    def move_planned(self, target):
        goal = self.coerce_to_pose(target, frame_id=self.world_frame)
        goal = TestGoal(23)
        with closing(action_client_generator(self._path_planner_client, goal)) as it:
            for v in it:
                yield

    def hover_and_land(self, it, timeout=5.0):
        "Hover on tracking failure, landing after *timeout*."
        with closing(it):
            for v in it:
                yield
                if not self.is_tracking:
                    log.warn('tracking failsafe triggered; hovering for %.3g s', timeout)
                    for v in chain(timed(timeout, self.hover()), self.land()):
                        if self.is_tracking:
                            log.info('recovered from failure')
                            break
                    else:
                        # Break out of outermost loop if not recovered
                        break


def main(args=sys.argv[1:]):
    rospy.init_node('aumav_plan')
    logcolor.basic_config(level=logging.INFO)
    r = Robot()
    log.info('i am robot')
    r.run(r.wait_for_init())
    #r.run(r.hover_and_land(r.move_planned((1,2,3))))
    r.run(r.hover_and_land(r.publish_setpoint((0, 0, 1.0))))

if __name__ == "__main__":
    main()
