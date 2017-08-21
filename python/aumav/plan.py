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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import RCIn, RCOut


log = logging.getLogger('aumav.plan')


class LandingFailed(Exception): pass
class TrackingFailed(Exception): pass
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


class BaseRobot(object):

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

    def _receive_local_pose(self, msg):
        pos, ori = msg.pose.position, msg.pose.orientation
        self.position = np.r_[pos.x, pos.y, pos.z]
        self.orientation = np.r_[ori.x, ori.y, ori.z, ori.w]
        self.last_pose_time = msg.header.stamp
        if msg.header.frame_id != self.world_frame:
            log.warn('received pose in frame %r but expected it to be in %r',
                     msg.header.frame_id, self.world_frame)

    @property
    def orientation_euler(self):
        return euler_from_quaternion(self.orientation)

    @property
    def heading(self):
        return self.orientation_euler[2]

    def _receive_tracking_status(self, msg):
        self.is_tracking = msg.data

    def coerce_to_pose(self, target, frame_id=None):
        if hasattr(target, 'pose') and hasattr(target, 'header'):
            return target
        px, py, pz = self.position
        qx, qy, qz, qw = self.orientation
        if len(target) == 2:
            try:
                (px, py, pz), (qx, qy, qz, qw) = target
            except IndexError:
                (px, py) = target
        elif len(target) == 3:
            px, py, pz = target
        elif len(target) == 4:
            px, py, pz, theta = target
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
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
        while self.position is None:
            rospy.loginfo_throttle(10.0, 'waiting for pose from mavros')
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

    def hover_and_land(self, it, timeout=5.0):
        "Hover on tracking failure, landing after *timeout*."
        with closing(it):
            for v in it:
                yield
                if not self.is_tracking:
                    log.warn('tracking failsafe triggered; hovering for %.3g s', timeout)
                    for v in chain(timed(timeout, self.hover()), self.land()):
                        yield
                        if self.is_tracking:
                            log.info('recovered from failure')
                            break
                    else:
                        # Break out of outermost loop if not recovered
                        log.warn('unable to recover from tracking failure; landing now')
                        raise TrackingFailed


class PathPlanningMixin(object):

    def __init__(self, *a, **k):
        super(PathPlanningMixin, self).__init__(*a, **k)
        self._path_planner_client = SimpleActionClient('/path_planner', PoseStamped)

    def move_planned(self, target):
        goal = self.coerce_to_pose(target, frame_id=self.world_frame)
        with closing(action_client_generator(self._path_planner_client, goal)) as it:
            for v in it:
                yield

    def wait_for_init(self):
        super(PathPlanningMixin, self).wait_for_init()
        while not self._path_planner_client.wait_for_server(rospy.Duration(-1.0)):
            rospy.loginfo_throttle(10.0, 'waiting for path planning action server')
            yield


class RCMixin(object):
    
    rc_min, rc_max = 1000.0, 2000.0
    rc_range = rc_max - rc_min
    rc_num_in, rc_num_out = 8, 8
    rc_in_names = ['dy', 'dx', 'dz', 'dtheta', None, None, None, 'toggle']
    rc_in_indices = [rc_in_names.index('dx'),
                     rc_in_names.index('dy'),
                     rc_in_names.index('dz'),
                     rc_in_names.index('dtheta')]
    rc_in_toggle = rc_in_names.index('toggle')
    assert len(rc_in_names) == rc_num_in

    def __init__(self, *a, **k):
        super(RCMixin, self).__init__(*a, **k)
        # RC controller inputs in [1000, 2000]
        self._rc_in, self._rc_in_rssi = None, None
        self._sub_rc_in  = rospy.Subscriber('/mavros/rc/in', RCIn, self._receive_rc_in)
        # PWM outputs in [1000, 2000]
        self._rc_out = None
        self._sub_rc_out = rospy.Subscriber('/mavros/rc/out', RCOut, self._receive_rc_out)

    def _receive_rc_in(self, msg):
        assert len(msg.channels) == self.rc_num_in
        self._rc_in = np.r_[msg.channels]
        self._rc_in_rssi = msg.rssi

    def _receive_rc_out(self, msg):
        assert len(msg.channels) == self.rc_num_out
        self._rc_out = np.r_[msg.channels]

    def rc_control(self, a_max=0.5, alpha_max=np.pi/4, gamma=0.8, deadzone=5e-2):
        self.rc_setpoint = np.r_[self.position, self.heading]
        vel = np.r_[0.0, 0.0, 0.0, 0.0]
        scale = np.r_[a_max, -a_max, a_max, -alpha_max]/self.control_frequency
        while True:
            if self._rc_in is None:
                rospy.logwarn_throttle(2.0, 'No RC input received in RC control loop')
                continue
            if self._rc_in[self.rc_in_toggle] > self.rc_max*0.8:
                self.rc_setpoint = np.r_[self.position, self.heading]
            rc_in = 2.0*(self._rc_in[self.rc_in_indices] - self.rc_min)/self.rc_range - 1.0
            rc_in[np.abs(rc_in) < deadzone] = 0.0
            vel = gamma*vel + rc_in*scale
            theta = self.rc_setpoint[3]
            R = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                          [np.sin(theta),  np.cos(theta), 0, 0],
                          [            0,              0, 1, 0],
                          [            0,              0, 0, 1]])
            self.rc_setpoint += np.dot(R, vel)
            target_msg = self.coerce_to_pose(self.rc_setpoint, frame_id=self.world_frame)
            self._pub_setpoint_position.publish(target_msg)
            yield


def main(args=sys.argv[1:]):
    rospy.init_node('aumav_plan')
    logcolor.basic_config(level=logging.INFO)
    class Robot(RCMixin, BaseRobot): pass
    r = Robot()
    r.run(r.wait_for_init())
    log.info('i am robot')
    #r.run(r.hover_and_land(r.move_planned((1,2,3))))
    #r.run(r.hover_and_land(r.scan()))
    #r.run(r.hover_and_land(r.publish_setpoint((0, 0, 1.0))))
    r.run(r.hover_and_land(r.rc_control()))

if __name__ == "__main__":
    main()
