#! /usr/bin/env python

from geometry_msgs.msg import Pose, Quaternion, Twist
from rbot_msgs.msg import ControlCommand, State

import rbot_utils


def test_copy_1d_if():
    a = Quaternion()
    c = Quaternion()
    d = Quaternion()

    a.x = 6
    c.x = True
    d.x = False

    b = Quaternion()
    rbot_utils.copy_1d_if(a, b, c)
    assert(a.x == b.x)

    b = Quaternion()
    rbot_utils.copy_1d_if(a, b, d)
    assert(not (a.x == b.x))


def test_copy_2d_if():
    a = Quaternion()
    c = Quaternion()
    d = Quaternion()

    a.x = 6
    a.y = 7

    c.x = True
    c.y = True

    d.x = False
    d.y = False

    b = Quaternion()
    rbot_utils.copy_2d_if(a, b, c)
    assert(a.x == b.x)
    assert(a.y == b.y)

    b = Quaternion()
    rbot_utils.copy_2d_if(a, b, d)
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))


def test_copy_3d_if():
    a = Quaternion()
    c = Quaternion()
    d = Quaternion()

    a.x = 6
    a.y = 7
    a.z = 8

    c.x = True
    c.y = True
    c.z = True

    d.x = False
    d.y = False
    d.z = False

    b = Quaternion()
    rbot_utils.copy_3d_if(a, b, c)
    assert(a.x == b.x)
    assert(a.y == b.y)
    assert(a.z == b.z)

    b = Quaternion()
    rbot_utils.copy_3d_if(a, b, d)
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))
    assert(not (a.z == b.z))


def test_copy_4d_if():
    a = Quaternion()
    c = Quaternion()
    d = Quaternion()

    a.x = 6
    a.y = 7
    a.z = 8
    a.w = 9

    c.x = True
    c.y = True
    c.z = True
    c.w = True

    d.x = False
    d.y = False
    d.z = False
    d.w = False

    b = Quaternion()
    rbot_utils.copy_4d_if(a, b, c)
    assert(a.x == b.x)
    assert(a.y == b.y)
    assert(a.z == b.z)
    assert(a.w == b.w)
    assert(a == b)

    b = Quaternion()
    rbot_utils.copy_4d_if(a, b, d)
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))
    assert(not (a.z == b.z))
    assert(not (a.w == b.w))
    assert(not (a == b))


def test_copy_pose_if():
    a = Pose()
    c = Pose()
    d = Pose()

    a.position.x = 3
    a.position.y = 4
    a.position.z = 5
    a.orientation.x = 6
    a.orientation.y = 7
    a.orientation.z = 8
    a.orientation.w = 9

    c.position.x = True
    c.position.y = True
    c.position.z = True
    c.orientation.x = True
    c.orientation.y = True
    c.orientation.z = True
    c.orientation.w = True

    d.position.x = False
    d.position.y = False
    d.position.z = False
    d.orientation.x = False
    d.orientation.y = False
    d.orientation.z = False
    d.orientation.w = False

    b = Pose()
    rbot_utils.copy_pose_if(a, b, c)
    assert(a.position == b.position)
    assert(a.orientation == b.orientation)
    assert(a == b)

    b = Pose()
    rbot_utils.copy_pose_if(a, b, d)
    assert(not (a.position == b.position))
    assert(not (a.orientation == b.orientation))
    assert(not (a == b))


def test_copy_motion_if():
    a = Twist()
    c = Twist()
    d = Twist()

    a.linear.x = 3
    a.linear.y = 4
    a.linear.z = 5
    a.angular.x = 6
    a.angular.y = 7
    a.angular.z = 8

    c.linear.x = True
    c.linear.y = True
    c.linear.z = True
    c.angular.x = True
    c.angular.y = True
    c.angular.z = True

    d.linear.x = False
    d.linear.y = False
    d.linear.z = False
    d.angular.x = False
    d.angular.y = False
    d.angular.z = False

    b = Twist()
    rbot_utils.copy_motion_if(a, b, c)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)

    b = Twist()
    rbot_utils.copy_velocity_if(a, b, c)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)

    b = Twist()
    rbot_utils.copy_acceleration_if(a, b, c)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)

    b = Twist()
    rbot_utils.copy_motion_if(a, b, d)
    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))

    b = Twist()
    rbot_utils.copy_velocity_if(a, b, d)
    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))

    b = Twist()
    rbot_utils.copy_acceleration_if(a, b, d)
    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))


def test_copy_setpoint_if():
    a = State()
    c = State()
    d = State()

    a.pose.position.x = 3
    a.pose.position.y = 4
    a.pose.position.z = 5
    a.pose.orientation.x = 6
    a.pose.orientation.y = 7
    a.pose.orientation.z = 8
    a.pose.orientation.w = 9
    a.velocity.linear.x = 3
    a.velocity.linear.y = 4
    a.velocity.linear.z = 5
    a.acceleration.angular.x = 6
    a.acceleration.angular.y = 7
    a.acceleration.angular.z = 8

    c.pose.position.x = True
    c.pose.position.y = True
    c.pose.position.z = True
    c.pose.orientation.x = True
    c.pose.orientation.y = True
    c.pose.orientation.z = True
    c.pose.orientation.w = True
    c.velocity.linear.x = True
    c.velocity.linear.y = True
    c.velocity.linear.z = True
    c.acceleration.angular.x = True
    c.acceleration.angular.y = True
    c.acceleration.angular.z = True

    d.pose.position.x = False
    d.pose.position.y = False
    d.pose.position.z = False
    d.pose.orientation.x = False
    d.pose.orientation.y = False
    d.pose.orientation.z = False
    d.pose.orientation.w = False
    d.velocity.linear.x = False
    d.velocity.linear.y = False
    d.velocity.linear.z = False
    d.acceleration.angular.x = False
    d.acceleration.angular.y = False
    d.acceleration.angular.z = False

    b = State()
    rbot_utils.copy_setpoint_if(a, b, c)
    assert(a.pose == b.pose)
    assert(a.velocity == b.velocity)
    assert(a.acceleration == b.acceleration)
    assert(a == b)

    b = State()
    rbot_utils.copy_setpoint_if(a, b, d)
    assert(not (a.pose == b.pose))
    assert(not (a.velocity == b.velocity))
    assert(not (a.acceleration == b.acceleration))
    assert(not (a == b))


def test_copy_1d():
    a = Quaternion()
    b = Quaternion()

    a.x = 1
    assert(not (a.x == b.x))
    rbot_utils.copy_1d(a, b)
    assert(a.x == b.x)


def test_copy_2d():
    a = Quaternion()
    b = Quaternion()

    a.x = 1
    a.y = 2
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))
    rbot_utils.copy_2d(a, b)
    assert(a.x == b.x)
    assert(a.y == b.y)


def test_copy_3d():
    a = Quaternion()
    b = Quaternion()

    a.x = 1
    a.y = 2
    a.z = 3
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))
    assert(not (a.z == b.z))
    rbot_utils.copy_3d(a, b)
    assert(a.x == b.x)
    assert(a.y == b.y)
    assert(a.z == b.z)


def test_copy_4d():
    a = Quaternion()
    b = Quaternion()

    a.x = 1
    a.y = 2
    a.z = 3
    a.w = 4
    assert(not (a.x == b.x))
    assert(not (a.y == b.y))
    assert(not (a.z == b.z))
    assert(not (a.w == b.w))
    rbot_utils.copy_4d(a, b)
    assert(a.x == b.x)
    assert(a.y == b.y)
    assert(a.z == b.z)
    assert(a.w == b.w)


def test_copy_pose():
    a = Pose()
    b = Pose()
    c = Pose()
    d = Pose()

    a.position.x = 5
    a.position.y = 6
    a.position.z = 7
    a.orientation.x = 1
    a.orientation.y = 2
    a.orientation.z = 3
    a.orientation.w = 4

    assert(not (a.position == b.position))
    assert(not (a.orientation == b.orientation))
    assert(not (a == b))
    rbot_utils.copy_pose(a, b)
    assert(a.position == b.position)
    assert(a.orientation == b.orientation)
    assert(a == b)


def test_copy_motion():
    a = Twist()
    b = Twist()

    a.linear.x = 3
    a.linear.y = 4
    a.linear.z = 5
    a.angular.x = 6
    a.angular.y = 7
    a.angular.z = 8

    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))
    rbot_utils.copy_motion(a, b)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)

    b = Twist()
    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))
    rbot_utils.copy_velocity(a, b)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)

    b = Twist()
    assert(not (a.linear == b.linear))
    assert(not (a.angular == b.angular))
    assert(not (a == b))
    rbot_utils.copy_acceleration(a, b)
    assert(a.linear == b.linear)
    assert(a.angular == b.angular)
    assert(a == b)


def test_copy_setpoint():
    a = State()
    b = State()

    a.pose.position.x = 3
    a.pose.position.y = 4
    a.pose.position.z = 5
    a.pose.orientation.x = 6
    a.pose.orientation.y = 7
    a.pose.orientation.z = 8
    a.pose.orientation.w = 9
    a.velocity.linear.x = 3
    a.velocity.linear.y = 4
    a.velocity.linear.z = 5
    a.acceleration.angular.x = 6
    a.acceleration.angular.y = 7
    a.acceleration.angular.z = 8

    assert(not (a.pose == b.pose))
    assert(not (a.velocity == b.velocity))
    assert(not (a.acceleration == b.acceleration))
    assert(not (a == b))
    rbot_utils.copy_setpoint(a, b)
    assert(a.pose == b.pose)
    assert(a.velocity == b.velocity)
    assert(a.acceleration == b.acceleration)
    assert(a == b)


def test_any_1d():
    a = Quaternion()
    assert(not rbot_utils.any_1d(a))

    a.x = True
    assert(rbot_utils.any_1d(a))


def test_any_2d():
    a = Quaternion()
    assert(not rbot_utils.any_2d(a))

    a.x = True
    assert(rbot_utils.any_2d(a))

    a = Quaternion()
    a.y = True
    assert(rbot_utils.any_2d(a))


def test_any_3d():
    a = Quaternion()
    assert(not rbot_utils.any_3d(a))

    a.x = True
    assert(rbot_utils.any_3d(a))

    a = Quaternion()
    a.y = True
    assert(rbot_utils.any_3d(a))

    a = Quaternion()
    a.z = True
    assert(rbot_utils.any_3d(a))


def test_any_4d():
    a = Quaternion()
    assert(not rbot_utils.any_4d(a))

    a.x = True
    assert(rbot_utils.any_4d(a))

    a = Quaternion()
    a.y = True
    assert(rbot_utils.any_4d(a))

    a = Quaternion()
    a.z = True
    assert(rbot_utils.any_4d(a))

    a = Quaternion()
    a.w = True
    assert(rbot_utils.any_4d(a))


def test_any_pose():
    a = Pose()
    assert(not rbot_utils.any_pose(a))

    a.position.x = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.position.y = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.position.z = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.orientation.x = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.orientation.y = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.orientation.z = True
    assert(rbot_utils.any_pose(a))

    a = Pose()
    a.orientation.w = True
    assert(rbot_utils.any_pose(a))


def test_any_motion():
    a = Twist()
    assert(not rbot_utils.any_motion(a))
    assert(not rbot_utils.any_velocity(a))
    assert(not rbot_utils.any_acceleration(a))

    a.linear.x = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))

    a = Twist()
    a.linear.y = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))

    a = Twist()
    a.linear.z = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))

    a = Twist()
    a.angular.x = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))

    a = Twist()
    a.angular.y = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))

    a = Twist()
    a.angular.z = True
    assert(rbot_utils.any_motion(a))
    assert(rbot_utils.any_velocity(a))
    assert(rbot_utils.any_acceleration(a))


def test_any_setpoint():
    a = State()
    assert(not rbot_utils.any_setpoint(a))

    a.pose.position.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.position.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.position.z = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.orientation.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.orientation.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.orientation.z = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.pose.orientation.w = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.linear.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.linear.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.linear.z = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.angular.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.angular.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.velocity.angular.z = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.linear.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.linear.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.linear.z = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.angular.x = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.angular.y = True
    assert(rbot_utils.any_setpoint(a))

    a = State()
    a.acceleration.angular.z = True
    assert(rbot_utils.any_setpoint(a))


def test_all_1d():
    a = Quaternion()
    assert(not rbot_utils.all_1d(a))

    a.x = True
    assert(rbot_utils.all_1d(a))


def test_all_2d():
    a = Quaternion()
    assert(not rbot_utils.all_2d(a))

    a.x = True
    assert(not rbot_utils.all_2d(a))
    a.y = True
    assert(rbot_utils.all_2d(a))


def test_all_3d():
    a = Quaternion()
    assert(not rbot_utils.all_3d(a))

    a.x = True
    assert(not rbot_utils.all_3d(a))
    a.y = True
    assert(not rbot_utils.all_3d(a))
    a.z = True
    assert(rbot_utils.all_3d(a))


def test_all_4d():
    a = Quaternion()
    assert(not rbot_utils.all_4d(a))

    a.x = True
    assert(not rbot_utils.all_4d(a))
    a.y = True
    assert(not rbot_utils.all_4d(a))
    a.z = True
    assert(not rbot_utils.all_4d(a))
    a.w = True
    assert(rbot_utils.all_4d(a))


def test_all_pose():
    a = Pose()
    assert(not rbot_utils.all_pose(a))

    a.position.x = True
    assert(not rbot_utils.all_pose(a))

    a.position.y = True
    assert(not rbot_utils.all_pose(a))

    a.position.z = True
    assert(not rbot_utils.all_pose(a))

    a.orientation.x = True
    assert(not rbot_utils.all_pose(a))

    a.orientation.y = True
    assert(not rbot_utils.all_pose(a))

    a.orientation.z = True
    assert(not rbot_utils.all_pose(a))

    a.orientation.w = True
    assert(rbot_utils.all_pose(a))


def test_all_motion():
    a = Twist()
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.linear.x = True
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.linear.y = True
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.linear.z = True
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.angular.x = True
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.angular.y = True
    assert(not rbot_utils.all_motion(a))
    assert(not rbot_utils.all_velocity(a))
    assert(not rbot_utils.all_acceleration(a))

    a.angular.z = True
    assert(rbot_utils.all_motion(a))
    assert(rbot_utils.all_velocity(a))
    assert(rbot_utils.all_acceleration(a))


def test_all_setpoint():
    a = State()
    assert(not rbot_utils.all_setpoint(a))

    a.pose.position.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.position.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.position.z = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.orientation.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.orientation.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.orientation.z = True
    assert(not rbot_utils.all_setpoint(a))

    a.pose.orientation.w = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.linear.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.linear.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.linear.z = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.angular.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.angular.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.velocity.angular.z = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.linear.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.linear.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.linear.z = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.angular.x = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.angular.y = True
    assert(not rbot_utils.all_setpoint(a))

    a.acceleration.angular.z = True
    assert(rbot_utils.all_setpoint(a))
