#! /usr/bin/env python


def copy_1d_if(origin, destination, condition):
    if condition.x:
        destination.x = origin.x


def copy_2d_if(origin, destination, condition):
    copy_1d_if(origin, destination, condition)
    if condition.y:
        destination.y = origin.y


def copy_3d_if(origin, destination, condition):
    copy_2d_if(origin, destination, condition)
    if condition.z:
        destination.z = origin.z


def copy_4d_if(origin, destination, condition):
    copy_3d_if(origin, destination, condition)
    if condition.w:
        destination.w = origin.w


def copy_pose_if(origin, destination, condition):
    copy_3d_if(origin.position, destination.position, condition.position)
    copy_4d_if(origin.orientation,
               destination.orientation,
               condition.orientation)


def copy_motion_if(origin, destination, condition):
    copy_3d_if(origin.linear, destination.linear, condition.linear)
    copy_3d_if(origin.angular, destination.angular, condition.angular)


def copy_velocity_if(origin, destination, condition):
    copy_motion_if(origin, destination, condition)


def copy_acceleration_if(origin, destination, condition):
    copy_motion_if(origin, destination, condition)


def copy_setpoint_if(origin, destination, condition):
    copy_pose_if(origin.pose, destination.pose, condition.pose)
    copy_motion_if(origin.velocity, destination.velocity, condition.velocity)
    copy_motion_if(origin.acceleration,
                   destination.acceleration,
                   condition.acceleration)


def copy_1d(origin, destination):
    destination.x = origin.x


def copy_2d(origin, destination):
    copy_1d(origin, destination)
    destination.y = origin.y


def copy_3d(origin, destination):
    copy_2d(origin, destination)
    destination.z = origin.z


def copy_4d(origin, destination):
    copy_3d(origin, destination)
    destination.w = origin.w


def copy_pose(origin, destination):
    copy_3d(origin.position, destination.position)
    copy_4d(origin.orientation,
            destination.orientation)


def copy_motion(origin, destination):
    copy_3d(origin.linear, destination.linear)
    copy_3d(origin.angular, destination.angular)


def copy_velocity(origin, destination):
    copy_motion(origin, destination)


def copy_acceleration(origin, destination):
    copy_motion(origin, destination)


def copy_setpoint(origin, destination):
    copy_pose(origin.pose, destination.pose)
    copy_motion(origin.velocity, destination.velocity)
    copy_motion(origin.acceleration,
                destination.acceleration)


def any_1d(origin):
    return origin.x


def any_2d(origin):
    return any_1d(origin) or origin.y


def any_3d(origin):
    return any_2d(origin) or origin.z


def any_4d(origin):
    return any_3d(origin) or origin.w


def any_pose(origin):
    return any_3d(origin.position) or any_4d(origin.orientation)


def any_motion(origin):
    return any_3d(origin.linear) or any_3d(origin.angular)


def any_velocity(origin):
    return any_motion(origin)


def any_acceleration(origin):
    return any_motion(origin)


def any_setpoint(origin):
    return any_pose(origin.pose) or \
        any_motion(origin.velocity) or \
        any_motion(origin.acceleration)


def all_1d(origin):
    return origin.x


def all_2d(origin):
    return all_1d(origin) and origin.y


def all_3d(origin):
    return all_2d(origin) and origin.z


def all_4d(origin):
    return all_3d(origin) and origin.w


def all_pose(origin):
    return all_3d(origin.position) and all_4d(origin.orientation)


def all_motion(origin):
    return all_3d(origin.linear) and all_3d(origin.angular)


def all_velocity(origin):
    return all_motion(origin)


def all_acceleration(origin):
    return all_motion(origin)


def all_setpoint(origin):
    return all_pose(origin.pose) and \
        all_motion(origin.velocity) and \
        all_motion(origin.acceleration)
