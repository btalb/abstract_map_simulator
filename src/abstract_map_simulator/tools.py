import os
import rospkg
import tf_conversions

PACKAGE_NAME = 'abstract_map_simulator'

DIR_CONFIG = 'config'
DIR_WORLDS = 'worlds'

SUFFIX_WORLD = '.world'


def configPath():
    """Returns the path to the config directory"""
    return os.path.join(packagePath(), DIR_CONFIG)


def packagePath():
    """Returns the root directory of the simulator package"""
    return rospkg.RosPack().get_path(PACKAGE_NAME)


def quaternionMsgToTuple(msg):
    """Helper function for converting a quaternion msg to a tuple"""
    return (msg.x, msg.y, msg.z, msg.w)


def quaternionMsgToYaw(msg):
    """Helper function for getting the yaw angle from a quaternion msg"""
    r, p, y = tf_conversions.transformations.euler_from_quaternion(
        quaternionMsgToTuple(msg))
    return y


def worldPath(env_name):
    """Returns the path to the world file for a requested environment"""
    return worldRoot(env_name) + SUFFIX_WORLD


def worldRoot(env_name):
    """Returns the common root of the file paths for a requested environment"""
    return os.path.join(packagePath(), DIR_WORLDS, env_name)
