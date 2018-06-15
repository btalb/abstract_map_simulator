import os
import rospkg

PACKAGE_NAME = 'human_cues_tag_simulator'

DIR_CONFIG = 'config'
DIR_WORLDS = 'worlds'

SUFFIX_WORLD = '.world'


def configPath():
    """Returns the path to the config directory"""
    return os.path.join(packagePath(), DIR_CONFIG)


def packagePath():
    """Returns the root directory of the simulator package"""
    return rospkg.RosPack().get_path(PACKAGE_NAME)


def worldPath(env_name):
    """Returns the path to the world file for a requested environment"""
    return worldRoot(env_name) + SUFFIX_WORLD


def worldRoot(env_name):
    """Returns the common root of the file paths for a requested environment"""
    return os.path.join(packagePath(), DIR_WORLDS, env_name)
