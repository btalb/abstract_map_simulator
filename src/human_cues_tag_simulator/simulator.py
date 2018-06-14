import csv
import os
import pdb
from PIL import Image
import re
import rospkg
import rospy

from human_cues_tag_simulator import tags

PACKAGE_NAME = 'human_cues_tag_simulator'

WORLD_DIR = 'worlds'
WORLD_TAG_DEPTH = 0.1
WORLD_TAG_WIDTH = 0.3
WORLD_TAG_CENTER = WORLD_TAG_WIDTH / 2
WORLD_TEMPLATE = 'config/template.world'
WORLD_VAR_REGEX = re.compile(r'@.*?@')
WORLD_WORLD_HEIGHT = 3

FLOOR_PLAN_SUFFIX = '.png'
PPM_SUFFIX = '_ppm.txt'
START_POSE_SUFFIX = '_start_pose.txt'
TAGS_SUFFIX = '_tags.txt'
WORLD_SUFFIX = '.world'


def defaultStartPose():
    """Returns a default starting pose"""
    return {'x': 0, 'y': 0, 'th_deg': 0}


def generateWorldFile(env_name):
    """Generate a world file from a provided environment name"""
    # Get some filenames
    rospack = rospkg.RosPack()
    path_pkg = rospack.get_path(PACKAGE_NAME)
    fn_template = os.path.join(path_pkg, WORLD_TEMPLATE)
    fn_env = os.path.join(path_pkg, WORLD_DIR, env_name)
    fn_fp = fn_env + FLOOR_PLAN_SUFFIX
    fn_ppm = fn_env + PPM_SUFFIX
    fn_start_pose = fn_env + START_POSE_SUFFIX
    fn_tags = fn_env + TAGS_SUFFIX
    fn_out = fn_env + WORLD_SUFFIX

    # Load all required data from the files
    im = Image.open(fn_fp)
    im_w, im_h = im.size
    ppm = loadPixelsPerMeter(fn_ppm)
    start_pose = (defaultStartPose() if not os.path.exists(fn_start_pose) else
                  loadStartPose(fn_start_pose))
    tag_list = tags.loadTags(fn_tags)

    # Loop through the template file, writing the output file at the same time
    subs_dict = {
        '@TAG_DEPTH@': str(WORLD_TAG_DEPTH),
        '@TAG_WIDTH@': str(WORLD_TAG_WIDTH),
        '@TAG_CENTER@': str(WORLD_TAG_CENTER),
        '@WORLD_HEIGHT@': str(WORLD_WORLD_HEIGHT),
        '@FLOOR_PLAN_FILENAME@': os.path.basename(fn_fp),
        '@FLOOR_PLAN_WH@': "%f %f" % (im_w / ppm, im_h / ppm),
        '@FLOOR_PLAN_XY@': "%f %f" % (-0.5 * im_w / ppm, -0.5 * im_h / ppm),
        '@ROBOT_POSE@': "%f %f %f" % tuple(start_pose.values())
    }
    with open(fn_template, 'r') as template_file, open(fn_out,
                                                       'w') as world_file:
        for line in template_file:
            line_out = line
            for v in re.findall(WORLD_VAR_REGEX, line):
                line_out = line_out.replace(v, subs_dict[v])
            world_file.write(line_out)

    # Finish, telling the user what was done
    print("Wrote a new world file to: %s" % (fn_out))


def loadPixelsPerMeter(fn, quiet=False):
    """Attempts to load a pixels per meter value from a provided file"""
    with open(fn, 'r') as ppm_file:
        ppm_reader = csv.reader(ppm_file, delimiter=' ')
        row = next(ppm_reader)
        ppm = float(row[0]) / float(row[1])

    if not quiet:
        rospy.loginfo("Loaded a PPM value of %f from: %s" % (ppm, fn))
    return ppm


def loadStartPose(fn):
    """Attempts to load a starting pose from a provided pose file"""
    with open(fn, 'r') as pose_file:
        pose_reader = csv.reader(pose_file, delimiter=' ')
        row = next(pose_reader)
        pose = {
            'x': float(row[0][0]),
            'y': float(row[0][1]),
            'th_deg': float(row[0][2])
        }

    rospy.loginfo("Loaded the following starting pose from: %s" % (fn))
    rospy.loginfo("\t%s" % (startPose2String(pose)))
    return pose


def startPose2String(p):
    """Takes a pose, and turns it into a verbose string"""
    return ("Starting pose @ (%f, %f), facing %f deg" % (p['x'], p['y'],
                                                         p['th_deg']))
