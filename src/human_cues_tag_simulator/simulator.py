import csv
import os
import pdb
from PIL import Image
import re
import rospkg
import rospy

from human_cues_tag_simulator import tags, tools

WORLD_TAG_DEPTH = 0.25
WORLD_TAG_HEIGHT = 0.5
WORLD_TAG_OFFSET = WORLD_TAG_DEPTH / 2
WORLD_TAG_WIDTH = 0.5
WORLD_TAG_Z = 1.35 - WORLD_TAG_HEIGHT / 2
WORLD_VAR_REGEX = re.compile(r'@.*?@')
WORLD_WORLD_HEIGHT = 3
WORLD_TAG_STRING = "apriltag\n(\n name \"tag_@id\"\n pose [ @pose ]\n)"

FILE_TEMPLATE = 'template.world'

SUFFIX_FLOOR_PLAN = '.png'
SUFFIX_PPM = '_ppm.txt'
SUFFIX_START_POSE = '_start_pose.txt'
SUFFIX_TAGS = '_tags.txt'


def defaultStartPose():
    """Returns a default starting pose"""
    return {'x': 0, 'y': 0, 'th_deg': 0}


def generateWorldFile(env_name):
    """Generate a world file from a provided environment name"""
    # Get some filenames
    fn_template = os.path.join(tools.configPath(), FILE_TEMPLATE)
    fn_env = tools.worldRoot(env_name)
    fn_fp = fn_env + SUFFIX_FLOOR_PLAN
    fn_ppm = fn_env + SUFFIX_PPM
    fn_start_pose = fn_env + SUFFIX_START_POSE
    fn_tags = fn_env + SUFFIX_TAGS
    fn_out = tools.worldPath(env_name)

    # Load all required data from the files
    im = Image.open(fn_fp)
    im_w, im_h = im.size
    ppm = loadPixelsPerMeter(fn_ppm)
    start_pose = (defaultStartPose() if not os.path.exists(fn_start_pose) else
                  loadStartPose(fn_start_pose))
    tag_list = tags.loadTags(fn_tags)

    # Loop through the template file, writing the output file at the same time
    subs_dict = {
        '@TAG_DEPTH@':
        str(WORLD_TAG_DEPTH),
        '@TAG_HEIGHT@':
        str(WORLD_TAG_HEIGHT),
        '@TAG_OFFSET@':
        str(WORLD_TAG_OFFSET),
        '@TAG_WIDTH@':
        str(WORLD_TAG_WIDTH),
        '@WORLD_HEIGHT@':
        str(WORLD_WORLD_HEIGHT),
        '@FLOOR_PLAN_FILENAME@':
        os.path.basename(fn_fp),
        '@FLOOR_PLAN_WH@':
        "%f %f" % (im_w / ppm, im_h / ppm),
        '@FLOOR_PLAN_XY@':
        "%f %f" % (0, 0),
        '@ROBOT_POSE@':
        "%f %f 0 %f" % (start_pose['x'], start_pose['y'], start_pose['th_deg'])
    }
    with open(fn_template, 'r') as template_file, open(fn_out,
                                                       'w') as world_file:
        # Rewrite each of the existing lines, with the included variables
        for line in template_file:
            line_out = line
            for v in re.findall(WORLD_VAR_REGEX, line):
                line_out = line_out.replace(v, subs_dict[v])
            world_file.write(line_out)

        # Add in the lines for each tag object to the end of the file
        for t in tag_list:
            world_file.write(
                WORLD_TAG_STRING.replace("@id", str(t['id'])).replace(
                    "@pose", "%f %f %f %f" %
                    (t['x'], t['y'], WORLD_TAG_Z, t['th_deg'])) + "\n\n")

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
        pose = {'x': float(row[0]), 'y': float(row[1]), 'th_deg': float(row[2])}

    rospy.loginfo("Loaded the following starting pose from: %s" % (fn))
    rospy.loginfo("\t%s" % (startPose2String(pose)))
    return pose


def startPose2String(p):
    """Takes a pose, and turns it into a verbose string"""
    return ("Starting pose @ (%f, %f), facing %f deg" % (p['x'], p['y'],
                                                         p['th_deg']))
