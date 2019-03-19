import glob
import os
from PIL import Image
import pdb
import sys

from human_cues_tag_simulator import simulator as sim

# THIS IS OLD DIRTY CODE (SHOULD STILL WORK, BUT NOT GUARANTEED TO BE
# CONSISTENT WITH WHAT IS DONE IN THE LIBRARY FILES)

PPM_SUFFIX = '_ppm.txt'

THRESH_OCCUPIED = 0.6
THRESH_FREE = 0.3


def main(map_filename):
    """Constructs a yaml file for a map image"""
    # Get the filename elements (including map name)
    fn_dir = os.path.abspath(os.path.dirname(map_filename))
    fn_map, fn_ext = os.path.splitext(os.path.basename(map_filename))

    # Get the pixels per meter value from file
    ppm = sim.loadPixelsPerMeter(
        os.path.join(fn_dir, fn_map + PPM_SUFFIX), True)
    print("\tExtracted a Pixels Per Meter of: %f" % (ppm))

    # Get the dimensions of the image
    map_image = Image.open(map_filename)
    map_w, map_h = map_image.size
    print("Map image has dimensions %dx%d" % (map_w, map_h))

    # Derive everything needed for the YAML file
    print("Deriving values needed for the YAML file:")
    mpp = 1 / ppm
    print("\tMeters per pixel:\t%f" % (mpp))
    print("\tMap dimensions (m):\t%fx%f" % (map_w * mpp, map_h * mpp))
    origin_x = -map_w / 2 * mpp
    origin_y = -map_h / 2 * mpp
    print("\tOrigin of map:\t\t(%f, %f, 0)" % (origin_x, origin_y))

    # Dump the output to the YAML file
    yaml_fn = fn_map + ".yaml"
    yaml = ("image: %s\n"
            "resolution: %f\n"
            "origin: [%f, %f, 0]\n"
            "negate: 0\n"
            "occupied_thresh: %f\n"
            "free_thresh: %f\n" % (fn_map + fn_ext, mpp, origin_x, origin_y,
                                   THRESH_OCCUPIED, THRESH_FREE))
    yaml_filename = os.path.join(fn_dir, yaml_fn)
    print("Dumping following yaml to: %s" % (yaml_filename))
    print("\t" + yaml.replace('\n', '\n\t', 5))
    yaml_file = open(yaml_filename, 'w')
    yaml_file.write(yaml)
    yaml_file.close()
    print("Done.")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise ValueError(
            "Yaml Generator needs exactly 1 argument (map filename)")
    main(sys.argv[1])
