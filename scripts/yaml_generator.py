import glob
import os
from PIL import Image
import pdb
import sys

PPM_SUFFIX = '_ppm'

THRESH_OCCUPIED = 0.6
THRESH_FREE = 0.3


def main(map_filename):
    """Constructs a yaml file for a map image"""
    # Get the filename elements (including map name)
    fn_dir = os.path.abspath(os.path.dirname(map_filename))
    fn_map, fn_ext = os.path.splitext(os.path.basename(map_filename))

    # Extract the pixels per meter parameter
    ppm_search = os.path.join(fn_dir, fn_map + PPM_SUFFIX + '*')
    ppms = glob.glob(ppm_search)
    if not ppms:
        raise ValueError(
            "No pixel per meter (ppm) file was found while searching in: %s" %
            ppm_search)
    elif len(ppms) > 1:
        raise ValueError(
            "%d pixel per meter matches were found (%s)" % (len(ppms), ppms))
    print("Extracting the Pixels Per Meter conversion from: %s" % (ppms[0]))
    with open(ppms[0], 'r') as ppms_file:
        vs = [float(x) for x in ppms_file.read().split()]
        if len(vs) != 2:
            raise ValueError(
                "Found %d values in Pixel Per Meter file (requires 2)" %
                (len(vs)))
    ppm = vs[0] / vs[1]
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
