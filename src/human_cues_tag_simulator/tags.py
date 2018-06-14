import csv
import math
import rospy
import tf

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
import visualization_msgs.msg as visualization_msgs


def loadTags(fn):
    """Attempts to load existing tags from a provided tags file"""
    with open(fn, 'r') as tags_file:
        tags_reader = csv.reader(tags_file, delimiter=' ')
        tags = [{
            'id': int(row[0]),
            'x': float(row[1]),
            'y': float(row[2]),
            'th_deg': float(row[3])
        } for row in tags_reader]

    tags = sorted(tags, key=lambda x: x['id'])
    rospy.loginfo("Loaded the following tags from: %s" % (fn))
    for t in tags:
        rospy.loginfo("\t%s" % (tag2String(t)))
    return tags


def publishTagMarkers(tags, publisher):
    """Republishes the tag_markers topic"""
    SHIFT = 0.05
    h = std_msgs.Header(stamp=rospy.Time.now(), frame_id='map')
    ca = std_msgs.ColorRGBA(r=1, a=1)
    cb = std_msgs.ColorRGBA(g=1, a=1)
    ct = std_msgs.ColorRGBA(b=1, a=1)
    sa = geometry_msgs.Vector3(x=0.1, y=0.5, z=0.5)
    sb = geometry_msgs.Vector3(x=0.1, y=0.25, z=0.25)
    st = geometry_msgs.Vector3(x=1, y=1, z=1)
    ma = visualization_msgs.MarkerArray()
    ma.markers.append(
        visualization_msgs.Marker(
            header=h, id=-1, action=visualization_msgs.Marker.DELETEALL))
    for i, t in enumerate(tags):
        th = t['th_deg'] * math.pi / 180.
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        q = geometry_msgs.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        ma.markers.append(
            visualization_msgs.Marker(
                header=h,
                id=i * 3,
                type=visualization_msgs.Marker.CUBE,
                action=visualization_msgs.Marker.ADD,
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(x=t['x'], y=t['y']),
                    orientation=q),
                scale=sa,
                color=ca))
        ma.markers.append(
            visualization_msgs.Marker(
                header=h,
                id=i * 3 + 1,
                type=visualization_msgs.Marker.CUBE,
                action=visualization_msgs.Marker.ADD,
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(
                        x=t['x'] + SHIFT * math.cos(th),
                        y=t['y'] + SHIFT * math.sin(th)),
                    orientation=q),
                scale=sb,
                color=cb))
        ma.markers.append(
            visualization_msgs.Marker(
                header=h,
                id=i * 3 + 2,
                type=visualization_msgs.Marker.TEXT_VIEW_FACING,
                action=visualization_msgs.Marker.ADD,
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(x=t[1], y=t[2], z=0.5),
                    orientation=q),
                scale=st,
                color=ct,
                text=str(t['id'])))
    publisher.publish(ma)


def nextTagID(tags):
    """Returns the next tag ID given the list of tags"""
    return 0 if not tags else sorted(tags, key=lambda x: x['id'])[-1][0] + 1


def tag2String(tag):
    """Takes a tag placement, and turns it into a verbose string"""
    return ("Tag with ID #%d, was placed @ (%f,%f), facing %f deg" %
            (tag['id'], tag['x'], tag['y'], tag['th_deg']))


def writeTags(fn, tags):
    """Writes the current tags to the provided destination"""
    print('\n')
    with open(fn, 'wb') as tags_file:
        tags_writer = csv.writer(tags_file, delimiter=' ')
        for t in tags:
            tags_writer.writerow(t.values())
    rospy.loginfo("Wrote %d tags to: %s" % (len(tags), fn))
