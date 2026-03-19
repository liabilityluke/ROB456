#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np
import os

# Your path planning code
try:
    import lab3.path_planning as path_planning
except:
    import path_planning as path_planning


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 1)
    axs.imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs.set_title("threshold image")

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs.plot(p[0], p[1], '.b', markersize=2)

    if robot_loc is not None:
        axs.plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
    if best_pt is not None:
        axs.plot(best_pt[0], best_pt[1], '*y', markersize=10)
    axs.axis('equal')

    # Implements a zoom - set zoom to 1.0 if no zoom
    width = im_threshhold.shape[1]
    height = im_threshhold.shape[0]

    axs.set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
    axs.set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    
    #for each of the four connected pixels, make sure they are not out of range, then see if they are free
    #if any are free, it is considered reachable.
    for connected_pixel in path_planning.four_connected(pix) :
        if connected_pixel[0] < im.shape[1] and connected_pixel[1] < im.shape[0] and connected_pixel[0] >= 0 and connected_pixel[0] >= 0 :
            if path_planning.is_free(im, (connected_pixel[0], connected_pixel[1])) :
                return True
    return False


def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return list of possible pixel (x,y) locations"""


    #store any points that are both unseen and reachable in a dictionary
    possible_locations = {}
    for row in range(im.shape[0]) :
        for collumn in range(im.shape[1]) :
            if path_planning.is_unseen(im, (collumn, row)):
                if is_reachable(im, (collumn, row)) :
                    possible_locations[(collumn, row)] = True

    #for each point in the dictionary, check if it has any neighbor that is
    #also in the dictionary. If so, put it in an array. This filters out any single
    #lonely points
    possible_locations_array = []

    for location in possible_locations.keys() :
        for connected_location in path_planning.four_connected(location) :
            if possible_locations.get(connected_location) != None :
                possible_locations_array.append(location)
                break

    return possible_locations_array
                

def find_best_point(im, possible_points : list, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from (list of tuples)
    @param robot_loc - location of the robot (in case you want to factor that in)
    """

    #choose the points that meet the criteria for a good point,
    #aka, must be neighbors with at least 3 free spaces, and no walls.
    acceptable_points = []
    for pt in possible_points :
        count_free = 0
        count_unseen = 0
        for ix in range(-1, 2):
            for iy in range(-1, 2):
                if pt[0] + ix < im.shape[1] and pt[1] + iy < im.shape[0] and pt[0] + ix >= 0 and pt[1] + iy >= 0 :
                    if path_planning.is_free(im, (pt[0] + ix, pt[1] + iy)):
                        count_free += 1
                    elif path_planning.is_unseen(im, (pt[0] + ix, pt[1] + iy)):
                        count_unseen += 1
        if count_free >= 3 and count_free + count_unseen == 9:
            acceptable_points.append(pt)

    #get the closest acceptable point that is at least 60 spaces away,
    #so that it doesn't just keep moving tiny distances
    closest_loc = (-1, -1)
    closest_dist = 100000000
    for location in acceptable_points :
        dist =  abs(robot_loc[0] - location[0]) + abs(robot_loc[1] - location[1])
        if 60 < dist < closest_dist :
            closest_dist = dist
            closest_loc = location
    
    #take the best unseen point and get one of it's free neighbors to return so that we can pathplan to it.
    for ix in range(-1, 2):
        for iy in range(-1, 2):
            if path_planning.is_free(im, (closest_loc[0] + ix, closest_loc[1] + iy)):
                return (closest_loc[0] + ix, closest_loc[1] + iy)

    #this will run if the best point has no free neighbors, which hopefully never happens because we already checked that it does.
    print("hmmm something went wrong... returning unseen point")
    return (-3, -3)


def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    #just choose every 20th point, unless that point lies within 10 of the last point,
    waypoints = []
    for i in range(1, len(path) - 10) :
        if i % 20 == 0 :
            waypoints.append(path[i])
    
    #put in the last point no matter what
    waypoints.append(path[-1])

    return waypoints


def test_unseen(im, pts):
    for pt in pts:
        count_free = 0
        count_unseen = 0
        for ix in range(-1, 2):
            for iy in range(-1, 2):
                if path_planning.is_free(im, (pt[0] + ix, pt[1] + iy)):
                    count_free += 1
                elif path_planning.is_unseen(im, (pt[0] + ix, pt[1] + iy)):
                    count_unseen += 1
        if count_free == 0 or count_unseen == 0:
            return False
    return True


def test_best(im, pt):
    """ Check that the selected point has at least 3 free neighbors"""
    count_free = 0
    count_unseen = 0
    for ix in range(-1, 2):
        for iy in range(-1, 2):
            if path_planning.is_free(im, (pt[0] + ix, pt[1] + iy)):
                count_free += 1
            elif path_planning.is_unseen(im, (pt[0] + ix, pt[1] + iy)):
                count_unseen += 1
    if count_free < 3:
        print("one!")
        return False
    if count_free + count_unseen != 9:
        print("two!")
        return False
    return True


if __name__ == '__main__':
    _, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (60, 40)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)



    assert test_unseen(im=im_thresh, pts=all_unseen)
    assert test_best(im=im_thresh, pt=best_unseen)

    plot_with_explore_points(im_thresh, zoom=1.0, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im_thresh, zoom=1.0, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt
    plt.show()

    print("Done")
