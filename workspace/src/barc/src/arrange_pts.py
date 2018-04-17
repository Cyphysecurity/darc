import cv2
import numpy as np
'''
functions for rearranging pints of a rectangle of selecting them from checherboard corners
'''

def arrange_rect_pts(points):
    '''
    arranges corners of a rectangle into the following order
    .-------->X
    |  3    2
    |  0    1
    Y
    '''
    ordered_pts = np.zeros((4, 2), dtype = "float32")
    r_x_vals = [points[0][0], points[1][0], points[2][0], points[3][0]]
    r_y_vals = [points[0][1], points[1][1], points[2][1], points[3][1]]

    sort_x_idx = np.argsort(r_x_vals)
    # points 1 and 2
    max_x_a_idx = sort_x_idx[3]
    max_x_b_idx = sort_x_idx[2]
    if (r_y_vals[max_x_a_idx] > r_y_vals[max_x_b_idx]): # a is 1 and b is 2
        ordered_pts[1] = [r_x_vals[max_x_a_idx], r_y_vals[max_x_a_idx]]
        ordered_pts[2] = [r_x_vals[max_x_b_idx], r_y_vals[max_x_b_idx]]
    else: # b is 1 and a is 2
        ordered_pts[1] = [r_x_vals[max_x_b_idx], r_y_vals[max_x_b_idx]]
        ordered_pts[2] = [r_x_vals[max_x_a_idx], r_y_vals[max_x_a_idx]]

    # points 0 and 3
    min_x_a_idx = sort_x_idx[1]
    min_x_b_idx = sort_x_idx[0]
    if (r_y_vals[min_x_a_idx] > r_y_vals[min_x_b_idx]): # a is 0 and b is 3
        ordered_pts[0] = [r_x_vals[min_x_a_idx], r_y_vals[min_x_a_idx]]
        ordered_pts[3] = [r_x_vals[min_x_b_idx], r_y_vals[min_x_b_idx]]
    else: # b is 3 and a is 0
        ordered_pts[0] = [r_x_vals[min_x_b_idx], r_y_vals[min_x_b_idx]]
        ordered_pts[3] = [r_x_vals[min_x_a_idx], r_y_vals[min_x_a_idx]]

    return (ordered_pts)

def select_checkerboard_points(corners, cols, rows):
    '''
    selects the checkerboard corners to form the following rectangle
    .-------->X
    |  3    2
    |  0    1
    Y
    '''
    ordered_pts = np.zeros((4, 2), dtype = "float32")
    ordered_pts[3] = corners[0][0][0]
    ordered_pts[2] = corners[0][cols-1][0]
    ordered_pts[1] = corners[0][cols*rows-1][0]
    ordered_pts[0] = corners[0][cols*(rows-1)][0]
    ordered_pts = arrange_rect_pts(ordered_pts)
    return(ordered_pts)


