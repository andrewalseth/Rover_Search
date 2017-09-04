import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
# Threshold of R,G > 120 and B < 80 work well for determining rocks
def color_thresh(img, rgb_thresh=(160, 160, 160), rgb_rock=(120,120,80)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    rock_select = np.zeros_like(img[:,:,0])
    # Apply the thresholds for RGB and assign 1's
    # where threshold was exceeded
    # Return the single-channel binary image
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])

    rocks = (img[:,:,0] > rgb_rock[0]) \
            & (img[:,:,1] > rgb_rock[1]) \
            & (img[:,:,2] < rgb_rock[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    rock_select[rocks] = 1

    # Return the binary image
    return color_select, rock_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6

    src = np.float32([[15, 140], [301, 140], [200, 96], [ 118, 96]])
    dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    r_warped = perspect_transform(Rover.img, src, dst)


    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #define RGB values for thresholds
    navi_rgb = (160, 160, 160)
    rock_rgb = (120, 120, 80)

    # get binary image, obstacle is defined as all space that is not navigable
    navi_thresh, rock_thresh = color_thresh(r_warped, rgb_thresh=navi_rgb,rgb_rock = rock_rgb)
    obst_thresh = 1 - navi_thresh



    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obst_thresh * 255
    Rover.vision_image[:,:,1] = rock_thresh * 255
    Rover.vision_image[:,:,2] = navi_thresh * 255

    # 5) Convert map image pixel values to rover-centric coords
    x_rover_img, y_rover_img = rover_coords(navi_thresh)
    x_rock_img, y_rock_img = rover_coords(rock_thresh)
    x_obs_img, y_obs_img = rover_coords(obst_thresh)


    # 6) Convert rover-centric pixel values to world coordinates
    # define scale and world size
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size

    x_map, y_map = pix_to_world(x_rover_img, y_rover_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    x_rock, y_rock = pix_to_world(x_rock_img, y_rock_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    x_obs, y_obs = pix_to_world(x_obs_img, y_obs_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)

    # verify that the rover pitch and roll are within normal ranges before mapping
    if (Rover.pitch < .08  or Rover.pitch > 359.92) and (Rover.roll < 0.7 or Rover.roll > 359.3):
        # set worldmap value of navigable terrain to 1
        Rover.worldmap[y_map, x_map, 2] = 1
        # set obstacle map value to 0 if determined to be navigable
        Rover.worldmap[y_map, x_map, 0] = 0

    # if obstacle is in a pixel already determined to be navigable, ignore
    Rover.worldmap[y_obs, x_obs, 0] = 1
    Rover.worldmap[y_rock, x_rock, 1] = 1



    # 8) Convert rover-centric pixel positions to polar coordinates

    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(x_rover_img, y_rover_img)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(x_rock_img, y_rock_img)
    if Rover.rock_angles is None:
        Rover.rock_angles = 0




    return Rover
