## Project: Search and Sample Return

**The goals / steps of this project are the following:**

**Training / Calibration**

* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

This is it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
And another!
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


In this section, I will go step by and show the changes I made to the scripts.

**The following items are in `perception.py` in `perception_step()` if otherwise not noted ~**
~~~
  # 1) Define source and destination points for perspective transform
  dst_size = 5
  bottom_offset = 6

  src = np.float32([[15, 140], [301, 140], [200, 96], [ 118, 96]])
  dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                ])
~~~
In this step, I created the image transform items based off of the example grid image below. Using the Jupyter test notebook, I found the pixel coordinates of the 1m square grid's corners

~~~
# 2) Apply perspective transform
r_warped = perspect_transform(Rover.img, src, dst)
~~~

This step simply transformed the image via the `perspect_transform()` function.

~~~
# 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
#define RGB values for thresholds
navi_rgb = (160, 160, 160)
rock_rgb = (120, 120, 80)

# get binary image, obstacle is defined as all space that is not navigable
navi_thresh, rock_thresh = color_thresh(r_warped, rgb_thresh=navi_rgb,rgb_rock = rock_rgb)
obst_thresh = 1 - navi_thresh
~~~

Based off of the test notebook, I utilized these threshold values for navigable terrain and rock detection. The obstacle image is simply the opposite of the navigable image. Since we are using a binary array to represent an image, this is accomplished by subtracting the navigable terrain binary array from 1.


In order to simplify the code, I added the rock threshold detection to `color_thresh()`; the code for that is below

~~~
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
~~~

One key difference between the rock threshold and the navigable terrain threshold is that the blue channel for the rock must be less than the value in the threshold, not above.

~~~
# 4) Update Rover.vision_image (this will be displayed on left side of screen)
Rover.vision_image[:,:,0] = obst_thresh * 255
Rover.vision_image[:,:,1] = rock_thresh * 255
Rover.vision_image[:,:,2] = navi_thresh * 255
~~~

The rover vision image is now updated based upon the thresholded images determined in the prior step

~~~
# 5) Convert map image pixel values to rover-centric coords
x_rover_img, y_rover_img = rover_coords(navi_thresh)
x_rock_img, y_rock_img = rover_coords(rock_thresh)
x_obs_img, y_obs_img = rover_coords(obst_thresh)
~~~

The `rover_coords()` function is used to convert the thresholded images to rover-centric coordinates

~~~
# 6) Convert rover-centric pixel values to world coordinates
# define scale and world size
world_size = Rover.worldmap.shape[0]
scale = 2 * dst_size

x_map, y_map = pix_to_world(x_rover_img, y_rover_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
x_rock, y_rock = pix_to_world(x_rock_img, y_rock_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
x_obs, y_obs = pix_to_world(x_obs_img, y_obs_img, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
~~~

Using `pix_to_world()`, the map values are created based on the scale, the rover-centric coordinates determined in the last step, and current rover information.

~~~
# 7) Update Rover worldmap (to be displayed on right side of screen)

# verify that the rover pitch and roll are within normal ranges before mapping
if (Rover.pitch < .04  or Rover.pitch > 359.06) and (Rover.roll < 1 or Rover.roll > 359):
    # set worldmap value of navigable terrain to 1
    Rover.worldmap[y_map, x_map, 2] = 1
    # set obstacle map value to 0 if determined to be navigable
    Rover.worldmap[y_map, x_map, 0] = 0

# if obstacle is in a pixel already determined to be navigable, ignore
Rover.worldmap[y_obs, x_obs, 0] = 1
Rover.worldmap[y_rock, x_rock, 1] = 1
~~~

To ensure good values are being stored in the map, the script determines if the rover pitch and roll are within normal values before mapping. The value for pitch is 0 +/- 0.1 (the vision image is very sensitive to pitch being off), and the value for roll is 0 +/- 0.7.

Once navigable terrain is mapped, it is not un-mapped. Only obstacle terrain can be overwritten by navigable terrain. This can lead to some issues in fidelity of demonstrating too much navigable terrain in the map, but in general it creates 80%+ fidelity maps

~~~
# 8) Convert rover-centric pixel positions to polar coordinates

# Update Rover pixel distances and angles
Rover.nav_dists, Rover.nav_angles = to_polar_coords(x_rover_img, y_rover_img)
Rover.rock_dists, Rover.rock_angles = to_polar_coords(x_rock_img, y_rock_img)
~~~

The pixel angles of navigable terrain and rocks are then calculated to create the steering angles in `decision_step()`

**The following items are in `decision.py` in `descision_step()` if otherwise not noted ~**



![alt text][image2]
#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

My settings to the Rover simulation program are as follow:
* Resolution - 1024 x 768 (windowed)
* Graphics quality - Good
* Output FPS - 22/25 (22 w/ video capture, 25 w/o)

Doing this, I was able to achieve an outcome of:
* 81.2% mapped
* 75.3% fidelity
* 5/6 rocks located
* 4/6 rocks picked up
* Elapsed time of 430.5 seconds

This outcome was the most 'optimal' in terms of balancing mapping, rock pickup, and elapsed time. The largest balancing act in the way that my current controls are set up are between the elapsed time and both mapping and rock location/pickup. I ran the simulator at other points and was able to achieve outcomes of 95%+ mapped/80%+ fidelity, with 6/6 rocks picked up. However, this would take in the range of 15-20 minutes.

To improve the balance between elapsed time and mapping, there are a few items that I could modify.

First, developing a technique to steer towards undiscovered terrain would reduce the time to map. Often, the rover will loop back around to terrain that has already been mapped, due to the visible terrain being a larger percentage of the rover's vision image. To address this, the rover would need to identify a steering angle that compared the navigable terrain to the map and weight the steering angle towards the unmapped terrain.

Second, developing a way to more intelligently pick the direction of turning when the rover is stopped could reduce the amount of time spent backtracking. A common event that increased the amount of time to map involved the rover backtracking after picking up a rock. One such way to complete this would be by holding the rover's travel path before it heads to pick up a rock. When completing the pickup, the rover could re-orient itself in the direction it had been traveling prior to picking up a rock

Third,

![alt text][image3]
