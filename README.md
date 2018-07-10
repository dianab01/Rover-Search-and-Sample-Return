## Project: Search and Sample Return
### Implementation of the first project  

---


**Steps taken in order to complete the project have been the following:**  

**Training / Calibration**  

* Run the Roversim simulator in "Training Mode" and familiarize with the commands
* Collect sample data for testing in the simulator
* Test given functions in the Jupyter notebook
* Create functions to detect golden rocks and obstacles in the image
* Apply image processing functions, such as perspective transformation, color thresholding for navigable terrain, rocks and obstacles within the `process_image()` functions in order to obtain a map of the environment superposed with the detections made
* Create a video in which frames recorded manually based on the `output_images` obtained at the previous step

**Autonomous Navigation / Mapping**

* Filling and adapting `perception.py` script with the image processing functions previously written
* Add states and commands within the `decision.py` script for autonomously driving the Rover in the simulated environment
* Modify and add `RoverState` class to match up with the newly required fields

[//]: # (Image References)

[image1]: ./misc/rock_section.png
[image2]: ./misc/fov_mask.png
[image3]: ./misc/output_image.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Notebook Analysis
#### 1. The imports and displaying some of the sample and recored images, a subsection of one of the images containing a rock has been taken.

![alt text][image1]

That will be further used in order to select more precisely the thresholds for detecting the rocks. Thus, the `get_rock_color_thresholds()` function has been implemented, which computes the histograms on the RGB channels and selects the most relevant data (the color values that appear more than once).

#### 2. Since a frame contains mostly irrelevant data for the trajectory control, tests have been made and only 60% from the original image's height has been kept for this reason.

#### 3. Modifying the perspective transform function to also return a binary mask of the Rover's field of view. 

![alt text][image2]

The mask is used to map as obstacles to the world only those parts of an image which are within the fov, thus obtaining a higher accuracy.

#### 4. Implementing the color threshold functions for determining the navigable path and rocks.

The corresponding threshold chosen by inspecting the color range. Moreover, the obstacles are detected from the binary navigable terrain image, on top of which the afore mentioned mask is applied.

#### 5. Converting to rover coordinates, applying rotations and translations to the rover to reflect the positioning on the actual world map and converting to world and polar coordinated.

#### 6. Uniting all the steps above in the `process_image()` function, that returns a mosaique output image.

The `output_image` displays the original image from the camera, the perspective trasnformation, as well as the map obtained by overlapping the navigable terrain in blue, obstacles in red and rocks as a white dot.

![alt text][image3]

### Autonomous Navigation and Mapping

#### 1. Add to `perception.py` script the thresholding and transformation functions.

Since the perspective transformation is not acurate enough when the roll and pitch angle are not close to zero, an approach to apply a rotation matrix to the original image has been attempted. However, a significant difference in the accuracy has not been observed, therefore the method has not been continued.

Within the `perception_step()` function, whenever the processed image contained a rock, its coordinates would be transformed to polar coords, and then only its center is added to the map, due to the fact that after the transformation its shape is elongated. At the same time, an inner variable of the RoverState class is updated depending on the frame containing or not a sample to collect.

#### 2. Added new states and conditons to the `decisions.py` script

A new state, `"collect_rock"` has been introduced, which the Rover enters whenever there is a rock in its field of view. In this state, the Rover is directed towards the position of the rock and it stops for collecting when it is near the sample.

#### 3. Added new variables to the `RoverState` class

#### 4. Driving the Rover autonomously in the simulator.
