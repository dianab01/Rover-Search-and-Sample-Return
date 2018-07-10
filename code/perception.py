import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(185, 170, 160)):
    # Create an array of zeros same xy size as img, but single channel
    
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obstacle_color_thresh(color_threshed, mask):
    color_select = np.ones_like(color_threshed[:,:])
    color_select = (color_select - color_threshed) * mask
    return color_select

def rock_color_thresh(img, rgb_thresh=(150, 130, 30)):
    rock_color_select = np.zeros_like(img[:,:,0])
    between_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    rock_color_select[between_thresh] = 1
    return rock_color_select

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
    # create a mask to only further consider only the objects in the camera field of view
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped, mask

# trial to compensate for the Rover pitch and roll angles
def rotate_image(img, yaw, pitch, roll):
    R_y = np.array([[np.cos(pitch),    0,      np.sin(pitch)  ],
                    [0,                1,           0         ],
                    [-np.sin(pitch),   0,      np.cos(pitch)  ]])
    
    R_x = np.array([[1,         0,                  0       ],
                    [0,         np.cos(roll), -np.sin(roll) ],
                    [0,         np.sin(roll), np.cos(roll)  ]])
    R = np.dot( R_y, R_x )
    Rt = np.transpose(R)
    rotated = cv2.warpPerspective(img, Rt, (img.shape[1], img.shape[0]))
    return rotated
    

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    rock_threshed = rock_color_thresh(warped)
    obstacle_threshed = obstacle_color_thresh(threshed, mask)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = (obstacle_threshed.astype(dtype=np.float)*255)
    Rover.vision_image[:,:,2] = (threshed.astype(dtype=np.float)*255)
    
    threshed = threshed[np.int(threshed.shape[0]*0.6):threshed.shape[0],:]
    obstacle_threshed = obstacle_threshed[np.int(obstacle_threshed.shape[0]*0.6):obstacle_threshed.shape[0],:]
    
    # 5) Convert map image pixel values to rover-centric coords
    navigable_x_pix, navigable_y_pix = rover_coords(threshed)
    rock_x_pix, rock_y_pix = rover_coords(rock_threshed)
    obstacle_x_pix, obstacle_y_pix = rover_coords(obstacle_threshed)
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 2 * dst_size
    world_size = Rover.worldmap.shape[0]
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x_pix, navigable_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_x_pix, rock_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_pix, obstacle_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] = np.clip(Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] + 100, 0, 255)
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] = np.clip(Rover.worldmap[navigable_y_world, navigable_x_world, 2] + 50, 0, 255)
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_x_pix, navigable_y_pix)
    
    if(rock_threshed.any()):
        rock_dist, Rover.rock_angles = to_polar_coords(rock_x_pix, rock_y_pix)
        # select from the rock positions only the center values
        rock_center_index = np.argmin(rock_dist)
        rock_x_center = rock_x_world[rock_center_index]
        rock_y_center = rock_y_world[rock_center_index]
        # update the map only with the rock's center point
        Rover.worldmap[rock_y_center, rock_x_center] = 255
        Rover.vision_image[:,:,1] = (rock_threshed.astype(dtype=np.float)*255)
        Rover.rock_in_fov = True
    else:
        Rover.rock_in_fov = False
    
    return Rover
