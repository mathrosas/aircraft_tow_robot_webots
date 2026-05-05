import numpy as np
import math

def visualize_map(map_data):
        plt.figure(0)
        plt.imshow(map_data)
        plt.show()

def display_map(display, map):
    display.setColor(0xFFFFFF)
    for x in range(map.shape[0]):            
        for y in range(map.shape[1]):
            if ((map[x][y])>0.9):
                display.drawPixel(x,y)

def obstacle_distance(lidar):
        # get lidar range measurements removing the first and last 80 data returns
        ranges = np.array(lidar.getRangeImage())
        ranges[ranges == np.inf] = 100000
        ranges[ranges > 12.0] = 100000
        ranges = ranges[280:-280]
        # return closest_obstacle_distance
        closest_obstacle_distance = np.min(ranges)
        return closest_obstacle_distance

def world2map(xworld,yworld):
    xworld = max(min(xworld, 150), -150)
    yworld = max(min(yworld, 150), -150)
    
    pixelx = math.floor(xworld + 150)
    pixely = math.floor(150 - yworld)

    pixelx = max(min(pixelx, 299), 0)
    pixely = max(min(pixely, 299), 0)
    return [pixelx,pixely]

def map2world(pixelx,pixely):
    pixelx = max(min(pixelx, 299), 0)
    pixely = max(min(pixely, 299), 0)

    xworld = pixelx - 150
    yworld = 150 - pixely

    xworld = max(min(xworld, 150), -150)
    yworld = max(min(yworld, 150), -150)

    return [xworld,yworld]

def generate_wp():
    WP = []

    # turn left to the west wall
    y = -18
    for x in range(-134, -143, -2):
        WP.append((x,y))

    # turn left to the south wall
    x = -142
    y = -20
    WP.append((x,y))

    for y in range(-22, -143, -4):
        WP.append((x,y))

    # turn left to the east wall
    x = -140
    y = -142
    WP.append((x,y))

    for x in range(-138, 143, 4):
        WP.append((x,y))

    # turn left to the north wall
    x = 142
    y = -140
    WP.append((x,y))

    for y in range(-138, 143, 4):
        WP.append((x,y))

    # turn left to the east wall
    x = 140
    y = 142    
    WP.append((x,y))

    for x in range(138, -67, -4):
        WP.append((x,y))

    # turn left to right corner of hangar
    x = -66
    y = 140
    WP.append((x,y))

    for y in range(138, -17, -2):
        WP.append((x,y))

    # turn right to right side of hangar door
    x = -68
    y = -16
    WP.append((x,y))

    for x in range(-70, -89, -3):
        WP.append((x,y))

    # turn right to inside the hangar
    x = -88
    y = -14
    WP.append((x,y))
    
    for y in range(-14, -9, 3):
        WP.append((x,y))

    # turn right to east side of hangar
    x = -86
    y = -10
    WP.append((x,y))

    for x in range(-84, -73, 2):
        WP.append((x,y))

    # turn left to north side of hangar
    x = -74
    y = -8
    WP.append((x,y))

    for y in range(-6, 143, 2):
        WP.append((x,y))

    # turn left to west side of hangar
    x = -76
    y = 142
    WP.append((x,y))

    for x in range(-78, -141, -2):
        WP.append((x,y))

    # turn left to south side of hangar
    x = -140
    y = 140
    WP.append((x,y))

    for y in range(140, -11, -3):
        WP.append((x,y))

    # turn left to the left side of hangar door
    x = -138
    y = -10
    WP.append((x,y))

    for x in range(-136, -125, 2):
        WP.append((x,y))

    # turn right to outside hangar door
    x = -126
    y = -12
    WP.append((x,y))

    y = -16
    WP.append((x,y))

    return WP