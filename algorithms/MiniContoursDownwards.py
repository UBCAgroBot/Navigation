import cv2
import numpy as np
import time
import operator
import sys
import math
from algorithms.utils import Lines

class MiniContoursDownwards():
    
    def __init__(self, config):
        
        #width and height of frame
        self.config = config
        self.WIDTH = config.frame_width
        self.HEIGHT = config.frame_length
        # smoothing kernel
        self.kernel = np.ones((5,5),np.float32)/25
        self.morphology_kernel = np.ones((9,9),np.float32)

        # thresholds for the color of the crop rows
        self.low_green = np.array(config.low_green)
        self.high_green = np.array(config.high_green)

        # random colors for drawing lines etc
        self.color_1 = (255, 255, 0) #blue
        self.color_2 = (200, 0, 255) #pink
        self.color_3 = (0,0,255) #red (removed points)
        self.contour_color = (0, 129, 255) #color for contours

        # parameters for best fit line
        self.max_vote = self.config.max_vote
        self.num_strips=self.config.num_strips
        self.lines_max=self.config.lines_max
        self.param=self.config.param
        self.reps=self.config.reps
        self.aeps=self.config.aeps

    def apply_filters(self, original_frame):
        # orginal_frame: BGR frame 
        # returns frame: filtered BGR frame

        frame = cv2.filter2D(original_frame, -1, self.kernel)
        frame = cv2.bilateralFilter(frame, 9, 10, 75)    

        return frame
    
    def get_centroids(self, mask, num_strips, show):

        # mask is a binary mask of the image
        # number of strips is the number of strips to divide the image into for finding centroids
        # returns list [(x1, y1), (x2, y2), ...] of all the centroids obtained
        
        strips = []
        width = int(mask.shape[0]/num_strips)
        for i in range (100):
            strips.append(mask[i*width:(i+1)*width, 0:mask.shape[1]])


        centroids = []
        for i, strip in enumerate(strips):
            contours, hierarchy = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            strip_centroids = []
            for contour in contours:
                M = cv2.moments(contour)
                try:
                    cX = int(M["m10"] / M["m00"])
                    strip_centroids.append((cX, strip.shape[0] * (i+0.5)))
                except:
                    pass
            centroids.append(strip_centroids)

        return centroids
    
    
    def get_center_hough_lines(self, frame, show, num_strips=60, lines_max=30, dist_type = cv2.DIST_L2, param=0, reps=0.01, aeps=0.01):
        # frame: BGR frame 
        # num_strips: number of strips for centroid calculation
        # other parameters required to calculate line of best fit through centroids using cv2.fitline
        # returns: 
        # frame: original frame with best fit lines drawn on
        # line: [[vx, vy, x, y]] of the line of best fit through all the centroids. [vx, vy] is a vector that describes the line, where x, y is a point on the line     
        
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self.low_green, self.high_green)
        mask = cv2.medianBlur(mask, 9)
        # mask = cv2.GaussianBlur(mask, (9,9), 10)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel)
        centroids = self.get_centroids(mask, num_strips=num_strips, show=show)
        
        points = np.zeros(mask.shape, dtype=np.uint8)
        # points = cv2.Mat.zeros(mask.shape[0], mask.shape[1], cv.CV_8UC3)
        points_vector = []
        
        # ret, thresh = cv2.threshold(mask, 0, 254, 0)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cnt = contours
        # cv2.drawContours(frame, cnt, -1, self.contour_color, 3)
        # cv2.fillPoly(frame, pts=cnt, color=(0, 255, 0))
        # for c in cnt:
        #     if cv2.contourArea(c) > 3000:
        #         ellipse = cv2.fitEllipse(c)
        #         cv2.ellipse(frame, ellipse, (255, 255, 255), 2)
                
        height, width = frame.shape[0], frame.shape[1]
        split_factor = 10
        segmented_points = [[] for _ in range(split_factor)]

        for i, strip_centroid in enumerate(centroids):
            for centroid in strip_centroid:
                x,y = centroid[0], centroid[1]

                # vertically split the points
                idx = int(x / width * split_factor)
                segmented_points[idx].append([int(x),int(y)])

                if show:
                    cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 3, self.color_1, -1) 
                    cv2.circle(mask, (int(centroid[0]), int(centroid[1])), 3, self.color_1, -1)
                points_vector.append([int(centroid[0]), int(centroid[1])])
                    
                        
        c_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        for point in points_vector:
            try:
                points[point[1]][point[0]] = 255
            except:
                pass
        
        #if there are less than 5 points in the vector, we don't draw the line since it will give a poor approximation of the actual crop row
        if len(points_vector) > 5:
            points_vector = np.array([points_vector])
            line = cv2.fitLine(points_vector,  
                                distType = dist_type,
                                param=param,
                                reps=reps,
                                aeps=aeps
                                )
            print(int(100* line[0]), int(100 * line[1]), int(line[2]), int(line[3]))
            #want to show the line
            if show:
                cv2.line(frame, (int(line[2]) - self.WIDTH*int(1000 * line[0]), int(line[3]) - self.HEIGHT*int(1000*line[1])), (int(line[2]) + self.WIDTH*int(1000 * line[0]), int(line[3]) + self.HEIGHT*int(1000 * line[1])), self.color_2, 3)

        if show:
            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.imshow('c_mask', c_mask)
            cv2.imshow('points', points)

        return frame, line


    def process_frame(self, original_frame, num_strips=60, show=False):
        
        # original_frame: BGR frame
        # returns frame: original_frame with the best fit line and centroids drawn on
        #         angle: angle between the best fit line and a line drawn vertically down the center of the screen
        frame = self.apply_filters(original_frame)
        
        frame, lines = self.get_center_hough_lines(frame,
                                                    show,
                                                    num_strips=num_strips,
                                                    lines_max=self.lines_max,
                                                    dist_type=cv2.DIST_L2,
                                                    param=self.param,
                                                    reps=self.reps,
                                                    aeps=self.aeps)

        #intersections = Lines.get_intersections(lines)

        #if not intersections:
            #return frame, None

        #x_points = [point[0] for point in intersections]
        #y_points = [point[1] for point in intersections]

        #calculate angle relative to a straight line downwards
        #ref_line = cv2.line(frame, (0,0), (0, self.HEIGHT), self.color_1, 0)
        
        angle = None

        return frame, angle
    