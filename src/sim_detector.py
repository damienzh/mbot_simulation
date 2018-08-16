#! /usr/bin/env python

import cv2
import numpy as np
import copy


class SimDetector:
    def __init__(self):
        self.image = None

        self.blue_upper = np.array([255, 10, 10])
        self.blue_lower = np.array([80, 0, 0])
        self.green_upper = np.array([10, 255, 10])
        self.green_lower = np.array([0, 80, 0])
        self.red_upper = np.array([10, 10, 255])
        self.red_lower = np.array([0, 0, 80])

        self.boundingbox = None
        self.label = None
        self.found = False
        self.marked_image = None

    def detect_cube(self, img):
        self.image = img
        self.marked_image = copy.copy(self.image)
        red_mask = cv2.inRange(self.image, self.red_lower, self.red_upper)
        # self.showimg(red_mask)
        green_mask = cv2.inRange(self.image, self.green_lower, self.green_upper)
        # self.showimg(green_mask)
        blue_mask = cv2.inRange(self.image, self.blue_lower, self.blue_upper)
        # self.showimg(blue_mask)

        __, red_cnts, __ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        __, green_cnts, __ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        __, blue_cnts, __ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_cnt, red_area = self.find_max_cnt(red_cnts)
        green_cnt, green_area = self.find_max_cnt(green_cnts)
        blue_cnt, blue_area = self.find_max_cnt(blue_cnts)
        # print (red_area, green_area, blue_area)

        if max([red_area, green_area, blue_area]) > 1000:  # found something in layers
            self.found = True
            max_area_ind = np.argmax([red_area, green_area, blue_area])
            self.label = ['Red', 'Green', 'Blue'][max_area_ind]
            (x, y, w, h) = cv2.boundingRect([red_cnt, green_cnt, blue_cnt][max_area_ind])
            self.boundingbox = (x, y, w, h)
            # cv2.drawContours(self.image, [red_cnt[0], green_cnt[0], blue_cnt[0]][max_area_ind], -1, (0, 255, 255), 3)
            cv2.rectangle(self.marked_image, (x, y), (x + w, y + h), (0, 255, 255), 3)
            cv2.putText(self.marked_image, self.label, (self.boundingbox[0], self.boundingbox[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 2255, 255))
            # self.showimg(self.marked_image)
        return self.found

    def reset(self):
        self.image = None
        self.boundingbox = None
        self.label = None
        self.found = False
        self.marked_image = None

    def load_image(self, path):
        self.image = cv2.imread(path)

    def showimg(self, img):
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def calc_contours_area(self, cnts):
        areas = []
        for cnt in cnts:
            areas.append(cv2.contourArea(cnt))
        return areas

    def find_max_cnt(self, cnts):
        if len(cnts) > 0:
            areas = self.calc_contours_area(cnts)
            idx = np.argmax(areas)
            cnt = cnts[idx]
            area = areas[idx]
        else:
            cnt = [0]
            area = 0
        return cnt, area


if __name__ == '__main__':
    path = '/home/k/catkin_ws/src/mbot_simulation/test_data/sim_rgb_red.png'
    d = SimDetector()
    d.load_image(path)
    d.detect_cube(d.image)
    # cv2.imwrite('sim_rgb_red_boundingbox.png', d.marked_image)
