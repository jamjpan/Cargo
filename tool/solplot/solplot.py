from collections import defaultdict
from PIL import ImageFont
import numpy as np
import argparse
import math
import cv2


def get_parser():
    parser = argparse.ArgumentParser(description='RSP Solution Visualization')
    parser.add_argument('rnet_file')
    parser.add_argument('prob_file')
    parser.add_argument('sol_file')
    parser.add_argument('-o', '--out', metavar='OUTPUT', help='output video path (default: a.avi)',
                        default='a.mp4')
    return parser


class Visualizer:
    fps = 30.0
    map_scale = 5000
    output_file = None
    draw_inactive = False
    # colors from https://materializecss.com/color.html
    load_color_map = [(196, 203, 128), (172, 182, 77), (154, 166, 38), (136, 150, 0),
                      (123, 137, 0), (107, 121, 0), (92, 105, 0), (64, 77, 0)]  # teal
    vehicle_finish_color = (139, 125, 96)  # cyan
    customer_timeout_color = (158, 158, 158)  # grey
    customer_waiting_color = (7, 193, 255)  # amber
    customer_assigned_color = (74, 195, 139)  # light green
    customer_dropped_color = (139, 125, 96)  # cyan
    road_color = (224, 224, 224)  # light grey
    customer_size = 5
    vehicle_size = 6
    road_size = 1

    node = {}
    edge = defaultdict(list)
    long_upper = 0
    long_lower = 0
    lat_upper = 0
    lat_lower = 0
    video_width = 0
    video_height = 0
    screen = None

    vehicle = []
    vehicle_count = 0
    vehicle_map = {}
    customer = []
    customer_count = 0

    def __init__(self, rnet_file, prob_file, sol_file, output_file, map_scale=5000,
                 draw_inactive=True, fps=30.0):
        self.output_file = output_file
        self.map_scale = map_scale
        self.draw_inactive = draw_inactive
        self.fps = fps
        self.get_background(rnet_file)
        self.get_problem(prob_file)
        self.run_solution(sol_file)

    def get_background(self, rnet_file):
        """ Generate road network background from rnet_file """
        min_long = 10000
        min_lat = 10000
        max_long = -10000
        max_lat = -10000
        try:
            with open(rnet_file) as f_rnet:
                for line in f_rnet:
                    col = line.strip().split()
                    # get picture bounds
                    longitude = float(col[3])
                    latitude = float(col[4])
                    if longitude < min_long:
                        min_long = longitude
                    if longitude > max_long:
                        max_long = longitude
                    if latitude < min_lat:
                        min_lat = latitude
                    if latitude > max_lat:
                        max_lat = latitude
                    self.node[int(col[1])] = key = (longitude, latitude)
                    longitude = float(col[5])
                    latitude = float(col[6])
                    if longitude < min_long:
                        min_long = longitude
                    if longitude > max_long:
                        max_long = longitude
                    if latitude < min_lat:
                        min_lat = latitude
                    if latitude > max_lat:
                        max_lat = latitude
                    self.node[int(col[2])] = value = (longitude, latitude)
                    self.edge[key].append(value)

            # video padding
            self.long_upper = max_long + 0.005
            self.long_lower = min_long - 0.005
            self.lat_upper = max_lat + 0.005
            self.lat_lower = min_lat - 0.005 - 100 / self.map_scale
            print('Longitude bounds:', self.long_lower, self.long_upper)
            print('Latitude bounds:', self.lat_lower, self.lat_upper)
            self.video_width = math.floor((self.long_upper - self.long_lower) * self.map_scale)
            self.video_height = math.floor((self.lat_upper - self.lat_lower) * self.map_scale)
            self.screen = np.full((self.video_height, self.video_width, 3), 255, dtype=np.uint8)
            print('Video resolution:', self.video_width, self.video_height)

            # draw road network
            for k, v_list in self.edge.items():
                x_k = self.long2x(k[0])
                y_k = self.lat2y(k[1])
                for v in v_list:
                    x_v = self.long2x(v[0])
                    y_v = self.lat2y(v[1])
                    cv2.line(self.screen, (x_k, y_k), (x_v, y_v), self.road_color, self.road_size)
            # draw legends
            x_start = 20
            y_start = self.video_height - 80
            text = 'Vehicle'
            # font = ImageFont.truetype(font='font/FiraMono-Medium.otf', size=1)
            # print(cv2.getTextSize(text, fontFace=font, fontScale=1))
            # print(cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=.5, thickness=1))
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20
            for i in range(0, 8):
                cv2.rectangle(self.screen, (x_start, y_start),
                              (x_start + self.vehicle_size * 2 + 1,
                               y_start + self.vehicle_size * 2 + 1),
                              self.load_color_map[i], cv2.FILLED)
                x_start += self.vehicle_size * 2 + 1
            x_start += 10
            text = 'load(0~7)'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20
            cv2.rectangle(self.screen, (x_start, y_start),
                          (x_start + self.vehicle_size * 2 + 1,
                           y_start + self.vehicle_size * 2 + 1),
                          self.vehicle_finish_color, cv2.FILLED)
            x_start += self.vehicle_size * 2 + 11
            text = 'inactive'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)

            x_start = 20
            y_start += text_height + 20
            text = 'Customer'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20

            # draw waiting
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_waiting_color, cv2.FILLED)
            x_start += self.customer_size + 10
            text = 'waiting'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20

            # draw assigned
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_assigned_color, cv2.FILLED)
            x_start += self.customer_size + 10
            text = 'assigned'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20

            # draw dropped
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_dropped_color, cv2.FILLED)
            x_start += self.customer_size + 10
            text = 'dropped'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20

            # draw timeout
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_timeout_color, cv2.FILLED)
            x_start += self.customer_size + 10
            text = 'timeout'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  .5, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, .5, (0, 0, 0), 1)
            x_start += text_width + 20
            # cv2.imwrite('a.png', self.screen, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        except BaseException:
            print('Reading rnet file failed')
            raise

    def get_problem(self, prob_file):
        """ Load vehicles and customers from problem instance """
        try:
            with open(prob_file) as f_prob:
                lines = f_prob.readlines()[6:]
                for line in lines:
                    col = line.split()
                    id, o, d, q, e = int(col[0]), int(col[1]), int(col[2]), int(col[3]), int(col[4])
                    if q < 0:
                        # early time, origin, destination, load
                        self.vehicle.append([e, o, d, 0])
                        self.vehicle_count = len(self.vehicle)
                        self.vehicle_map[id] = len(self.vehicle) - 1
                    else:
                        # early time, origin, destination, picked
                        self.customer.append([e, o, d, 0])
                        self.customer_count = len(self.customer)
        except BaseException:
            print('Reading prob file failed')
            raise

    def run_solution(self, sol_file):
        """ Run solution simulation and write it to output video """
        out = cv2.VideoWriter(self.output_file, cv2.VideoWriter_fourcc(*'MP4V'), self.fps,
                              (self.video_width, self.video_height))
        with open(sol_file) as f_sol:
            lines = f_sol.readlines()[8:]
            time_count = 0
            for line in lines:
                canvas = self.screen.copy()
                # canvas = self.screen
                col = line.split()
                # draw vehicles
                for i in range(0, self.vehicle_count):
                    if time_count >= self.vehicle[i][0]:
                        if int(col[i + 1]) is not -1:
                            # current position
                            x = self.long2x(self.node[int(col[i + 1])][0])
                            y = self.lat2y(self.node[int(col[i + 1])][1])
                            cv2.rectangle(canvas, (x - self.vehicle_size, y - self.vehicle_size),
                                          (x + self.vehicle_size, y + self.vehicle_size),
                                          self.load_color_map[self.vehicle[i][3]],
                                          cv2.FILLED)
                        else:
                            if self.draw_inactive:
                                continue
                            # destination position
                            x_d = self.long2x(self.node[self.vehicle[i][2]][0])
                            y_d = self.lat2y(self.node[self.vehicle[i][2]][1])
                            cv2.rectangle(canvas,
                                          (x_d - self.vehicle_size, y_d - self.vehicle_size),
                                          (x_d + self.vehicle_size, y_d + self.vehicle_size),
                                          self.vehicle_finish_color, cv2.FILLED)
                # draw customers
                col_base = self.vehicle_count + 1
                for i in range(0, self.customer_count):
                    if time_count >= self.customer[i][0]:
                        status = int(col[col_base + 2 * i])
                        # origin position
                        x_o = self.long2x(self.node[self.customer[i][1]][0])
                        y_o = self.lat2y(self.node[self.customer[i][1]][1])
                        # destination position
                        x_d = self.long2x(self.node[self.customer[i][2]][0])
                        y_d = self.lat2y(self.node[self.customer[i][2]][1])
                        if status is 3:
                            if self.draw_inactive:
                                continue
                            cv2.circle(canvas, (x_o, y_o), self.customer_size,
                                       self.customer_timeout_color,
                                       cv2.FILLED)
                        elif status is 0:
                            if int(col[col_base + 2 * i + 1]) is not 0:
                                cv2.circle(canvas, (x_o, y_o), self.customer_size,
                                           self.customer_assigned_color,
                                           cv2.FILLED)
                            else:
                                cv2.circle(canvas, (x_o, y_o), self.customer_size,
                                           self.customer_waiting_color,
                                           cv2.FILLED)
                        elif status is 1:
                            if self.customer[i][3] is 0:
                                self.vehicle[self.vehicle_map[int(col[col_base + 2 * i + 1])]][
                                    3] += 1
                                self.customer[i][3] = 1
                        elif status is 2:
                            cv2.circle(canvas, (x_d, y_d), self.customer_size,
                                       self.customer_dropped_color,
                                       cv2.FILLED)
                            if self.customer[i][3] is 1:
                                self.vehicle[self.vehicle_map[int(col[col_base + 2 * i + 1])]][
                                    3] -= 1
                                self.customer[i][3] = 0

                # display and write to video
                cv2.imshow('Visualizer', canvas)
                cv2.waitKey(10)
                out.write(canvas)
                time_count += 1

    def long2x(self, longitude):
        """ longitude to x coordinate in the video """
        return math.floor((longitude - self.long_lower) * self.map_scale)

    def lat2y(self, latitude):
        """ latitude to y coordinate in the video """
        return self.video_height - math.floor((latitude - self.lat_lower) * self.map_scale)


if __name__ == '__main__':
    args = vars(get_parser().parse_args())
    v = Visualizer(args['rnet_file'], args['prob_file'], args['sol_file'], args['out'],
                   map_scale=10000, draw_inactive=False)
