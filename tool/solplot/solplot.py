from collections import defaultdict
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
    map_scale = 5000
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

    output_file = None

    def __init__(self, rnet_file, prob_file, sol_file, output_file):
        self.output_file = output_file
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
            self.long_upper = max_long + 0.005
            self.long_lower = min_long - 0.005
            self.lat_upper = max_lat + 0.005
            self.lat_lower = min_lat - 0.005
            print('Longitude bounds:', self.long_lower, self.long_upper)
            print('Latitude bounds:', self.lat_lower, self.lat_upper)
            self.video_width = math.floor((self.long_upper - self.long_lower) * self.map_scale)
            self.video_height = math.floor((self.lat_upper - self.lat_lower) * self.map_scale)
            self.screen = np.full((self.video_height, self.video_width, 3), 255, dtype=np.uint8)
            print('Video resolution:', self.video_width, self.video_height)
            print('Start rendering background...')
            for k, v_list in self.edge.items():
                x_k = self.long2x(k[0])
                y_k = self.lat2y(k[1])
                for v in v_list:
                    x_v = self.long2x(v[0])
                    y_v = self.lat2y(v[1])
                    cv2.line(self.screen, (x_k, y_k), (x_v, y_v), (0, 0, 0), 1)
            print('Finish rendering background')
            cv2.imwrite('a.png', self.screen, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        except:
            print('Reading rnet file failed')
            raise

    def get_problem(self, prob_file):
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
                    # earyly time, origin, destination, picked
                    self.customer.append([e, o, d, 0])
                    self.customer_count = len(self.customer)

    def run_solution(self, sol_file):
        out = cv2.VideoWriter(self.output_file, cv2.VideoWriter_fourcc(*'MP4V'), 30.0, (self.video_width, self.video_height))
        # out = cv2.VideoWriter(self.output_file, 0x00000021, 30.0, (self.video_width, self.video_height))
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
                        x = self.long2x(self.node[int(col[i + 1])][0])
                        y = self.lat2y(self.node[int(col[i + 1])][1])
                        x_d = self.long2x(self.node[self.vehicle[i][2]][0])
                        y_d = self.lat2y(self.node[self.vehicle[i][2]][1])
                        if int(col[i + 1]) is not 0:
                            cv2.rectangle(canvas, (x - 5, y - 5), (x + 5, y + 5), (0, 255, 0),
                                          self.vehicle[i][3] + 1)
                        else:
                            cv2.rectangle(canvas, (x_d - 5, y_d - 5), (x_d + 5, y_d + 5), (255, 0, 0),
                                          self.vehicle[i][3] + 1)
                # draw customers
                col_base = self.vehicle_count + 1
                for i in range(0, self.customer_count):
                    if time_count >= self.customer[i][0]:
                        status = int(col[col_base + 2 * i])
                        x_o = self.long2x(self.node[self.customer[i][1]][0])
                        y_o = self.lat2y(self.node[self.customer[i][1]][1])
                        x_d = self.long2x(self.node[self.customer[i][2]][0])
                        y_d = self.lat2y(self.node[self.customer[i][2]][1])
                        if status is 3:
                            cv2.circle(canvas, (x_o, y_o), 3, (159, 152, 145), 2)
                        elif status is 0:
                            if int(col[col_base + 2 * i + 1]) is not 0:
                                cv2.circle(canvas, (x_o, y_o), 3, (0, 255, 0), 2)
                            else:
                                cv2.circle(canvas, (x_o, y_o), 3, (0, 0, 255), 2)
                        elif status is 1:
                            if self.customer[i][3] is 0:
                                self.vehicle[self.vehicle_map[int(col[col_base + 2 * i + 1])]][3] += 1
                                self.customer[i][3] = 1
                        elif status is 2:
                            cv2.circle(canvas, (x_d, y_d), 3, (255, 0, 0), 2)
                            if self.customer[i][3] is 1:
                                self.vehicle[self.vehicle_map[int(col[col_base + 2 * i + 1])]][3] -= 1
                                self.customer[i][3] = 0

                # cv2.imshow('show', canvas)
                # cv2.waitKey(100)
                out.write(canvas)

                time_count += 1

    def long2x(self, longitude):
        return math.floor((longitude - self.long_lower) * self.map_scale)

    def lat2y(self, latitude):
        return self.video_height - math.floor((latitude - self.lat_lower) * self.map_scale)


if __name__ == '__main__':
    args = vars(get_parser().parse_args())
    v = Visualizer(args['rnet_file'], args['prob_file'], args['sol_file'], args['out'])
