from collections import defaultdict
from PIL import ImageFont
import numpy as np
import argparse
import math
import cv2

FPS=1.0

def get_parser():
    parser = argparse.ArgumentParser(description='RSP Solution Visualization')
    parser.add_argument('rnet_file')
    parser.add_argument('prob_file')
    parser.add_argument('sol_file')
    parser.add_argument('-o', '--out', metavar='OUTPUT', help='output video path (default: a.mp4)',
                        default='a.mp4')
    return parser


class Visualizer:
    map_scale = 5000
    output_file = None
    draw_inactive = False
    # colors from https://materializecss.com/color.html
    # and colorbrewer2.org
    load_color_map = [(180,117,69),(209,173,116),(233,217,171),(248,243,224),
            (144,224,254),(97,174,253),(67,109,244),(39,48,215)]
    #load_color_map = [(196, 203, 128), (172, 182, 77), (154, 166, 38), (136, 150, 0),
    #                  (123, 137, 0), (107, 121, 0), (92, 105, 0), (64, 77, 0)]  # teal
    background_color = (56, 50, 38)  # dark-green darken-4
    vehicle_finish_color = (139, 125, 96)  # cyan
    customer_timeout_color = (158, 158, 158)  # grey
    customer_waiting_color = (7, 193, 255)  # amber
    customer_assigned_color = (74, 195, 139)  # light green
    customer_dropped_color = (139, 125, 96)  # cyan
    # road_color = (224, 224, 224)  # light grey
    road_color = (117, 117, 117)  # grey darken-1
    text_color = (224, 224, 224)  # grey lighten-2
    customer_size = 4
    vehicle_size = 8
    vehicle_border_size = 0
    road_size = 1
    text_size = 1

    node = {}
    edge = defaultdict(list)
    long_upper = 0
    long_lower = 0
    lat_upper = 0
    lat_lower = 0
    video_width = 0
    video_height = 0
    screen = None

    vehicle = {}
    vehicle_count = 0
    vehicle_map = {}
    customer = {}
    customer_count = 0
    show_vehicle_route = []

    def __init__(self, rnet_file, prob_file, sol_file, output_file, map_scale=5000,
                 draw_inactive=True, fps=FPS):
        self.output_file = output_file
        self.map_scale = map_scale
        self.draw_inactive = draw_inactive
        self.fps = fps
        self.text_size = self.text_size * map_scale / 8000
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
            self.screen = np.full((self.video_height, self.video_width, 3), self.background_color,
                                  dtype=np.uint8)
            print('Video resolution:', self.video_width, self.video_height)

            # draw road network
            for k, v_list in self.edge.items():
                x_k = self.long2x(k[0])
                y_k = self.lat2y(k[1])
                for v in v_list:
                    x_v = self.long2x(v[0])
                    y_v = self.lat2y(v[1])
                    cv2.line(self.screen, (x_k, y_k), (x_v, y_v), self.road_color, self.road_size,
                             cv2.LINE_AA)

            # draw legends
            x_start = 20
            y_start = self.video_height - 80
            text = 'Vehicle'
            # font = ImageFont.truetype(font='font/FiraMono-Medium.otf', size=1)
            # print(cv2.getTextSize(text, fontFace=font, fontScale=1))
            # print(cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=.5, thickness=1))
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
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
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
            x_start += text_width + 20
            cv2.rectangle(self.screen, (x_start, y_start),
                          (x_start + self.vehicle_size * 2 + 1,
                           y_start + self.vehicle_size * 2 + 1),
                          self.vehicle_finish_color, cv2.FILLED)
            x_start += self.vehicle_size * 2 + 11
            text = 'inactive'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)

            x_start = 20
            y_start += text_height + 20
            text = 'Customer'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
            x_start += text_width + 20

            # draw waiting
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_waiting_color, cv2.FILLED, cv2.LINE_AA)
            x_start += self.customer_size + 10
            text = 'waiting'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
            x_start += text_width + 20

            # draw assigned
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_assigned_color, cv2.FILLED, cv2.LINE_AA)
            x_start += self.customer_size + 10
            text = 'assigned'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
            x_start += text_width + 20

            # draw dropped
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_dropped_color, cv2.FILLED, cv2.LINE_AA)
            x_start += self.customer_size + 10
            text = 'dropped'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
            x_start += text_width + 20

            # draw timeout
            cv2.circle(self.screen, (x_start, y_start + text_height // 2), self.customer_size,
                       self.customer_timeout_color, cv2.FILLED, cv2.LINE_AA)
            x_start += self.customer_size + 10
            text = 'timeout'
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_TRIPLEX,
                                                                  self.text_size, 1)
            cv2.putText(self.screen, text, (x_start, y_start + text_height),
                        cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1, cv2.LINE_AA)
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
                        # early time, origin, destination, load, route, position
                        self.vehicle[id] = Vehicle(e, o, d)
                        # self.vehicle_map[id] = len(self.vehicle) - 1
                    else:
                        # early time, origin, destination, status
                        self.customer[id] = Customer(e, o, d)
                self.vehicle_count = len(self.vehicle)
                self.customer_count = len(self.customer)
        except BaseException:
            print('Reading prob file failed')
            raise

    def update(self, action, data):
        if action == 'R':
            vid = int(data[0])
            self.vehicle[vid].r.clear()
            for index in range(1, len(data)):
                self.vehicle[vid].r.append(int(data[index]))
        elif action == 'V':
            for index in range(0, len(data), 2):
                self.vehicle[int(data[index])].p = int(data[index + 1])
        elif action == 'P':
            for cid in data:
                cid = int(cid)
                self.customer[cid].s = 2
                self.vehicle[self.customer[cid].m].l += 1
        elif action == 'D':
            for cid in data:
                cid = int(cid)
                self.customer[cid].s = 3
                self.vehicle[self.customer[cid].m].l -= 1
        elif action == 'T':
            for cid in data:
                self.customer[int(cid)].s = 4
        elif action == 'M':
            vid = int(data[0])
            for index in range(1, len(data)):
                cid = int(data[index])
                if cid < 0:
                    self.customer[cid].m = 0
                    self.customer[cid].s = 0
                    # TODO: remove
                else:
                    self.customer[cid].m = vid
                    self.customer[cid].s = 1
                    self.vehicle[vid].m.append(cid)
                    if len(self.vehicle[vid].m) == 6:
                        # print(vid)
                        self.show_vehicle_route.append(vid)
        elif action == 'A':
            for vid in data:
                vid = int(vid)
                self.vehicle[vid].s = 2
                if vid in self.show_vehicle_route:
                    self.show_vehicle_route.remove(vid)

    def run_solution(self, log_file):
        """ Run solution simulation and write it to output video """
        out = cv2.VideoWriter(self.output_file, cv2.VideoWriter_fourcc(*'MP4V'), self.fps,
                              (self.video_width, self.video_height))
        with open(log_file) as f_log:
            time_count = 0
            # for line in lines:
            line = f_log.readline()
            while line:
                line = line.strip()
                col = line.split(' ')
                if len(col) is 0:
                    break
                while col[0].isdigit() and int(col[0]) == time_count:
                    self.update(col[1], col[2:])
                    line = f_log.readline()
                    col = line.split(' ')

                canvas = self.screen.copy()
                # canvas = self.screen

                # draw routes
                for vid in self.show_vehicle_route:
                    route = self.vehicle[vid].r
                    if len(route) is 0:
                        continue
                    node_1 = route[0]
                    for index in range(1, len(route)):
                        node_2 = route[index]
                        a_x = self.long2x(self.node[node_1][0])
                        a_y = self.lat2y(self.node[node_1][1])
                        b_x = self.long2x(self.node[node_2][0])
                        b_y = self.lat2y(self.node[node_2][1])
                        cv2.line(canvas, (a_x, a_y), (b_x, b_y), (255, 255, 255), 1, cv2.LINE_AA)
                        node_1 = node_2

                # draw customers
                for cid in self.customer:
                    if time_count >= self.customer[cid].e:
                        status = self.customer[cid].s
                        o_x = self.long2x(self.node[self.customer[cid].o][0])
                        o_y = self.lat2y(self.node[self.customer[cid].o][1])
                        d_x = self.long2x(self.node[self.customer[cid].d][0])
                        d_y = self.lat2y(self.node[self.customer[cid].d][1])
                        if status == 0:
                            cv2.circle(canvas, (o_x, o_y), self.customer_size,
                                       self.customer_waiting_color, cv2.FILLED, cv2.LINE_AA)
                        elif status == 1:
                            cv2.circle(canvas, (o_x, o_y), self.customer_size,
                                       self.customer_assigned_color, cv2.FILLED, cv2.LINE_AA)
                        elif status == 2:
                            pass
                        elif status == 3:
                            cv2.circle(canvas, (d_x, d_y), self.customer_size,
                                       self.customer_dropped_color, cv2.FILLED, cv2.LINE_AA)
                        elif status == 4:
                            if self.draw_inactive:
                                cv2.circle(canvas, (o_x, o_y), self.customer_size,
                                           self.customer_timeout_color, cv2.FILLED, cv2.LINE_AA)

                # draw vehicles
                for vid in self.vehicle:
                    if time_count >= self.vehicle[vid].e:
                        status = self.vehicle[vid].s
                        if status == 2:
                            if self.draw_inactive:
                                d_x = self.long2x(self.node[self.vehicle[vid].d][0])
                                d_y = self.lat2y(self.node[self.vehicle[vid].d][1])
                                cv2.circle(canvas, (d_x, d_y), self.vehicle_size+self.vehicle_border_size,
                                           (255,255,255), cv2.FILLED, cv2.LINE_AA)
                                cv2.circle(canvas, (d_x, d_y), self.vehicle_size,
                                           self.vehicle_finish_color, cv2.FILLED, cv2.LINE_AA)
                                #cv2.rectangle(canvas,
                                #              (d_x - self.vehicle_size, d_y - self.vehicle_size),
                                #              (d_x + self.vehicle_size, d_y + self.vehicle_size),
                                #              self.vehicle_finish_color, cv2.FILLED)
                        else:
                            x = self.long2x(self.node[self.vehicle[vid].p][0])
                            y = self.lat2y(self.node[self.vehicle[vid].p][1])
                            if vid in self.show_vehicle_route:
                                cv2.circle(canvas, (x, y), self.vehicle_size+self.vehicle_border_size,
                                           (255,255,255), cv2.FILLED, cv2.LINE_AA)
                                cv2.circle(canvas, (x, y), self.vehicle_size,
                                           (0, 0, 255), cv2.FILLED, cv2.LINE_AA)
                                #cv2.rectangle(canvas,
                                #              (x - self.vehicle_size, y - self.vehicle_size),
                                #              (x + self.vehicle_size, y + self.vehicle_size),
                                #              (0, 0, 255), cv2.FILLED)
                            else:
                                cv2.circle(canvas, (x, y), self.vehicle_size+self.vehicle_border_size,
                                           (255,255,255), cv2.FILLED, cv2.LINE_AA)
                                cv2.circle(canvas, (x, y), self.vehicle_size,
                                           self.load_color_map[self.vehicle[vid].l], cv2.FILLED, cv2.LINE_AA)
                                #cv2.rectangle(canvas,
                                #              (x - self.vehicle_size, y - self.vehicle_size),
                                #              (x + self.vehicle_size, y + self.vehicle_size),
                                #              self.load_color_map[self.vehicle[vid].l], cv2.FILLED)

                # draw time
                text = 'Time: %d' % time_count
                (text_width, text_height), baseline = cv2.getTextSize(text,
                                                                      cv2.FONT_HERSHEY_TRIPLEX,
                                                                      self.text_size, 1)
                cv2.putText(canvas, text, (5, 5 + text_height),
                            cv2.FONT_HERSHEY_TRIPLEX, self.text_size, self.text_color, 1,
                            cv2.LINE_AA)

                # display and write to video
                # cv2.imshow('Visualizer', canvas)
                # cv2.waitKey(10)
                out.write(canvas)
                if time_count % 500 == 0:
                    print('processing ' + str(time_count))
                time_count += 1

    def long2x(self, longitude):
        """ longitude to x coordinate in the video """
        return math.floor((longitude - self.long_lower) * self.map_scale)

    def lat2y(self, latitude):
        """ latitude to y coordinate in the video """
        return self.video_height - math.floor((latitude - self.lat_lower) * self.map_scale)


class Vehicle:
    def __init__(self, e, o, d):
        super(Vehicle, self)
        self.e = e  # early time
        self.o = o  # origin
        self.d = d  # destination
        self.l = 0  # load
        self.r = list()  # route
        self.p = o  # position
        self.s = 0  # status 0 idle 1 running 2 finish
        self.m = []  # matched customers


class Customer:
    def __init__(self, e, o, d):
        super(Customer, self)
        self.e = e  # early time
        self.o = o  # origin
        self.d = d  # destination
        self.s = 0  # status 0 waiting 1 matched 2 picked 3 dropped 4 timeout
        self.m = 0  # matched vehicle


if __name__ == '__main__':
    args = vars(get_parser().parse_args())
    v = Visualizer(args['rnet_file'], args['prob_file'], args['sol_file'], args['out'],
                   map_scale=8000, draw_inactive=False)
