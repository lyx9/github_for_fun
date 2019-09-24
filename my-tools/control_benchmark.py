# !/usr/bin/env python
##################################################################
#
# Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
#
##################################################################
"""
Authors: Dingfeng Guo
"""

from __future__ import division
import os
import re
import csv
import numpy as np
import prettytable as pt
import matplotlib.pyplot as plt


class BenchMark(object):
    """
    plot the steer controller info
    """
    def __init__(self):
        """
        init the steer info
        """
        self.raw_data_filename = "data/control_log_raw_data.txt"
        self.fields = ['current_index', 'position_x', 'position_y',
                       'station_error', 'path_remain', 'current_speed',
                       'reference_speed', 'speed_error',
                       'current_acc', 'reference_acc', 'mpc_out_acc',
                       'throttle', 'brake',
                       'lateral_error', 'current_heading', 'reference_heading',
                       'heading_error',
                       'heading_error_rate', 'reference_curvature',
                       'reference_curvature_filtered',
                       'steer_feedforward',
                       'steer_feedback', 'steer_output', 'process_time',
                       'steer_position', 'chassis_throttle',
                       'chassis_brake', 'steer_torque', 'driving_mode']
        tmp_string = ''
        for field in self.fields:
            tmp_string += field + ','
        self.header = 'I0000 00:00:00.000000 MPC_Control_Detail,' + tmp_string[0:-1]
        self.data_info = {}
        for field in self.fields:
            self.data_info[field] = []
        self.scene_stright_line = {}
        for field in self.fields:
            self.scene_stright_line[field] = []
        self.scene_right_turn = {}
        for field in self.fields:
            self.scene_right_turn[field] = []
        self.scene_turn = {}
        for field in self.fields:
            self.scene_turn[field] = []
        self.scene_u_turn = {}
        for field in self.fields:
            self.scene_u_turn[field] = []
        self.scene_brake = {}
        for field in self.fields:
            self.scene_brake[field] = []
        self.f_mean = 0.4
        self.f_rmse = 0.2
        self.f_max = 0.3
        self.f_outlier = 0.1
        self.f_pacc = 0.05
        self.f_nacc = 0.4
        self.f_jerk = 0.05
        self.f_pedal_switch = 0.5
        self.f_lat_acc = 0.3
        self.f_steer_switch = 0.5
        self.f_steer = 0.2
        self.f = {}
        self.f['straight'] = [0.5, 0.1, 0.3, 0.1]
        self.f['right_turn'] = [0.1, 0.1, 0.4, 0.4]
        self.f['u_turn'] = [0.2, 0.1, 0.6, 0.1]
        self.f['turn'] = [0.05, 0.05, 0.6, 0.3]
        self.f['brake'] = [0.7, 0.3, 0, 0]
        self.f_scene = {}
        self.f_scene['straight'] = 0.3
        self.f_scene['turn'] = 0.4
        self.f_scene['right_turn'] = 0.1
        self.f_scene['u_turn'] = 0.05
        self.f_scene['brake'] = 0.15
        self.index = 0
        station_mean_base = [0.5, 0.3, 0.3, 0.6, 0.2]
        station_max_base = [2, 0.5, 1, 0.5, 0.5]
        speed_mean_base = [0.5, 0.4, 0.4, 0.4, 0.3]
        speed_max_base = [1, 0.5, 0.5, 0.5, 1]
        lateral_mean_base = [0.05, 0.1, 0.1, 0.1, 0.05]
        lateral_max_base = [0.1, 0.2, 0.2, 0.2, 0.1]
        heading_mean_base = [0.03, 0.05, 0.1, 0.1, 0.05]
        heading_max_base = [0.05, 0.15, 0.2, 0.2, 0.05]
        self.station_mean_base = {}
        self.station_max_base = {}
        self.speed_mean_base = {}
        self.speed_max_base = {}
        self.lateral_mean_base = {}
        self.lateral_max_base = {}
        self.heading_mean_base = {}
        self.heading_max_base = {}
        for scene in ['straight', 'turn', 'right_turn', 'u_turn', 'brake']:
            self.station_mean_base[scene] = station_mean_base[self.index]
            self.station_max_base[scene] = station_max_base[self.index]
            self.speed_mean_base[scene] = speed_mean_base[self.index]
            self.speed_max_base[scene] = speed_max_base[self.index]
            self.lateral_mean_base[scene] = lateral_mean_base[self.index]
            self.lateral_max_base[scene] = lateral_max_base[self.index]
            self.heading_mean_base[scene] = heading_mean_base[self.index]
            self.heading_max_base[scene] = heading_max_base[self.index]
            self.index += 1
        self.station_rmse_base = 0.2
        self.speed_rmse_base = 0.2
        self.lateral_rmse_base = 0.05
        self.heading_rmse_base = 0.02
        self.station_outlier_base = 1
        self.speed_outlier_base = 1
        self.lateral_outlier_base = 1
        self.heading_outlier_base = 1
        self.positive_acc_num_base = 0.001
        self.negative_acc_num_base = 0.01
        self.jerk_mean_base = 0.05
        self.pedal_switch_base = 0.1
        self.lateral_max_acc_base = 1
        self.lateral_delta_max_acc_base = 0.05
        self.delta_steer_mean_base = 0.01
        self.delta_steer_max_base = 5
        self.switch_number_base = 0.001
        self.switch_mean_base = 10

    def load_data(self):
        """
        load the data
        """
        filenames = []
        with open('data/data_name', 'r') as rf:
            for line in rf:
                filenames.append(line.split()[0])
        print("control log files include: " + str(filenames))

        if os.path.exists(self.raw_data_filename):
            shell_cmd = "rm " + self.raw_data_filename
            os.system(shell_cmd)

        for filename in filenames:
            shell_cmd = "grep MPC_Control_Detail " + filename +\
                        " >> " + self.raw_data_filename
            os.system(shell_cmd)
            print("load data done: " + filename)

        with open(self.raw_data_filename, 'r+') as rp:
            first_line = rp.readline()
            other = rp.read()
            if not re.search("current_index", first_line):
                rp.seek(0, 0)
                rp.write(str(self.header) + '\n' + other)

        with open(self.raw_data_filename, 'r') as fp:
            reader = csv.DictReader(fp)
            for line in reader:
                try:
                    if float(line['driving_mode']) == 1:
                        for field in self.fields:
                            self.data_info[field].append(float(line[field]))
                except Exception:
                    continue

    def cutoff_scene(self):
        """
        get different scene
        """
        num = len(self.data_info['driving_mode'])
        for index in range(num):
            if self.data_info['current_speed'][index] > 5 and\
               abs(self.data_info['steer_position'][index]) < 5 and\
               abs(self.data_info['lateral_error'][index]) < 0.3 and\
               abs(self.data_info['station_error'][index]) < 4:
                self.extract_data(index, self.scene_stright_line)
            if self.data_info['current_speed'][index] < 5 and\
               self.data_info['reference_curvature_filtered'][index] < -0.05 and\
               self.data_info['steer_position'][index] < -10 and\
               abs(self.data_info['lateral_error'][index]) < 0.5:
                self.extract_data(index, self.scene_right_turn)
            if self.data_info['current_speed'][index] > 5 and\
               abs(self.data_info['reference_curvature_filtered'][index]) > 0.005 and\
               abs(self.data_info['steer_position'][index]) > 5 and\
               abs(self.data_info['lateral_error'][index]) < 0.5:
                self.extract_data(index, self.scene_turn)
            if self.data_info['current_speed'][index] < 5 and\
               abs(self.data_info['reference_curvature_filtered'][index]) > 0.08 and\
               abs(self.data_info['steer_position'][index]) > 10 and\
               abs(self.data_info['lateral_error'][index]) < 0.5:
                self.extract_data(index, self.scene_u_turn)
            if self.data_info['current_speed'][index] < 2 and\
               abs(self.data_info['reference_curvature_filtered'][index]) < 0.01 and\
               abs(self.data_info['steer_position'][index]) < 5 and\
               abs(self.data_info['brake'][index]) > 20 and\
               abs(self.data_info['station_error'][index]) < 4 and\
               abs(self.data_info['lateral_error'][index]) < 0.3:
                self.extract_data(index, self.scene_brake)

    def extract_data(self, index, scene):
        """
        extract the data
        """
        scene['station_error'].append(
            self.data_info['station_error'][index])
        scene['speed_error'].append(
            self.data_info['speed_error'][index])
        scene['lateral_error'].append(
            self.data_info['lateral_error'][index])
        scene['heading_error'].append(
            self.data_info['heading_error'][index])
        scene['current_acc'].append(
            self.data_info['current_acc'][index])
        scene['throttle'].append(
            self.data_info['throttle'][index])
        scene['brake'].append(
            self.data_info['brake'][index])
        scene['current_speed'].append(
            self.data_info['current_speed'][index])
        scene['reference_curvature_filtered'].append(
            self.data_info['reference_curvature_filtered'][index])
        scene['steer_output'].append(
            self.data_info['steer_output'][index])
        return scene

    def divid_number(self, base, data, number=1):
        """
        clamp the data
        """
        if data == 0:
            out1 = 1
        else:
            out1 = base / data / number
        if out1 < 0:
            out = 0
        elif out1 > 1:
            out = 1
        else:
            out = out1
        return out

    def compute_score(self, scene, scene_name):
        """
        process the data and compute the score
        """
        prev_acc = 0
        prev_cmd = 0
        prev_acc_lat = 0
        prev_steer = 0
        number = 0
        num_station = 0
        num_speed = 0
        num_lateral = 0
        num_heading = 0
        num_pacc = 0
        num_nacc = 0
        jerk = []
        num_jerk = 0
        num_cmd_switch = 0
        num_steer = 0
        delta_switch_list = []
        delta_steer_list = []
        score = []
        acc_lat_list = []
        delta_acc_lat_list = []
        result = {}
        number = 0.01 * len(scene['station_error'])
        if number < 5:
            result['score'] = ['N/A', 'N/A', 'N/A', 'N/A']
            result['error'] = ['N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A',
                               'N/A', 'N/A', 'N/A', 'N/A']
            return result
        else:
            e_s_mean = round(np.mean(np.absolute(scene['station_error'])), 3)
            e_s_sd = round(np.std(np.absolute(scene['station_error'])), 3)
            e_s_max = round(np.max(np.absolute(scene['station_error'])), 3)
            e_v_mean = round(np.mean(np.absolute(scene['speed_error'])), 3)
            e_v_sd = round(np.std(np.absolute(scene['speed_error'])), 3)
            e_v_max = round(np.max(np.absolute(scene['speed_error'])), 3)

            e_l_mean = round(np.mean(np.absolute(scene['lateral_error'])), 3)
            e_l_sd = round(np.std(np.absolute(scene['lateral_error'])), 3)
            e_l_max = round(np.max(np.absolute(scene['lateral_error'])), 3)
            e_th_mean = round(np.mean(np.absolute(scene['heading_error'])), 3)
            e_th_sd = round(np.std(np.absolute(scene['heading_error'])), 3)
            e_th_max = round(np.max(np.absolute(scene['heading_error'])), 3)

            for error in np.absolute(scene['station_error']):
                if abs(error - e_s_mean) / e_s_sd > 3:
                    num_station += 1
            for error in scene['speed_error']:
                if abs(error - e_v_mean) / e_v_sd > 3:
                    num_speed += 1

            for error in np.absolute(scene['lateral_error']):
                if abs(error - e_l_mean) / e_l_sd > 3:
                    num_lateral += 1
            for error in np.absolute(scene['heading_error']):
                if abs(error - e_th_mean) / e_th_sd > 3:
                    num_heading += 1

            station_99_percent = 1 - num_station / len(scene['station_error'])
            speed_99_percent = 1 - num_speed / len(scene['speed_error'])
            lateral_99_percent = 1 - num_lateral / len(scene['lateral_error'])
            heading_99_percent = 1 - num_heading / len(scene['heading_error'])
            for acc in scene['current_acc']:
                if acc > 3:
                    num_pacc += 1
                if acc < -2:
                    num_nacc += 1
                delta_acc = acc - prev_acc
                jerk.append(delta_acc)
                if delta_acc > 0.05:
                    num_jerk += 1
                prev_acc = acc

            jerk_mean = round(np.mean(np.absolute(jerk)), 3)

            for throttle, brake in zip(scene['chassis_throttle'],
                                       scene['chassis_brake']):
                cmd = (throttle - 18) - (brake - 15.5)
                if cmd * prev_cmd < 0:
                    num_cmd_switch += 1
                prev_cmd = cmd

            for speed, curvature in zip(scene['current_speed'],
                                        scene[
                                        'reference_curvature_filtered']):
                acc_lat = speed * speed * curvature
                acc_lat_list.append(acc_lat)
                delta_acc_lat = acc_lat - prev_acc_lat
                delta_acc_lat_list.append(delta_acc_lat)
                prev_acc_lat = acc_lat
            acc_lat_max = round(np.max(np.absolute(acc_lat_list)), 3)
            delta_acc_lat_max = round(np.max(
                np.absolute(delta_acc_lat_list)), 3)

            for steer in scene['steer_output']:
                delta_steer = steer - prev_steer
                delta_steer_list.append(delta_steer)
                if steer * prev_steer < 0 and\
                   abs(steer) > 3 and\
                   abs(prev_steer) > 3:
                    num_steer += 1
                    delta_switch_list.append(delta_steer)
                prev_steer = steer

            if delta_steer_list == []:
                delta_steer_mean = 0
                delta_steer_max = 0
            else:
                delta_steer_mean = round(np.mean(
                    np.absolute(delta_steer_list)), 3)
                delta_steer_max = round(np.max(
                    np.absolute(delta_steer_list)), 3)
            if delta_switch_list == []:
                delta_switch_mean = 0
            else:
                delta_switch_mean = round(np.mean(
                    np.absolute(delta_switch_list)), 3)

            score_outlier_long = \
                self.divid_number(self.station_outlier_base, num_station, number) +\
                self.divid_number(self.speed_outlier_base, num_speed, number)
            score_outlier_lat = \
                self.divid_number(self.lateral_outlier_base, num_lateral, number) +\
                self.divid_number(self.heading_outlier_base,
                                  num_heading, number)

            long_algo_score = ((self.divid_number(self.station_mean_base[scene_name], e_s_mean) +
                               self.divid_number(self.speed_mean_base[scene_name], e_v_mean)) *
                               self.f_mean / 2 +
                               (self.divid_number(self.station_rmse_base, e_s_sd) +
                               self.divid_number(self.speed_rmse_base, e_v_sd)) *
                               self.f_rmse / 2 +
                               (self.divid_number(self.station_max_base[scene_name], e_s_max) +
                               self.divid_number(self.speed_max_base[scene_name], e_v_max)) *
                               self.f_max / 2 +
                               score_outlier_long * self.f_outlier / 2) * 100
            lat_algo_score = ((self.divid_number(self.lateral_mean_base[scene_name], e_l_mean) +
                               self.divid_number(self.heading_mean_base[scene_name], e_th_mean)) *
                              self.f_mean / 2 +
                              (self.divid_number(self.lateral_rmse_base, e_l_sd) +
                               self.divid_number(self.heading_rmse_base, e_th_sd)) *
                              self.f_rmse / 2 +
                              (self.divid_number(self.lateral_max_base[scene_name], e_l_max) +
                               self.divid_number(self.heading_max_base[scene_name], e_th_max)) *
                              self.f_max / 2 + score_outlier_lat *
                              self.f_outlier / 2) * 100
            long_comfor_score = (self.divid_number(self.positive_acc_num_base,
                                 num_pacc, number) * self.f_pacc +
                                 self.divid_number(self.negative_acc_num_base,
                                 num_nacc, number) * self.f_nacc +
                                 self.divid_number(self.jerk_mean_base,
                                                   jerk_mean) * self.f_jerk +
                                 self.divid_number(self.pedal_switch_base,
                                 num_cmd_switch, number) *
                                 self.f_pedal_switch) * 100

            lat_comfort_score = ((self.divid_number(self.lateral_max_acc_base,
                                 acc_lat_max) +
                                 self.divid_number(self.lateral_delta_max_acc_base,
                                 delta_acc_lat_max)) *
                                 self.f_lat_acc / 2 +
                                 (self.divid_number(self.delta_steer_mean_base,
                                  delta_steer_mean) +
                                  self.divid_number(self.delta_steer_max_base,
                                  delta_steer_max)) * self.f_steer / 2 +
                                 (self.divid_number(self.switch_number_base,
                                  num_steer, number) +
                                  self.divid_number(self.switch_mean_base,
                                  delta_switch_mean)) *
                                 self.f_steer_switch / 2) * 100
            score = [round(long_algo_score, 3), round(long_comfor_score, 3),
                     round(lat_algo_score, 3), round(lat_comfort_score, 3)]
            error = [round(e_s_mean, 3), round(e_s_max, 3), round(e_v_mean, 3),
                     round(e_v_max, 3), round(e_l_mean, 3), round(e_l_max, 3),
                     round(e_th_mean, 3), round(e_th_max, 3), round(station_99_percent, 3),
                     round(speed_99_percent, 3),
                     round(lateral_99_percent, 3), round(heading_99_percent, 3)]
            result['score'] = score
            result['error'] = error
        return result

    def print_result(self, score_stright, score_right_turn,
                     score_turn, score_u_turn, score_brake):
        """
        print the result
        """
        score_list = {}
        total_score = {}
        score_list['straight'] = score_stright
        score_list['right_turn'] = score_right_turn
        score_list['turn'] = score_turn
        score_list['u_turn'] = score_u_turn
        score_list['brake'] = score_brake
        total_score_all = 0
        count = 0
        count_factor = 0
        total_score_long_algo = 0
        total_score_long_comfort = 0
        total_score_lat_algo = 0
        total_score_lat_comfort = 0
        for scene in ['straight', 'right_turn', 'turn',
                      'u_turn', 'brake']:
            if score_list[scene][0] == 'N/A':
                total_score[scene] = 'N/A'
                print ('Lack of Scene:' + str(scene))
            else:
                count += 1
                total_score[scene] = self.f[scene][0] * score_list[scene][0] +\
                    self.f[scene][1] * score_list[scene][1] +\
                    self.f[scene][2] * score_list[scene][2] +\
                    self.f[scene][3] * score_list[scene][3]
                total_score[scene] = round(total_score[scene], 3)
                total_score_long_algo += self.f[scene][0] *\
                    score_list[scene][0]
                total_score_long_comfort += self.f[scene][1] *\
                    score_list[scene][1]
                total_score_lat_algo += self.f[scene][2] * score_list[scene][2]
                total_score_lat_comfort += self.f[scene][3] *\
                    score_list[scene][3]
                total_score_all += self.f_scene[scene] * total_score[scene]
                count_factor += self.f_scene[scene]
        if count == 0:
            print ('Lack of All Scene')
        else:
            total_score_all = round(total_score_all / count_factor, 3)
            total_score_long_algo = round(total_score_long_algo / count, 3)
            total_score_long_comfort = round(total_score_long_comfort / count, 3)
            total_score_lat_algo = round(total_score_lat_algo / count, 3)
            total_score_lat_comfort = round(total_score_lat_comfort / count, 3)

        tb3 = pt.PrettyTable()
        tb3.field_names = ['Scene', 'long_algo_score', 'long_comfort_score',
                           'lat_algo_score', 'lat_comfort_score',
                           'Total Score']
        tb3.add_row(['Straight Line', score_stright[0],
                     score_stright[1], score_stright[2],
                     score_stright[3],
                     total_score['straight']])
        tb3.add_row(['Right Turn', score_right_turn[0],
                     score_right_turn[1],
                     score_right_turn[2],
                     score_right_turn[3],
                     total_score['right_turn']])
        tb3.add_row(['Turn', score_turn[0],
                     score_turn[1], score_turn[2],
                     score_turn[3], total_score['turn']])
        tb3.add_row(['U turn', score_u_turn[0],
                     score_u_turn[1], score_u_turn[2],
                     score_u_turn[3], total_score['u_turn']])
        tb3.add_row(['Brake', score_brake[0],
                     score_brake[1], score_brake[2],
                     score_brake[3], total_score['brake']])
        tb3.add_row(['Total', total_score_long_algo,
                     total_score_long_comfort,
                     total_score_lat_algo,
                     total_score_lat_comfort, total_score_all])
        print(tb3)

    def plot_result(self, error_straight, error_right_turn,
                    error_turn, error_u_turn, error_brake):
        """
        plot the result
        """
        error_list = {}
        error_list['straight'] = error_straight
        error_list['right_turn'] = error_right_turn
        error_list['turn'] = error_turn
        error_list['u_turn'] = error_u_turn
        error_list['brake'] = error_brake
        for scene in ['straight', 'right_turn', 'turn', 'u_turn', 'brake']:
            if error_list[scene][0] == 'N/A':
                error_list[scene] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            else:
                continue
        fig = plt.figure()
        name_list = ['', 'straight line', 'right turn', 'turn', 'u turn', 'brake']
        station_error_mean = [error_list['straight'][0], error_list['right_turn'][0],
                              error_list['turn'][0], error_list['u_turn'][0],
                              error_list['brake'][0]]
        station_error_max = [error_list['straight'][1], error_list['right_turn'][1],
                             error_list['turn'][1],
                             error_list['u_turn'][1], error_list['brake'][1]]
        speed_error_mean = [error_list['straight'][2], error_list['right_turn'][2],
                            error_list['turn'][2],
                            error_list['u_turn'][2], error_list['brake'][2]]
        speed_error_max = [error_list['straight'][3], error_list['right_turn'][3],
                           error_list['turn'][3],
                           error_list['u_turn'][3], error_list['brake'][3]]
        lateral_error_mean = [error_list['straight'][4], error_list['right_turn'][4],
                              error_list['turn'][4],
                              error_list['u_turn'][4], error_list['brake'][4]]
        lateral_error_max = [error_list['straight'][5], error_list['right_turn'][5],
                             error_list['turn'][5],
                             error_list['u_turn'][5], error_list['brake'][5]]
        heading_error_mean = [error_list['straight'][6], error_list['right_turn'][6],
                              error_list['turn'][6],
                              error_list['u_turn'][6], error_list['brake'][6]]
        heading_error_max = [error_list['straight'][7], error_list['right_turn'][7],
                             error_list['turn'][7],
                             error_list['u_turn'][7], error_list['brake'][7]]

        tb = pt.PrettyTable()
        tb.field_names = ['Error Statistic Index', 'Straight Line', 'Right Turn',
                          'Turn', 'U Turn', 'Brake']
        tb.add_row(['Station Error Max', error_list['straight'][1], error_list['right_turn'][1],
                    error_list['turn'][1],
                    error_list['u_turn'][1], error_list['brake'][1]])
        tb.add_row(['Station Error Mean', error_list['straight'][0], error_list['right_turn'][0],
                    error_list['turn'][0], error_list['u_turn'][0],
                    error_list['brake'][0]])
        tb.add_row(['Station Error 99%', error_list['straight'][8], error_list['right_turn'][8],
                    error_list['turn'][8], error_list['u_turn'][8],
                    error_list['brake'][8]])
        tb.add_row(['Speed Error Max', error_list['straight'][3], error_list['right_turn'][3],
                    error_list['turn'][3],
                    error_list['u_turn'][3], error_list['brake'][3]])
        tb.add_row(['Speed Error Mean', error_list['straight'][2], error_list['right_turn'][2],
                    error_list['turn'][2],
                    error_list['u_turn'][2], error_list['brake'][2]])
        tb.add_row(['Speed Error 99%', error_list['straight'][9], error_list['right_turn'][9],
                    error_list['turn'][9],
                    error_list['u_turn'][9], error_list['brake'][9]])
        tb.add_row(['Lateral Error Max', error_list['straight'][5], error_list['right_turn'][5],
                    error_list['turn'][5],
                    error_list['u_turn'][5], error_list['brake'][5]])
        tb.add_row(['Lateral Error Mean', error_list['straight'][4], error_list['right_turn'][4],
                    error_list['turn'][4],
                    error_list['u_turn'][4], error_list['brake'][4]])
        tb.add_row(['Lateral Error 99%', error_list['straight'][10], error_list['right_turn'][10],
                    error_list['turn'][10],
                    error_list['u_turn'][10], error_list['brake'][10]])
        tb.add_row(['Heading Error Max', error_list['straight'][7], error_list['right_turn'][7],
                    error_list['turn'][7],
                    error_list['u_turn'][7], error_list['brake'][7]])
        tb.add_row(['Heading Error Mean', error_list['straight'][6], error_list['right_turn'][6],
                    error_list['turn'][6],
                    error_list['u_turn'][6], error_list['brake'][6]])
        tb.add_row(['Heading Error 99%', error_list['straight'][11], error_list['right_turn'][11],
                    error_list['turn'][11],
                    error_list['u_turn'][11], error_list['brake'][11]])
        print(tb)

        station_mean_base = [0.5, 0.3, 0.3, 0.6, 0.2]
        station_max_base = [2, 0.5, 1, 0.5, 0.5]
        speed_mean_base = [0.5, 0.4, 0.4, 0.4, 0.3]
        speed_max_base = [1, 0.5, 0.5, 0.5, 1]
        lateral_mean_base = [0.05, 0.1, 0.1, 0.1, 0.05]
        lateral_max_base = [0.2, 0.3, 0.3, 0.3, 0.2]
        heading_mean_base = [0.03, 0.05, 0.05, 0.05, 0.01]
        heading_max_base = [0.05, 0.05, 0.05, 0.1, 0.05]
        size = 5
        x = np.arange(size)
        total_width, n = 0.7, 2
        width = total_width / n
        x = x - (total_width - width) / 2
        ax = fig.add_subplot(2, 2, 1)
        plt.bar(x, station_error_max, alpha=0.9, width=0.35, facecolor='lightskyblue',
                edgecolor='white', label='max_error')
        plt.bar(x + width, station_error_mean, alpha=0.9, width=0.35, facecolor='yellowgreen',
                edgecolor='white', label='mean_error')
        plt.bar(x, station_max_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='max_error_base')
        plt.bar(x + width, station_mean_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='mean_error_base')
        ax.set_xticklabels(name_list)
        plt.legend(loc='upper right')
        plt.title('Station Error')
        ax1 = fig.add_subplot(2, 2, 2)
        plt.bar(x, speed_error_max, alpha=0.9, width=0.35, facecolor='lightskyblue',
                edgecolor='white', label='max_error')
        plt.bar(x + width, speed_error_mean, alpha=0.9, width=0.35, facecolor='yellowgreen',
                edgecolor='white', label='mean_error')
        plt.bar(x, speed_max_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='max_error_base')
        plt.bar(x + width, speed_mean_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='mean_error_base')
        ax1.set_xticklabels(name_list)
        plt.legend(loc='upper right')
        plt.title('Speed Error')
        ax2 = fig.add_subplot(2, 2, 3)
        plt.bar(x, lateral_error_max, alpha=0.9, width=0.35, facecolor='lightskyblue',
                edgecolor='white', label='max_error')
        plt.bar(x + width, lateral_error_mean, alpha=0.9, width=0.35, facecolor='yellowgreen',
                edgecolor='white', label='mean_error')
        plt.bar(x, lateral_max_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='max_error_base')
        plt.bar(x + width, lateral_mean_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='mean_error_base')
        ax2.set_xticklabels(name_list)
        plt.legend(loc='upper right')
        plt.title('Lateral Error')
        ax3 = fig.add_subplot(2, 2, 4)
        plt.bar(x, heading_error_max, alpha=0.9, width=0.35, facecolor='lightskyblue',
                edgecolor='white', label='max_error')
        plt.bar(x + width, heading_error_mean, alpha=0.9, width=0.35, facecolor='yellowgreen',
                edgecolor='white', label='mean_error')
        plt.bar(x, heading_max_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='max_error_base')
        plt.bar(x + width, heading_mean_base, alpha=0.9, width=0.35, facecolor='none', ls='dashed',
                edgecolor='red', label='mean_error_base')
        ax3.set_xticklabels(name_list)
        plt.legend(loc='upper right')
        plt.title('Heading Error')
        plt.show()


def main():
    """
    main function
    """
    benchmark = BenchMark()
    benchmark.load_data()
    benchmark.cutoff_scene()
    score_straight = benchmark.compute_score(
        benchmark.scene_stright_line, 'straight')['score']
    score_right_turn = benchmark.compute_score(
        benchmark.scene_right_turn, 'right_turn')['score']
    score_turn = benchmark.compute_score(
        benchmark.scene_turn, 'turn')['score']
    score_u_turn = benchmark.compute_score(
        benchmark.scene_u_turn, 'u_turn')['score']
    score_brake = benchmark.compute_score(
        benchmark.scene_brake, 'brake')['score']
    error_straight = benchmark.compute_score(
        benchmark.scene_stright_line, 'straight')['error']
    error_right_turn = benchmark.compute_score(
        benchmark.scene_right_turn, 'right_turn')['error']
    error_turn = benchmark.compute_score(
        benchmark.scene_turn, 'turn')['error']
    error_u_turn = benchmark.compute_score(
        benchmark.scene_u_turn, 'u_turn')['error']
    error_brake = benchmark.compute_score(
        benchmark.scene_brake, 'brake')['error']
    benchmark.print_result(score_straight, score_right_turn,
                           score_turn, score_u_turn, score_brake)
    benchmark.plot_result(error_straight, error_right_turn,
                          error_turn, error_u_turn, error_brake)


if __name__ == '__main__':
    main()
