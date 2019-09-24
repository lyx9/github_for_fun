#!/usr/bin/env python
# -*- coding: utf-8 -*-
################################################################################
#
# Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved.
#
################################################################################
"""
This module provide function to analysis the stastics files, record by shell script "mnt-pnc-topics.sh".

Authors: chendong(chendong06@baidu.com)
Date:    2017-05-11 18:21:05
"""
import os
import getopt
import sys
import re
import csv
import time
from datetime import datetime


def list_drive_mode(xlog_path, csv_file_direct=''):

    xlog_path = os.path.join(os.getcwd(), xlog_path)
    print xlog_path

    canbus_files = get_canbus_file(xlog_path)
    if len(canbus_files) == 0:
        return []
    write_list = []
    drive_mode_status = []
    #process every canbus xlog file
    for i in range(0, len(canbus_files)):
        drive_mode_status = read_canbus_file(canbus_files[i])
        if len(drive_mode_status) < 2:
            continue
        write_list.extend(drive_mode_status)

    #delete adjacent two same drive mod
    up_drive_mod = ''
    for i in range(len(write_list) - 1, -1, -1):
        if up_drive_mod == '':
            up_drive_mod = write_list[i][1]
        elif write_list[i][1] != up_drive_mod:
            up_drive_mod = write_list[i][1]
        elif write_list[i][1] == up_drive_mod:
            write_list.pop(i+1)

    #change result from [(time1,'COMPLETE_AUTO_DRIVE'),(time2, 'NOT_AUTO_DRIVE')] to
    #[(time1,time2,time2-time1,'COMPLETE_AUTO_DRIVE')]
    list_temp = []
    for i in range(len(write_list) - 1, -1, -1):
        if write_list[i][1] == 'COMPLETE_AUTO_DRIVE':
            #lastes driving mode is auto
            if i == len(write_list) -1:
                list_temp.insert(0,(write_list[i][0], 0, 0))
            else:
                cur_auto_time = time_diff(write_list[i+1][0], write_list[i][0])
                list_temp.insert(0,(write_list[i][0], write_list[i+1][0], cur_auto_time))
    write_list = list_temp
    #delete error data
    for i in write_list[-1]:
        if i == 0:
            write_list.pop()
            break

    if csv_file_direct == '':
        csv_file_name = os.path.join(os.getcwd(),'canbus-autodrive-time' +
                                 datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.csv')
    else:
        csv_file_name = os.path.join(csv_file_direct,'canbus-autodrive-time' +
                                 datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.csv')
        print 'get csv file name is:', csv_file_name

    print "=" * 130
    print "now write the %d result." % (len(write_list))
    write_csv(csv_file_name, write_list)
    return write_list


def get_canbus_file(xlog_path):
    canbus_xlog_list = []
    if os.path.exists(xlog_path):
        p_a, p_b = os.path.split(xlog_path)
        if p_b == "":
            # if xlog_path end with '\',delete the '\'
            xlog_path = p_a
            p_a, p_b = os.path.split(p_a)
            # print "new xlog_path is:%s p_a is: %s p_b is:%s" % (xlog_path, p_a, p_b)
        # specify one directory to be analyzed
        if os.path.isfile(xlog_path):
            canbus_xlog_list = [xlog_path, ]
            print("just analyze one specify file")
        # specify the root directory include monitor_pnc_topics
        else:
            file_list = os.listdir(xlog_path)
            pat = re.compile('^canbus_proxy\.log\.\d+-\d+\.\d+$')
            for i in range(0, len(file_list)):
                if len(pat.findall(file_list[i])) is not 0:
                    canbus_xlog_list.append(os.path.join(xlog_path, file_list[i]))
        canbus_xlog_list.sort()

        for i in range(0, len(canbus_xlog_list)):
            print "canbus %d file is: %s" % (i, canbus_xlog_list[i])

        return canbus_xlog_list


def read_canbus_file(file_name):
    drive_mode_list = []
    pat = re.compile('^.*\s(\d+:\d+:\d+\.\d+).*\sdriving_mode:\s(\w*)\s.*')
    canbus_file = open(file_name, 'r')
    up_status = ''
    try:
        print "=" * 100
        print "now parser the file %s" % (file_name)
        line_num = 1
        log_line = canbus_file.readline()
        while log_line:
            #print "num:%d" % (line_num)
            if '[chassis]' not in log_line or len(pat.findall(log_line)) is 0:
                line_num = line_num + 1
                log_line = canbus_file.readline()
                continue

            time = pat.findall(log_line)[0][0]
            drive_mode = pat.findall(log_line)[0][1]
            if drive_mode != 'COMPLETE_AUTO_DRIVE':
                drive_mode = 'NOT_AUTO_DRIVE'
            #first drive mode
            if len(drive_mode_list) == 0:
                print "find first drive mode in %d is: %s" % (line_num, pat.findall(log_line))
                up_status = drive_mode
                drive_mode_list.append((time, drive_mode))

            else:
                if drive_mode == up_status:
                    line_num = line_num + 1
                    log_line = canbus_file.readline()
                    continue
                else:
                    #print "next drive mod in %d is %s" % (line_num, drive_mode)
                    up_status = drive_mode
                    drive_mode_list.append((time, drive_mode))
            if len(drive_mode_list) == 5:
                print 'first five drive mode change is : %s' % (drive_mode_list)
            line_num = line_num + 1
            log_line = canbus_file.readline()
    finally:
        if canbus_file:
            canbus_file.close()

    print "origin drive mod change num is : %d" % (len(drive_mode_list))
    #when auto mode duration time is below 5 seconds. The data is not effective.
    not_auto_time = ''
    #print 'befor filter', drive_mode_list
    for i in range(len(drive_mode_list)-1, -1, -1):
        #print i, drive_mode_list[i]
        if drive_mode_list[i][1] == 'NOT_AUTO_DRIVE':
            not_auto_time = drive_mode_list[i][0]
        elif not_auto_time !='' and drive_mode_list[i][1] == 'COMPLETE_AUTO_DRIVE':
            # check whether delete up auto mode msg
            timediff = time_diff(drive_mode_list[i + 1][0], drive_mode_list[i][0])
            #print "time diff", timediff
            if time_diff(drive_mode_list[i+1][0], drive_mode_list[i][0]) < 5:
                drive_mode_list.pop(i+1)
                drive_mode_list.pop(i)
            #print "after filte", drive_mode_list
    print "after filter auto drive time duration more than 5 seconds. \nThere remain %d's mode change " % \
          (len(drive_mode_list))
    return drive_mode_list


def time_diff(time1, time2):
    # print "times:", time1, time2
    pat = re.compile('^\d\d:\d\d:\d\d\.\d+$')
    if len(pat.findall(time1)) == 0 or len(pat.findall(time2)) == 0:
        print "time data error"
    format_time_1 = time.strptime('2017 ' + time1[:8], '%Y %H:%M:%S')
    format_time_2 = time.strptime('2017 ' + time2[:8], '%Y %H:%M:%S')
    ms_diff = float(int(time1[9:]) - int(time2[9:])) / 1000000
    seconds_diff = time.mktime(format_time_1) - time.mktime(format_time_2)
    return float(seconds_diff) + ms_diff


def write_csv(csv_file, data_list):
    """
    *write data to csv file
    """
    print "csv name is %s" % (csv_file)

    try:
        print "now open csv file '%s' and write auto drive time section (len is %d )to it." % \
              (csv_file, len(data_list))
        with open(csv_file, 'wb') as write_file:
            writer = csv.writer(write_file)
            writer.writerow(['start-time', 'end-time', 'duration-time'])
            print data_list
            writer.writerows(data_list)
        write_file.close()
    except IOError as xxx_todo_changeme:
        (errno, strerror) = xxx_todo_changeme.args
        print("I/O error({0}): {1}".format(errno, strerror))
    return


if __name__ == "__main__":
    opts, args = getopt.getopt(sys.argv[1:], "d:h")

    for op, value in opts:
        if op == "-d":
            list_drive_mode(value, '')
        elif op == "-h":
            print "option -d to specify directory"
