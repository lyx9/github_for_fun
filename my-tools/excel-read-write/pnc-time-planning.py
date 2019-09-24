#! /usr/bin/env python
# -*- coding: utf-8 -*-
################################################################################
#
# Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved.
#
################################################################################
"""
This module provide function to analysis the PNC mode generated xlog file, to get algorithm consumption in time.

Authors: chendong(chendong06@baidu.com)
Date:    2017-05-13 22:20:05
"""
import os
import sys
import csv

import numpy as np
import getopt
import re
from datetime import datetime
import time

import drive_mode
import xlrd
import xlwt
from xlutils.copy import copy as xlcopy


def Calculate(data_direct_path, is_filter, start_time=0, end_time=9999999):
    """
    :param data_direct_path: the xlog files directory
    :param is_filter: bool param to decide whether filter data by driving mode
    :return:
    """
    file_type_list = ['prediction.log', 'planning.log', 'control.log']
    key_words = [["time_logger.cpp:62",], ["] task_manager:total", ], ["calc_time is:",]]
    title = ['prediction', 'planning', 'control']

    # get all nodes consum time
    nodes_perf_data_dict = {}
    for i in range(0, len(file_type_list)):
        check_files_list = DirectCheck(data_direct_path, file_type_list[i])
        if len(check_files_list) == 0:
            nodes_perf_data_dict[file_type_list[i]] = []
            continue

        node_csm_time = []

        for j in range(0, len(key_words[i])):
            node_csm_time.append([])
            #title
            #node_csm_time[j].append(('time', key_words[i][j]))
            for k in range(0, len(check_files_list)):
                file_path = os.path.join(data_direct_path, check_files_list[k])
                node_csm_time[j].extend(GetXlogNodePerfDataList(file_path, file_type_list[i], key_words[i][j]))

        nodes_perf_data_dict[file_type_list[i]] = node_csm_time

    # make direct to save result files
    result_path = os.path.join(os.getcwd(), 'PNC-Perf-Test-%s' %
                               (datetime.now().strftime("%Y-%m-%d-%H-%M-%S")))
    os.mkdir(result_path)


    # filter data by drive mode
    if is_filter == True:
        if start_time == 0:
            ##get auto drive mode time list
            auto_drive_time = drive_mode.list_drive_mode(data_direct_path, result_path)
        else:
            #use function input params
            auto_drive_time = [[start_time, end_time]]

        for i in range(0, len(file_type_list)):
            if len(nodes_perf_data_dict[file_type_list[i]]) == 0:
                continue
            if len(auto_drive_time) == 0:
                print 'drive mode get Error!'
            else:
                # according driving mode filter consum data
                for j in range(0, len(key_words[i])):
                    if len(nodes_perf_data_dict[file_type_list[i]][j]) == 1:
                        continue
                    nodes_perf_data_dict[file_type_list[i]][j] = filter_perf_data_by_drivemode(
                        nodes_perf_data_dict[file_type_list[i]][j], auto_drive_time)

    # sampling long data which length larger than 65536
    for i in range(0, len(key_words)):
        if len(nodes_perf_data_dict[file_type_list[i]]) == 0:
            continue
        for j in range(0, len(key_words[i])):
            origin_len = len(nodes_perf_data_dict[file_type_list[i]][j])
            if origin_len > 65535:
                print "sample %d data of %s:%s" % (origin_len, file_type_list[i],key_words[i][j])
                nodes_perf_data_dict[file_type_list[i]][j] = SampleLongData(nodes_perf_data_dict[file_type_list[i]][j])
                print "sample result %d to %d" % (origin_len, len(nodes_perf_data_dict[file_type_list[i]][j]))
            if len(nodes_perf_data_dict[file_type_list[i]][j]) > 65535:
                break

    # write detail data to xls file
    xls_file_name = os.path.join(result_path,'PNC-Perf-Data' +
                         datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.xls')
    #decide whether to new a xls file
    is_new = True
    for i in range(0, len(file_type_list)):

        if len(nodes_perf_data_dict[file_type_list[i]]) == 0:
            #file has no data
            continue

        print "=" * 130
        for j in range(0, len(key_words[i])):
            if len(nodes_perf_data_dict[file_type_list[i]][j]) == 1:
                #data type of one file only has one data
                continue
            #suport abstract multi data type
            print "%d write for %s" % (j, file_type_list[i])
            #new sheet
            nodes_perf_data_dict[file_type_list[i]][j].insert(0,('time', title[i]))
            #print write_content
            #break
            WriteListToXLS(xls_file_name, nodes_perf_data_dict[file_type_list[i]][j], is_new)
        is_new = False

    # write statistics result
    result_stastics_content = []
    result_stastics_content.append(('node_name', 'avg', 'min', 'max', '99%value', '95%value', '1%value'))
    for i in range(0, len(file_type_list)):
        print "caculate stastic data of %s" % (file_type_list[i])
        if len(nodes_perf_data_dict[file_type_list[i]]) == 0:
            print "no data"
            continue
        for j in range(0, len(key_words[i])):
            if len(nodes_perf_data_dict[file_type_list[i]][j]) < 2:
                print "only one data"
                continue

            print "write data of %s" % (file_type_list[i])
            result_stastics_content.append(stastic_perf_time(nodes_perf_data_dict[file_type_list[i]][j][1:],
                                                             title[i]))
    print "stastic :\n", result_stastics_content
    WriteListToXLS(xls_file_name, result_stastics_content)


def SampleLongData(data_list, threshold=65535):
    index = len(data_list) / threshold + 1
    temp = []
    for i in range(0, len(data_list)):
        if i % index == 0:
            temp.append(data_list[i])
    return temp


def GetXlogNodePerfDataList(filepath, file_type, timekeywords):
    """
    :param filepath:
    :param timekeywords:
    :return:
    """
    file = open(filepath, 'r')
    csm_time = []

    try:
        print "=" * 100
        print "now parser the file %s" % (filepath)
        words = Rewords(timekeywords)
        #print "'^\S+\s+(\d+\:\d+:\d+\.\d+)\s+.*%s\s*(\d+\.?\d*)\s*.*'" % (words)
        if file_type == 'prediction.log':
            pat = re.compile('^\S+\s+(\d+\:\d+:\d+\.\d+)\s+.*%s\s*(-?\d+\.?\d*)\s*.*' % (words))
        elif file_type == 'planning.log':
            pat = re.compile('^\S+\s+(\d+\:\d+:\d+\.\d+)\s+.*%s\[(-?\d+\.?\d*)\]\s*.*' % (words))
        elif file_type == 'control.log':
            pat = re.compile('^\S+\s+(\d+\:\d+:\d+\.\d+)\s+.*%s\s*(-?\d+\.?\d*)ms.*' % (words))
        data_gain = 1
        if file_type == 'prediction.log':
            data_gain = 1000
        elif file_type == 'planning.log':
            data_gain = 0.001
        else:
            data_gain = 1

        log_line = file.readline()
        while log_line:
            #print timekeywords, log_line
            if timekeywords in log_line:
                #print "log line", log_line
                #print "line is:", log_line
                times = data_gain * float(pat.findall(log_line)[0][1])
                if times:
                    csm_time.append((pat.findall(log_line)[0][0], times))
            log_line = file.readline()

        print "file include [ %d ] time_consum data" % (len(csm_time))

    finally:
        if file:
            file.close()
    return csm_time


def Rewords(origin_words):
    """
    change keywords of re in origin to \ + keywords
    """
    keys = ['[', ']']
    newwords=""
    for i in range(0, len(keys)):
        kn = origin_words.find(keys[i])
        if kn != -1:
            origin_words = origin_words[:kn] + '\\' +  origin_words[kn:]
    return origin_words


def WriteListToCSV(csv_file_name, data_title, data_list):
    """
    *write data to csv file
    """
    print "csv file name is %s" % (csv_file_name)

    try:
        print "now write time_consum data (len is %d )to it." % \
              (len(data_list))
        with open(csv_file_name, 'wb') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(data_title)
            writer.writerows(data_list)
        csv_file.close()
    except IOError as xxx_todo_changeme:
        (errno, strerror) = xxx_todo_changeme.args
        print("I/O error({0}): {1}".format(errno, strerror))
    return


def WriteListToXLS(xls_file_name, data_list, is_new_file=False, sheet_name='pnc'):
    #if rb.sheet_by_name(sheet_name):
    if is_new_file:
        ncols = 0
        wb = xlwt.Workbook()
        ws = wb.add_sheet(sheet_name)
    else:
        rb = xlrd.open_workbook(xls_file_name)
        ncols =rb.sheets()[0].ncols
        wb = xlcopy(rb)
        ws = wb.get_sheet(0)
    print "data list", data_list[:5]
    xstyle = xlwt.Style.easyxf(num_format_str='0.00')
    for i in range(0, len(data_list)):
        for j in range(0, len(data_list[i])):
            ws.write(i, ncols + j, data_list[i][j], xstyle)

    wb.save(xls_file_name)


def DirectCheck(path, file_type):
    """check effective directory in root directory """
    print "\nDirectCheck path: %s" % (path)

    files_all_list = []
    type_file_list = []
    pat = re.compile('%s\.\d+-\d+\.\d+' % (file_type))
    if os.path.isdir(path):
        files_all_list = os.listdir(path)
        for i in range(0, len(files_all_list)):
            if len(pat.findall(files_all_list[i])) != 0:
                type_file_list.append(files_all_list[i])
    else:
        print path, "is not a effective path"
        return []
    type_file_list.sort()
    print ("there are %d %s files to be analyze: \n%s"
           % (len(type_file_list), file_type, type_file_list))

    return type_file_list


def time_diff(time1, time2):
    #print "times:", time1, time2
    pat = re.compile('^\d\d:\d\d:\d\d\.\d+$')
    if len(pat.findall(time1)) == 0 or len(pat.findall(time2)) == 0:
        print "time data error"
    format_time_1 = time.strptime('2017 ' + time1[:8], '%Y %H:%M:%S')
    format_time_2 = time.strptime('2017 ' + time2[:8], '%Y %H:%M:%S')
    ms_diff = float(int(time1[9:])-int(time2[9:]))/1000000
    seconds_diff = time.mktime(format_time_1) - time.mktime(format_time_2)
    return float(seconds_diff) + ms_diff


def filter_perf_data_by_drivemode(node_perf_data_list, auto_drive_time_section_list):
    index_auto_time_section = 0
    auto_time_start = auto_drive_time_section_list[index_auto_time_section][0]
    auto_time_end = auto_drive_time_section_list[index_auto_time_section][1]
    #print "time start and end", auto_time_start, auto_time_end

    before_filter_perf_data = node_perf_data_list
    #print "filet before data:", before_filter_perf_data
    #print "auto time section", auto_drive_time_section_list
    after_filter_perf_data = []

    for i in range(0, len(before_filter_perf_data)):
        time_diff_start = time_diff(before_filter_perf_data[i][0], auto_time_start)
        time_diff_end = time_diff(before_filter_perf_data[i][0], auto_time_end)
        #node consum data time before auto drive
        if  time_diff_start <= 0:
            continue
        # node consum data time before auto drive
        elif time_diff_start > 0 and time_diff_end < 0:
            after_filter_perf_data.append(before_filter_perf_data[i])
        elif time_diff_end >= 0:
            index_auto_time_section = index_auto_time_section + 1
            if index_auto_time_section <= len(auto_drive_time_section_list) - 1:
                auto_time_start = auto_drive_time_section_list[index_auto_time_section][0]
                auto_time_end = auto_drive_time_section_list[index_auto_time_section][1]
            else:
                break

    return after_filter_perf_data


def stastic_perf_time(data_list, file_type):
    print '#248', data_list[:5]
    value_list = []
    for i in range(0, len(data_list)):
        value_list.append(data_list[i][1])
    print '%s avg is %.2f min is %.2f max is %.2f' % (file_type, sum(value_list)/len(value_list),
        min(value_list), max(value_list))
    return (file_type, sum(value_list)/len(value_list), min(value_list), max(value_list),
                np.percentile(value_list,99), np.percentile(value_list, 95), np.percentile(value_list, 1))


def usage():
    """display help msg"""
    print "python pnc-time-parser.py [options]"
    print "options:"
    print "-a log_path \n\taccording drive mode to stastics PNC module time consuming,time section of " \
          "auto drive from canbus file"
    print "-o path start_time end_time \n\tstastics PNC module time consuming by specify time section"
    print "-t path \n\tjust abstract perf data without filter data time"
    print "-h show the help"


if __name__ == "__main__":

    node_csm_time = []
    opts, args = getopt.getopt(sys.argv[1:], "a:o:t:h")
    for op, value in opts:
        if op == "-h":
            usage()
            sys.exit()
        elif op == "-a":
            Calculate(value, True)
        elif op == "-o":
            print args[:]
            Calculate(value, True, args[0], args[1])
        elif op == "-t":
            Calculate(value, False)
        else:
            usage()
    if len(opts) == 0:
        usage()
