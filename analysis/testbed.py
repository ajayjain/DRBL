#!/usr/env python
# -*- coding: utf-8 -*-
"""
Author: Ajay Jain
Date: Sept. 7, 2014
"""

import csv
import os
import signal
import subprocess
import time

data = []
headers = None

SIM_TIMEOUT_SECS = 30

def load_params(filename="params.tsv"):
	global data, headers

	tsv = open(filename, 'r')
	reader = csv.reader(tsv, delimiter='\t')

	for i, row in enumerate(reader):
		if i == 0:
			headers = row
		else:
			data.append(tuple(row))
	tsv.close()

def append_success(run_data, fire_times):
	data = run_data + tuple(fire_times)
	row = "%s\t%s\t%s\t%s\t%f\t%f\t%f\t%f\t%f\t%f\t\n" % data	# 4 params, 6 shot times
	with open('runs_success.tsv', 'ab') as f:
		f.write(row)

def append_failure(run_data, fire_times):
	fire_data = [t if t else -1.0 for t in fire_times]
	data = run_data + tuple(fire_data)
	row = "%s\t%s\t%s\t%s\t%f\t%f\t%f\t%f\t%f\t%f\t\n" % data	# 4 params, 6 shot times
	with open('runs_success.tsv', 'ab') as f:
		f.write(row)

if __name__=="__main__":
	load_params()
	print headers
	for run_data in data:
		shots_file = os.path.abspath('shots.tsv')
		open('shots.tsv', 'w').close() # clear file

		print run_data
		command_template = """roslaunch /home/ajay/catkin_ws/src/husky_pursuit/analysis/testbed.launch \
		max_lin_0:=%s \
		max_ang_0:=%s \
		max_lin_1:=%s \
		max_ang_1:=%s \
		shots_file:=""".replace('\t', ' ')
		command_template += shots_file
		command = command_template % run_data
		print command

		# The os.setsid() is passed in the argument preexec_fn so
		# it's run after the fork() and before  exec() to run the shell.
		start_time = time.time()
		pro = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
		with open('shots.tsv', 'r') as file:
			fire_times = [-1] * 6
			while True:
				where = file.tell()
				line = file.readline()
				if not line:
					time.sleep(0.1)
					file.seek(where)
				else:
					fire_time, fire_count = line.rstrip().split('\t')
					fire_time = float(fire_time)
					fire_count = int(fire_count)
					print fire_time, fire_count

					if fire_count <= 5:
						fire_times[fire_count] = fire_time
					if fire_count >= 5:
						print "reached fire_count %d at time %f. KILLING LAUNCH PROCESS..." % (fire_count, fire_time)
						os.killpg(pro.pid, signal.SIGTERM)  # Send the signal to all the process groups
						append_success(run_data, fire_times)
						break
				if time.time() - start_time > SIM_TIMEOUT_SECS:
					print "TIMEOUT. KILLING LAUNCH PROCESS..."
					os.killpg(pro.pid, signal.SIGTERM)  # Send the signal to all the process groups
					append_failure(run_data, fire_times)