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
pos_data = []
headers = None

SIM_TIMEOUT_SECS = 30

def load_params(filename="params.tsv"):
	global data, pos_data, headers

	tsv = open(filename, 'r')
	reader = csv.reader(tsv, delimiter='\t')

	for i, row in enumerate(reader):
		if i == 0:
			headers = row
		else:
			data.append(tuple(row))
	tsv.close()

def render_template(run_data):
	tpl = ""
	with open('testbed.world.template', 'rb') as f_tpl:
		tpl = f_tpl.read()
	with open('testbed.world', 'wb') as f_world:
		content = tpl.format(x0=run_data[4],
							 y0=run_data[5],
							 x1=run_data[6],
							 y1=run_data[7])
		f_world.write(content)

def append_data(run_data, fire_times, filename):
	row =  "%s\t%s\t%s\t%s\t"	# 4 params
	row += "%s\t%s\t%s\t%s\t"	# 4 coord values
	row += "%f\t%f\t%f\t%f\t%f\t%f\n"	# 6 shot times
	data = run_data + tuple(fire_times)
	with open(filename, 'ab') as f:
		f.write(row % data)

if __name__=="__main__":
	load_params()
	print headers
	for run_data in data:
		render_template(run_data)

		shots_file = os.path.abspath('shots.tsv')
		open('shots.tsv', 'w').close() # clear file

		print run_data
		command_template = """roslaunch /home/ajay/catkin_ws/src/husky_pursuit/analysis/testbed.launch \
		max_lin_0:=%s \
		max_ang_0:=%s \
		max_lin_1:=%s \
		max_ang_1:=%s \
		x0:=%s \
		y0:=%s \
		x1:=%s \
		y1:=%s \
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
						append_data(run_data, fire_times, 'runs_success.tsv')
						break
				if time.time() - start_time > SIM_TIMEOUT_SECS:
					print "TIMEOUT. KILLING LAUNCH PROCESS..."
					os.killpg(pro.pid, signal.SIGTERM)  # Send the signal to all the process groups
					append_data(run_data, fire_times, 'runs_failure.tsv')
					break