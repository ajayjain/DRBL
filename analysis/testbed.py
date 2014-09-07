#!/usr/env python
# -*- coding: utf-8 -*-

import csv
import os
import signal
import subprocess
import time

data = []
headers = None

def load_params(filename="params.tsv"):
	global data, headers

	tsv = open(filename, 'r')
	reader = csv.reader(tsv, delimiter='\t')

	for i, row in enumerate(reader):
		if i == 0:
			headers = row
		else:
			data.append(row)
	tsv.close()

if __name__=="__main__":
	load_params()
	print headers
	for run_data in data:
		print tuple(run_data)
		command_template = """roslaunch /home/ajay/catkin_ws/src/husky_pursuit/analysis/testbed.launch \
		max_lin_0:=%s \
		max_ang_0:=%s \
		max_lin_1:=%s \
		max_ang_1:=%s""".replace('\t', ' ')
		command = command_template % tuple(run_data)
		print command

		open('shots.tsv', 'w').close() # clear file

		# The os.setsid() is passed in the argument preexec_fn so
		# it's run after the fork() and before  exec() to run the shell.
		pro = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
		time.sleep(5)
		os.killpg(pro.pid, signal.SIGTERM)  # Send the signal to all the process groups




# data = {
	
# }

# f_in = open('testbed_template.launch', 'r')
# template = Template(f_in.read())
# output = template.render(data)
# f_in.close()

# f_out = open('testbed.launch', 'w')
# f_out.write(output)
# f_out.close()
