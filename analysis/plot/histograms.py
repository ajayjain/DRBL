#!/usr/bin/python
import csv
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

shots = [[] for _ in xrange(6)]

with open('runs_success.tsv','rb') as tsvin:
	tsvin = csv.reader(tsvin, delimiter='\t')

	for row in tsvin:
		if len(row) != 14 or row[0] == 'max_lin_0': # header row
			continue
		row = [float(el) for el in row]
		max_lin_1, max_ang_1, x1, y1, data = row[2], row[3], row[6], row[7], row[8:14]
		if x1 == -3 and y1 == -4:
			for i, shot in enumerate(data):
				shots[i].append(shot)

with open('runs_failure.tsv','rb') as tsvin:
	tsvin = csv.reader(tsvin, delimiter='\t')

	for row in tsvin:
		if len(row) != 14 or row[0] == 'max_lin_0': # header row
			continue
		row = [float(el) for el in row]
		max_lin_1, max_ang_1, x1, y1, data = row[2], row[3], row[6], row[7], row[8:14]
		if x1 == -3 and y1 == -4:
			for i, shot in enumerate(data):
				if shot != -1:
					shots[i].append(shot)
			# shots[0].append(shot_0)

		# csvout.writerows([row[2:4] for _ in xrange(count)])

# print shots
num_bins = 50
# the histogram of the data
n, bins, patches = plt.hist(shots[0], num_bins, normed=False, facecolor='green', alpha=0.5)

plt.xlabel('Time to shot_0 (sec)')
plt.ylabel('Probability')

plt.title(r'Histogram of time to first shot')

# Tweak spacing to prevent clipping of ylabel

plt.subplots_adjust(left=0.15)


plt.show()
