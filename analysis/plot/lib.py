import csv

def mean(data):
    n = len(data)
    if n < 1:
    	return -1
    return (reduce(lambda tot, el: tot + el, data)/n)

def average_trial_data():
	dat = dict()

	with open('runs_success.tsv','rb') as tsvin:
		tsvin = csv.reader(tsvin, delimiter='\t')

		for row in tsvin:
			if len(row) != 14 or row[0] == 'max_lin_0': # header row
				continue
			max_lin_1, max_ang_1, x1, y1, shots = row[2], row[3], float(row[6]), float(row[7]), row[8:14]
			shots = [float(el) for el in shots]
			if x1 == -3 and y1 == -4:
				if max_ang_1 not in dat:
					dat[max_ang_1] = dict()
				if max_lin_1 not in dat[max_ang_1]:
					dat[max_ang_1][max_lin_1] = [list() for _ in xrange(6)]
				for i, shot in enumerate(shots):
					dat[max_ang_1][max_lin_1][i].append(shot)

	with open('runs_failure.tsv','rb') as tsvin:
		tsvin = csv.reader(tsvin, delimiter='\t')

		for row in tsvin:
			if len(row) != 14 or row[0] == 'max_lin_0': # header row
				continue
			max_lin_1, max_ang_1, x1, y1, shots = row[2], row[3], float(row[6]), float(row[7]), row[8:14]
			shots = [float(el) for el in shots]
			if x1 == -3 and y1 == -4 and shots.count(-1) < 6:
				if max_ang_1 not in dat:
					dat[max_ang_1] = dict()
				if max_lin_1 not in dat[max_ang_1]:
					dat[max_ang_1][max_lin_1] = [list() for _ in xrange(6)]
				for i, shot in enumerate(shots):
					if shot != -1:
						dat[max_ang_1][max_lin_1][i].append(shot)

	# print dat

	# average trials. -1 for a shot time means no successful trials were had
	avgdat = dat
	for ang in avgdat:
		for lin in avgdat[ang]:
			for (i, shotarr) in enumerate(avgdat[ang][lin]):
				avg = mean(shotarr)
				avgdat[ang][lin][i] = avg

	return avgdat