#!/usr/bin/python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from lib import mean, average_trial_data

avgdat = average_trial_data()

shots = [[] for _ in xrange(6)]
for ang in avgdat:
	for lin in avgdat[ang]:
		for (i, avg) in enumerate(avgdat[ang][lin]):
			if avg >= 0:
				shots[i].append(avg)

num_bins = 100
# the histogram of the data


for i, shotarr in enumerate(shots):
	# plot formatting
	ax = plt.subplot(3, 2, i + 1)
	plt.tight_layout(w_pad=0.0, h_pad=0.00)
	axes = plt.gca()
	axes.set_xlim([0,70])
	axes.set_ylim([0.00,0.30])

	# histogram plotting
	n, bins, patches = plt.hist(shots[i], num_bins, normed=True, facecolor='green', alpha=0.5, histtype='bar')

	# calculate properties
	avg = np.mean(shots[i])
	med = np.median(shots[i])
	sigma = np.std(shots[i])

	# add a 'best fit' line
	y = mlab.normpdf(bins, med, sigma)
	print avg, med, sigma
	plt.plot(bins, y, 'r--')

	# add a 'best fit' line
	y = mlab.normpdf(bins, med, sigma)
	print avg, med, sigma
	plt.plot(bins, y, 'r--')

	# add text box
	textstr = '$\mu=%.2f$\n$\\tilde{\mathrm{t}}=%.2f$\n$\sigma=%.2f$' %(avg, med, sigma)
	props = dict(boxstyle='round', facecolor='wheat', alpha=0.5) # these are matplotlib.patch.Patch properties
	left, width = .35, .5
	bottom, height = .3, .5
	right = left + width
	top = bottom + height
	ax.text(right, top, textstr, transform=ax.transAxes, fontsize=14,
        horizontalalignment='right', verticalalignment='top', bbox=props)

	plt.ylabel('Probability of time')
	plt.xlabel('Time $\mathrm{t}$ to $\mathrm{shot\_%d}$ (sec)' % i)
	plt.title('$\mathrm{shot\_%d}$' % i)

# plt.ylabel('Normalized probability')
plt.suptitle(r'Probabilities of times for shots 0 through 5')
# Tweak spacing to prevent clipping of ylabel

# plt.subplots_adjust(left=0.15)


plt.show()
