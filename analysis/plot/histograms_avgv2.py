#!/usr/bin/python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from lib import mean, average_trial_data_v2

def grouped_shots():
	shots = [[] for _ in xrange(6)]
	for ang in success:
		for lin in success[ang]:
			for (i, avg) in enumerate(success[ang][lin]):
				shots[i].append(avg)
	return shots

(success, failure) = average_trial_data_v2()
num_bins = 100
shots = grouped_shots()


def PDF():
	for i, shotarr in enumerate(shots):
		# plot formatting
		ax = plt.subplot(6, 1, i + 1)
		# plt.tight_layout(w_pad=0.0, h_pad=0.00)
		axes = plt.gca()
		axes.set_xlim([0,70])
		axes.set_ylim([0.00,0.35])

		# histogram plotting
		n, bins, patches = plt.hist(shots[i], num_bins, normed=True, facecolor='green', alpha=0.5, histtype='stepfilled')

		# calculate properties
		avg = np.mean(shots[i])
		med = np.median(shots[i])
		sigma = np.std(shots[i])

		# add a 'best fit' line
		# y = mlab.normpdf(bins, med, sigma)
		print avg, med, sigma
		# plt.plot(bins, y, 'r--')

		# add text box \\tilde{\mathrm{t}}
		textstr = '$\mathrm{mean}=%.2f$\n$\mathrm{median}=%.2f$\n$\sigma=%.2f$' %(avg, med, sigma)
		props = dict(boxstyle='round', facecolor='wheat', alpha=0.5) # these are matplotlib.patch.Patch properties
		left, width = .35, .5
		bottom, height = .26, .5
		right = left + width
		top = bottom + height
		ax.text(right, top, textstr, transform=ax.transAxes, fontsize=14,
	        horizontalalignment='center', verticalalignment='top', bbox=props)

		textstr = '$\mathrm{shot\_%d}$' % i
		ax.text(right, 0.95, textstr, transform=ax.transAxes, fontsize=14,
			horizontalalignment='center', verticalalignment='top')

		plt.ylabel('PDF')
	plt.xlabel('Time $\mathrm{t}$ to $\mathrm{shot\_i}$ (sec)')
	plt.suptitle(r'Time probabilities for shots 0 through 5, $\Delta x = 3$, $\Delta y = 4$')

def ECDF():
	for i, shotarr in enumerate(shots):
		# plot formatting
		ax = plt.subplot(6, 1, i + 1)
		# plt.tight_layout(w_pad=0.0, h_pad=0.00)
		axes = plt.gca()
		axes.set_ylim([0.00,0.3])
		axes2 = axes.twinx()
		axes.set_xlim([0, 72])
		axes2.set_xlim([0,72])

		# histogram plotting
		n, bins, patches = axes.hist(shots[i], num_bins, normed=True, facecolor='green', alpha=0.5, histtype='stepfilled')
		# n, bins, patches = plt.hist(shots[i], num_bins, normed=True, cumulative=True, facecolor='green', alpha=0.5, histtype='stepfilled')

		# calculate properties
		avg = np.mean(shots[i])
		med = np.median(shots[i])
		sigma = np.std(shots[i])

		srted = np.sort( shotarr )
		yvals = np.arange(len(srted))/float(len(srted))
		print srted
		print yvals
		axes2.plot( srted, yvals, 'r' )


		# add a 'best fit' line
		# y = mlab.normpdf(bins, med, sigma)
		print avg, med, sigma
		# plt.plot(bins, y, 'r--')

		# add text box \\tilde{\mathrm{t}}
		textstr = '$\mu=%.2f$\n$\mathrm{median}=%.2f$\n$\sigma=%.2f$' %(avg, med, sigma)
		props = dict(boxstyle='round', facecolor='wheat', alpha=0.5) # these are matplotlib.patch.Patch properties
		left, width = .35, .5
		bottom, height = .24, .5
		right = left + width
		top = bottom + height
		ax.text(right, top, textstr, transform=ax.transAxes, fontsize=14,
	        horizontalalignment='center', verticalalignment='top', bbox=props)

		textstr = '$\mathrm{shot\_%d}$' % i
		ax.text(right, 0.93, textstr, transform=ax.transAxes, fontsize=14,
			horizontalalignment='center', verticalalignment='top')

		axes.set_ylabel('PDF (green)')
		axes2.set_ylabel('CDF (red)')
		axes.set_xlabel('Simuation timeline (seconds)')
	plt.suptitle('Time distribution of 6 shots fired by the pursuer across all factor sets\nPDF (green): Fraction of shots fired during each of 100 timestep bins\nCDF (red): Fraction of shots fired by that time\n$\Delta x = 3$ m, $\Delta y = 4$ m, Evader $0.6$ m/s and $1.0$ rad/s velocity limits', fontname="Times New Roman Bold", fontsize=16)

def CDF():
	# shotarr is a flat array if floats for each of 6 histograms
	for i, shotarr in enumerate(shots):
		# plot formatting
		ax = plt.subplot(6, 1, i + 1)
		# plt.tight_layout(w_pad=0.0, h_pad=0.00)
		axes = plt.gca()
		axes.set_xlim([0,80])
		# axes.set_ylim([0.00,0.35])

		# for ECF: http://stackoverflow.com/questions/3209362/how-to-plot-empirical-cdf-in-matplotlib-in-python
		srted = np.sort( shotarr )
		yvals = np.arange(len(srted))/float(len(srted))
		print srted
		print yvals
		plt.plot( srted, yvals )

		# histogram plotting
		# n, bins, patches = plt.hist(shots[i], num_bins, normed=True, facecolor='green', alpha=0.5, histtype='stepfilled')

		# calculate properties
		avg = np.mean(shots[i])
		med = np.median(shots[i])
		sigma = np.std(shots[i])

		# add a 'best fit' line
		# y = mlab.normpdf(bins, med, sigma)
		# print avg, med, sigma
		# plt.plot(bins, y, 'r--')

		# add text box \\tilde{\mathrm{t}}
		textstr = '$\mu=%.2f$\n$\mathrm{median}=%.2f$\n$\sigma=%.2f$' %(avg, med, sigma)
		props = dict(boxstyle='round', facecolor='wheat', alpha=0.5) # these are matplotlib.patch.Patch properties
		left, width = .35, .5
		bottom, height = .26, .5
		right = left + width
		top = bottom + height
		ax.text(right, top, textstr, transform=ax.transAxes, fontsize=14,
	        horizontalalignment='center', verticalalignment='top', bbox=props)

		textstr = '$\mathrm{shot\_%d}$' % i
		ax.text(right, 0.95, textstr, transform=ax.transAxes, fontsize=14,
			horizontalalignment='center', verticalalignment='top')

		plt.ylabel('CDF')
	plt.xlabel('Time $\mathrm{t}$ to $\mathrm{shot\_i}$ (sec)')

	# plt.suptitle(r'Time probabilities for shots 0 through 5, $\Delta x = 3$, $\Delta y = 4$')
	plt.suptitle(r'Time probabilities for shots 0 through 5, $\Delta x = 3 \textrm{m}$, $\Delta y = 4 \textrm{m}$')

ECDF()
plt.show()
