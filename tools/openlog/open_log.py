'''
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
'''
import time
from numpy import *
from pylab import *
import struct
# import sys
# import os, fnmatch
import argparse
import re

LOGGEROL_SEP = '/'

def readLogFile(filename, hasAlignmentWord = True, quiet = False):

	f = open(filename, 'rb')
	# if not quiet:
	print('Opened'),
	print(filename)

	# First two lines are special
	line1 = f.readline()
	newStyleLog = False
	if line1[0] == LOGGEROL_SEP:
		newStyleLog = True

	if newStyleLog:
		line1split = line1.lstrip(LOGGEROL_SEP).rstrip('\n').split(LOGGEROL_SEP)
		logVer = int(line1split[0])
		if not quiet:
			print 'New style log version', logVer

		keys = line1split[2].split(',')
		fmt = line1split[1]
		data = dict()
		ncols = len(keys)
		for i in range(ncols):
			# print keys[i]
			# figure out the keys and cols
			m = re.search(r'\d+', keys[i])
			ndataincol = int(m.group())
			# only keep the part before the int for key name
			keys[i] = keys[i][:m.start()]
			data[keys[i]] = zeros((0,ndataincol), dtype=np.float32)
	else:
		keys = line1.rstrip('\n').split(',')
		fmt = f.readline().rstrip('\n')
		data = dict.fromkeys(keys,zeros((0)))
		ncols = len(keys)

	sz = struct.calcsize(fmt)

	if not quiet:
		print('Keys:'),
		print(keys)
		print('Format:'),
		print(fmt)
		print('Size:'),
		print(sz)

	# print data
	# abort

	# Now just data
	if not hasAlignmentWord:
		line = f.read(sz)
		while len(line) == sz:
			tup = struct.unpack(fmt, line)

			for i in range(ncols):
				data[keys[i]] = r_[data[keys[i]],tup[i]]#vstack((data,array(tup)))
			line = f.read(sz)

	else:
		wholeFile = f.read()
		# split by alignment word
		chunks = wholeFile.split('\xaa\xbb')
		for chunk in chunks:
			# print len(chunk)
			# abort
			if len(chunk) == sz:
				# good chunk
				tup = struct.unpack(fmt, chunk)
				curpos = 0
				for i in range(ncols):
					ndataincol = data[keys[i]].shape[1]
					# print ndataincol
					if newStyleLog:
						newrow = array(tup[curpos:curpos+ndataincol])
						curpos = curpos + ndataincol
						# convert from compressed to float (opposite of LoggerOL)
						if keys[i] in ['eul','angVel','pos','torq','Vbat','xdot']:
							newrow = 0.001*newrow
						elif keys[i] in ['acc','Ibat']:
							newrow = 0.01*newrow
						data[keys[i]] = vstack((data[keys[i]], newrow))
						# print data[keys[i]]
						# abort
					else:
						data[keys[i]] = r_[data[keys[i]],tup[i]]

	if not quiet:
		print('Read data of length'),
		print(data[keys[0]].shape[0])
		if 't' in keys:
			if len(data['t'].shape) > 1:
				delt = diff(data['t'],axis=0)
			else:
				delt = diff(data['t'])
			print 'Avg data rate = {:2.2f} Hz'.format(1000/float(mean(delt)))

	return data

#
def trimData(data,inds):
	for key in data.keys():
		data[key] = data[key][inds]
	return data

def getRegions(tCol, stateCol = None):
	if stateCol is None:
		return None

	boundaries = r_[0, where(diff(stateCol) != 0)[0]]
	regions = zeros((0,3), dtype=int)

	if size(boundaries) > 1:
		for i in range(1,size(boundaries)):
			aa = (boundaries[i-1], boundaries[i])
			regions = vstack((regions, array([stateCol[boundaries[i]], aa[0], aa[1]], dtype=int)))
		# Add last region
		regions = vstack((regions, array([stateCol[boundaries[i]+1], aa[1], tCol.shape[0]-1], dtype=int)))
	return regions

def drawRegions(tCol, stateCol = None):
	if stateCol is None:
		return

	regions = getRegions(tCol, stateCol)
	cm = get_cmap('brg')

	for r in regions:
		bb = interp(r[0], r_[min(stateCol), max(stateCol)], r_[0,1])
		axvspan(tCol[r[1]], tCol[r[2]], facecolor=cm(bb), alpha=0.2, ec='none')

	xlim(min(tCol),max(tCol))

def drawVLines(tCol, every=None):  
	if every is None:
		return
	for x in arange(0,max(tCol),every):
		if x > min(tCol):
			axvline(x=x,ymin=-1000,ymax=1000,color='black',alpha=0.2)

def plotTrial(filename, trialType = None, trimRange = None, vlineEvery = None):
	beQuiet = (trialType == 'noplot')
	data = readLogFile(filename, hasAlignmentWord=True, quiet=beQuiet)
	if trimRange is not None:
		data = trimData(data, where((data['t'] >= trimRange[0]) & (data['t'] <= trimRange[1]))[0])

	if trialType == 'noplot':
		return data

	try:
		figure(int(filename[-9:-4]))
	except:
		figure()

	close('all')
	# Just plot all the data in rows
	# nrows = size(data.keys()) - 1
	nrows = 4
	currow = 1

	for key in data.keys():
		# For legibility open a new figure after 4
		if currow > 4:
			figure()
			currow = 1
		if key != 't' and key != 'st' and key != 'mo':
			subplot(nrows,1,currow)
			plot(0.001*data['t'], data[key])
			if 'mo' in data.keys():
				drawRegions(0.001*data['t'], data['mo'])
			if 'st' in data.keys():
				drawRegions(0.001*data['t'], data['st'])
			ylabel(key)
			currow = currow+1
			tight_layout()

	return data


#####################################################################################
''' SCRIPT '''

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Plot some logs.')
	parser.add_argument('filename', help='filename to open (LOGxxxxx.TXT)')
	# parser.add_argument('-p', dest='pathRoot', help='Path under which to search for LOG*****.TXT files')
	parser.add_argument('-e', dest='trialType', help='experiment type {hop,freefall} (default: plot all data)')
	parser.add_argument('-t', nargs=2, dest='trimt', type=int, help='trim by time (default: show all)', metavar=('tstart', 'tend'))
	parser.add_argument('-v', dest='vlineEvery', type=int, help='vertical line every')
	args = parser.parse_args()

	data = plotTrial(args.filename, args.trialType, args.trimt, args.vlineEvery)
	show()

	# A = aEstimator()

