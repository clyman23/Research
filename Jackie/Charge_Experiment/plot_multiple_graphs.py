# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 16:06:53 2016

@author: clyman
"""

'''
Plot a figure of charge used versus encoder counts for each test
'''

from matplotlib import pyplot
from matplotlib import rcParams, cm
rcParams['font.family'] = 'serif'
rcParams['font.size'] = 16
import csv
import numpy
import os.path

lengths = numpy.array([13, 23, 33, 43, 53])

for ind, j  in enumerate(lengths):
    charge_totals = numpy.array([])
    
    for i in range(0,15):
        #filename = '{0:02d}_gravel_Feb28_{1:02d}.csv'.format(j,i)
        filename = '{0:02d}GrassApr13_{1:02d}.csv'.format(j,i) 
        figname = '{0:02d}GrassApr13_{1:02d}.png'.format(j,i)
        if os.path.isfile(filename) is True:
            with open(filename, 'rb') as f:
                mycsv = csv.reader(f)
                mycsv = list(mycsv)
                row_count = len(mycsv) - 2
                start_at = float(mycsv[1][0])
            
            f = open(filename, 'rb')
            reader = csv.reader(f)
            rownum = 0
            
            encoder = numpy.array([])
            seconds = numpy.array([])
            amps = numpy.array([])
            
            for r in reader:
                #print(r[1])
                if rownum>0 and rownum<row_count and r != []:
                    encoder = numpy.append(encoder, float(r[0]) - start_at)
                    seconds = numpy.append(seconds, float(r[1]))
                    amps = numpy.append(amps, float(r[2]))
                rownum +=1
            f.close()
            charge = seconds*amps
            tot = numpy.cumsum(charge)
            
            figure = pyplot.figure()
            pyplot.plot(encoder, tot , 'b-');
            pyplot.xlabel('distance(encoder counts)');
            pyplot.ylabel('charge C' );
            pyplot.savefig(figname)
            pyplot.close(figure)
            pyplot.show()
            charge_totals = numpy.append(charge_totals, tot[-1])
