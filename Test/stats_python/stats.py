import numpy 
import scipy.stats
import matplotlib.pyplot as plt

def mean(list):
	sum = 0
	for element in list:
		sum = sum + element
	sum = sum / len(list)
	return sum

def variance(list, mean):
	sum_el = 0 
	for element in list:
		temp_el = element - mean
		temp_el = temp_el * temp_el
		sum_el = sum_el + temp_el
	sum_el = sum_el / (len(list) - 1)
	return sum_el
	
def sd(variance):
	standd = numpy.sqrt(variance)
	return standd

precision = list()
recall = list()

with open('sketchezlec.dat') as f:
    for line in f:
        data = line.split()
        print(data)
        precision.append(float(data[1]))
        recall.append(float(data[2]))

mean_v = mean(precision)
sd_v = sd(variance(precision, mean_v))
print("Precision mean and sd " + str(mean_v) + " "+ str(sd_v))

mean_recall = mean(recall)
sd_recall = sd(variance(recall, mean_recall))
print("Recall mean and sd " + str(mean_recall) + " "+ str(sd_recall))
