import csv
import matplotlib.pyplot as plt
import numpy as np


plt.ylim(0, 1.1)
# seeds = [25, 33, 42, 50, 51, 68, 90]
seeds = [68, 25]
for seed in seeds:
	scores = []
	mean_scores = []
	csv_file = open('seed%s_scores.csv' % seed, 'r')
	reader = csv.reader(csv_file)
	for idx, row in enumerate(reader):
		mean_score = row[0]
		scores.append(float(mean_score))
		if idx < 100:
			mean_scores.append(sum(scores) * 1. / len(scores))
		else:
			mean_scores.append(sum(scores[idx-100:]) / 100.)

	
	plt.plot(mean_scores)

plt.legend(['Seed 68', 'Seed 25'])
plt.show()

