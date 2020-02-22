import random
import csv
service=['RESCUE','MEDICINE','FOOD']

cells=['A2','B3','C1','F2','D4','F6','C5','B6']

tsv=[]

for i in range(8):
	tsv.append([cells[i],random.choice(service),random.randint(1,4)])
	
for k in range(2):
	for i in range(8):
		if tsv[8*k+i][1]=='RESCUE':
			tsv.append([cells[i],random.choice(service),tsv[8*k+i][2]+10+random.randint(1,4)])
		else:
			tsv.append([cells[i],random.choice(service),tsv[8*k+i][2]+30+random.randint(1,4)])
		
		
with open('LED_Timing.tsv','wt') as outfile:
	tsv_writer=csv.writer(outfile,delimiter='\t')
	for i in tsv:
		tsv_writer.writerow(i)
