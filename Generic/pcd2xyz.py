import sys
import csv


if (len(sys.argv) < 2):
   print "usage: pcd2xyz.py [pcd filename]"
   exit()

fin = open(sys.argv[1])
reader = csv.reader(fin,delimiter=' ')
# strip file extension
outfilename = sys.argv[1][0:-4] + '.xyz'
outfile = open(outfilename,'w')
fout = csv.writer(outfile,delimiter=' ')

# skip header
for i in range(11):
  next(reader,None)

for row in reader:
  fout.writerow(row[0:3])
  


