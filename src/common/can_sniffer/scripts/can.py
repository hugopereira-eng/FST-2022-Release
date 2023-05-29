import csv
import time
import sys

f1 = open('candump.log', 'w')
with open(sys.argv[1]) as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        if row[2]:
            f1.write("(" + str(format(time.time()+float(row[0]),'f')) + ")" + " can" + str(int(not int(row[1])-1)) + " " + "{:03x}".format(int(row[2])).upper() + "#")
        else:
            continue
        for i in range(5,13):
            if row[i]:
                f1.write("{:02x}".format(int(row[i])).upper())
            else:
                break
        f1.write('\n')
f1.close()
