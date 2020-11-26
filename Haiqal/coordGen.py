import random

counter = 0
file1 = open("coordinates.txt","w") 

while counter<100000:
    x = random.triangular()
    y = random.gauss(0,3)
    file1.write(str(x)+" "+str(y)+"\n") 
    counter+=1