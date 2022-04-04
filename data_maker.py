f = open("log2.txt","w")

index_1 = 1023
index_2 = 0
index_3 = -200
index_4 = 0

for a in range(0,1000):
	if(a < 550):
		f.write("(0;(G,R);(0,526,"+str(index_1)+",90);(1,498,"+str(index_2)+",270));(2,(498,"+str(index_3)+",270));(3,("+str(index_4)+",526,0))\n")
	elif(a < 650):
		f.write("(0;(Y,R);(0,526,"+str(index_1)+",90);(1,498,"+str(index_2)+",270));(2,(498,"+str(index_3)+",270));(3,("+str(index_4)+",526,0))\n")
	else:
		f.write("(0;(R,G);(0,526,"+str(index_1)+",90);(1,498,"+str(index_2)+",270));(2,(498,"+str(index_3)+",270));(3,("+str(index_4)+",526,0))\n")

	index_1 = index_1 - 1
	index_2 = index_2 + 1
	if(a < 600):
		index_3 = index_3 + 1
	if(a < 200):
		index_4 = index_4 + 2
	elif(a >= 650):
		index_4 = index_4 + 1


f.close()