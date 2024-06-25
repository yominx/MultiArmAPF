import numpy as np 
import matplotlib.pyplot as plt
# robot_nums = [2, 3,4,5,6]


def make_result(prefix,postfix):
	result = []
	collision_info = []
	for robotnum in robot_nums:
		f = open(f'{prefix}{robotnum}{postfix}','r')
		# f = open(f'{robotnum}robot_log_noAPF.txt','r')
		lines = f.readlines()
		
		successed = 0
		total = len(lines)-1
		if total==0: continue
		means = []
		times = []
		collision=0
		for line in lines:
			divided = line.split('  ')
			if divided[0] == "mean":
				continue
			if divided[0] == "-1":
				collision+=1
			if divided[0] != "0" and divided[0] != "-1":
				successed += 1
				means.append(float(divided[0]))
				times.append(float(divided[2]))

		if len(means)==0: continue

		means = np.array(means)
		times = np.array(times)
		normalized_means = means/robotnum*1000

		res = (robotnum, float(successed / total), np.mean(normalized_means), 
				np.std(normalized_means), np.mean(times), np.std(times))
		result.append(res)
		collision_rate = collision / total
		collision_info.append(collision_rate)
	return np.array(result), collision_info

# result3 = make_result("data/", "UR5_apftimeonlyaa.txt")
# result4 = make_result("data/", "UR5_apftimeonly_noapf.txt")
robot_nums = [2,3,4,5,6,7,8,9]
# robot_nums = [3]

result3 ,collision3 = make_result("data_final/APF", ".txt")
result4 ,collision4 = make_result("data_final/RRT", ".txt")
# result3 = make_result("data/", "robot_log.txt")
# result4 = make_result("data/", "robot_log_noAPF.txt")

np.set_printoptions(precision=3)

print(collision3)
print(collision4)
# result4[7,1] = 0.91
# plt.plot(result3[:,0], result3[:,1], 'o-',label = 'Ours',    			color = [0.8,0.2,0.2])
# plt.plot(result4[:,0], result4[:,1], 'o-',label = 'RRT-only',    		color = [0.2,0.2,0.8])
plt.plot(result3[:,0], collision3, 'o-',label = 'Collision(ours)',    	color = [0.8,0.4,0.4])
plt.plot(result4[:,0], collision4, 'o-',label = 'Collition(RRT-only)',	color = [0.4,0.4,0.8])
# plt.plot(result3[:,0], result3[:,2],    color = 'red')
# plt.plot(result4[:,0], result4[:,2], label = 'RRT',    color = 'blue')
# plt.fill_between(result3[:,0],  result3[:,2]-result3[:,3],   result3[:,2]+result3[:,3],    alpha = 0.2,  color = 'red')
# plt.fill_between(result4[:,0], result4[:,2]-result4[:,3], result4[:,2]+result4[:,3],  alpha = 0.2,  color = 'blue')

# plt.plot(result3[:,0], result3[:,1]*100,   label = 'Proposed',  color = 'red')
# plt.plot(result4[:,0], result4[:,1]*100,   label = 'RRT',  color = 'blue')
# plt.show()

# plt.plot(result[:,0], result[:,4],   label = 'Proposed',  color = 'red')
# plt.plot(result3[:,0], result3[:,2],   label = 'Proposed',  color = 'red')
# plt.fill_between(result3[:,0],  result3[:,2]-result3[:,3],   result3[:,2]+result3[:,3],    alpha = 0.2,  color = 'red')
# plt.plot(result4[:,0], result4[:,4],   label = 'noAPF',     color = 'blue')

plt.xlabel('Number of Robots', fontsize=15)
plt.ylabel('Success Rate(%)', fontsize=15)
# plt.ylabel('Time to apply APF(ms)', fontsize=15)
plt.legend()
# plt.xlim([2, 9])
_, ymax = plt.ylim()
# plt.ylim([80, ymax])
plt.show()


