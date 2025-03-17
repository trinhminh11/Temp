from CONSTANT import *
import matplotlib.pyplot as plt
from SPARTA_CC import *
import networkx as nx
import timeit
import random


def Plotdata(H, T, S, Rs):
	plt.xlabel('width')
	plt.ylabel("height")
	plt.xlim(0, H)
	plt.ylim(0, H)

	theta = np.linspace( 0 , 2 * np.pi , 150 )

	for Ti in T:
		if type(Ti) == Target:
			a = Rs * np.cos( theta ) + Ti.v[0]
			b = Rs * np.sin( theta ) + Ti.v[1]


			plt.plot(a,b, c = 'r')

			plt.scatter(Ti.v[0], Ti.v[1], c = 'r')

		for Si in Ti.Sensors:
			plt.scatter(Si.v[0], Si.v[1], c = 'orange')

class Vertex:
	def __init__(self, V):
		self.V: Cluster = V
		self.deg = 0

class Edge:
	def __init__(self, A, B):
		self.V1: Vertex = A
		self.V2: Vertex = B
		self.dist = dist(A.V.Center.v, B.V.Center.v)

class Cluster:
	def __init__(self, Center, Targets):
		self.Center: Target = Center
		self.Targets: list[Target] = Targets
		self.e = -1
		if type(Center) == Base:
			self.e = Qmax
		else:
			for Ti in [Center] + self.Targets:
				if Ti.q > self.e:
					self.e = Ti.q

	def insert_anchor_node(self, S: list, Rn: list):
		for i in range(self.e-self.Center.q):
			tempS = Sensor(self.Center.v, Rs, [self.Center])
			self.Center.Sensors.append(tempS)
			S.append(tempS)
			Rn.append(self.Center.v)

	def Put_relay(self, S: list, Rn: list):

		for Si in self.Center.Sensors:
			c = dist(Si.v, self.Center.v)
			add = int((c-0.0001)//(Rsc))
			for j in range(add):
				x = Si.v[0] + (j+1)*(self.Center.v[0]-Si.v[0])/(add+1)
				y = Si.v[1] + (j+1)*(self.Center.v[1]-Si.v[1])/(add+1)
				z = Si.v[2] + (j+1)*(self.Center.v[2]-Si.v[2])/(add+1)

				sensor = (x, y)
				Rn.append(sensor)

		ans = []
		self.insert_anchor_node(S, Rn)

		for Ti in self.Targets:
			used = []
			for Sij in Ti.Sensors:
				for Sk in self.Center.Sensors:
					if Sk not in Ti.Sensors and Sk not in used:
						ans.append([Sij, Sk])
						used.append(Sk)
						break


		for Si, Sj in ans:

			Rn.append(Put_Relay(Si, Sj))
			# A = Si
			# B = Sj
			# all_sec = False
			# all_commu = False
			# MA = []
			# MB = []
			# for T1 in Si.Targets:
			# 	for T2 in Sj.Targets:
			# 		P1 = T1.v
			# 		P2 = T2.v

			# 		RP1 = T1.Rsz
			# 		RP2 = T2.Rsz


			# 		M1 = circle_line_intersection(P1, RP1, A.v, B.v)


			# 		M2 = circle_line_intersection(P2, RP2, A.v, B.v)


			# 		for M in M1:
			# 			MA.append(M)

			# 		for M in M2:
			# 			MB.append(M)


			# if A.v[0] < B.v[0]:
			# 	MA.sort(reverse = True)
			# 	MB.sort()

			# 	if len(MA) > 0 and len(MB) > 0:
			# 		if MA[0] >= MB[0]:
			# 			all_sec = True

			# elif A.v[0] > B.v[0]:
			# 	MA.sort()
			# 	MB.sort(reverse = True)
			# 	if len(MA) > 0 and len(MB) > 0:
			# 		if MA[0] <= MB[0]:
			# 			all_sec = True

			# else:
			# 	if A.v[1] < B.v[1]:
			# 		MA.sort(reverse = True, key = lambda x: x[1])
			# 		MB.sort(key = lambda x: x[1])

			# 		if len(MA) > 0 and len(MB) > 0:
			# 			if MA[1] >= MB[1]:
			# 				all_sec = True

			# 	elif A.v[1] > B.v[1]:
			# 		MA.sort(key = lambda x: x[1])
			# 		MB.sort(reverse = True, key = lambda x: x[1])

			# 		if len(MA) > 0 and len(MB) > 0:
			# 			if MA[1] <= MB[1]:
			# 				all_sec = True

			# if len(MA) > 0:
			# 	MA = MA[0]
			# else:
			# 	MA = False

			# if len(MB) > 0:
			# 	MB = MB[0]
			# else:
			# 	MB = False

			# if not MA and not MB:
			# 	Ta = A.Targets
			# 	Tb = B.Targets

			# 	for T in Ta:
			# 		if dist(T.v, B.v) <= T.Rsz:
			# 			all_sec = True
			# 			break

			# 	for T in Tb:
			# 		if dist(A.v, T.v) <= T.Rsz:
			# 			all_sec = True
			# 			break

			# 	if not all_sec:
			# 		all_commu = True

			# if all_sec:
			# 	c = dist(A.v, B.v)
			# 	add = int((c-0.0001)//(Rsc))
			# 	for j in range(add):
			# 		x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
			# 		y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)

			# 		sensor = (x, y)
			# 		Rn.append(sensor)

			# elif all_commu:
			# 	c = dist(A.v, B.v)
			# 	add = int((c-0.0001)//(Rc))
			# 	for j in range(add):
			# 		x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
			# 		y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)

			# 		sensor = (x, y)

			# 		Rn.append(sensor)

			# else:
			# 	put_A = False
			# 	put_B = False
			# 	if MA and MB:
			# 		Rn.append(MA)
			# 		Rn.append(MB)
			# 		put_A = True
			# 		put_B = True
			# 		c = dist(MA, MB)
			# 		add = int((c-0.0001)//(Rc))

			# 		for j in range(add):
			# 			x = MA[0] + (j+1)*(MB[0]-MA[0])/(add+1)
			# 			y = MA[1] + (j+1)*(MB[1]-MA[1])/(add+1)

			# 			sensor = (x, y)

			# 			Rn.append(sensor)

			# 	elif MA and not MB:
			# 		Rn.append(MA)
			# 		put_A = True

			# 		c = dist(MA, B.v)
			# 		add = int((c-0.0001)//(Rc))

			# 		for j in range(add):
			# 			x = MA[0] + (j+1)*(B.v[0]-MA[0])/(add+1)
			# 			y = MA[1] + (j+1)*(B.v[1]-MA[1])/(add+1)

			# 			sensor = (x, y)
			# 			Rn.append(sensor)

			# 	elif MB and not MA:
			# 		Rn.append(MB)
			# 		put_B = True

			# 		c = dist(A.v, MB)
			# 		add = int((c-0.0001)//(Rc))

			# 		for j in range(add):
			# 			x = A.v[0] + (j+1)*(MB[0]-A.v[0])/(add+1)
			# 			y = A.v[1] + (j+1)*(MB[1]-A.v[1])/(add+1)

			# 			sensor = (x, y)
			# 			Rn.append(sensor)

			# 	if put_A:
			# 		c = dist(A.v, MA)
			# 		add = int((c-0.0001)//(Rsc))
			# 		for j in range(add):
			# 			x = A.v[0] + (j+1)*(MA[0]-A.v[0])/(add+1)
			# 			y = A.v[1] + (j+1)*(MA[1]-A.v[1])/(add+1)

			# 			sensor = (x, y)
			# 			Rn.append(sensor)

			# 	if put_B:
			# 		c = dist(B.v, MB)
			# 		add = int((c-0.0001)//(Rsc))
			# 		for j in range(add):
			# 			x = B.v[0] + (j+1)*(MB[0]-B.v[0])/(add+1)
			# 			y = B.v[1] + (j+1)*(MB[1]-B.v[1])/(add+1)

			# 			sensor = (x, y)
			# 			Rn.append(sensor)

			# plt.plot([Si.v[0], Sj.v[0]], [Si.v[1], Sj.v[1]], c = 'green')

		return ans

def Clustering(T, Rcl):
	n = len(T)
	C = []
	used = []
	while True:
		maxneigh = float("-inf")

		bestT = None
		bestNeighs = []

		for Ti in T:
			if Ti not in used:
				center = Ti
				neighbours = []
				for Tj in T:
					if Ti != Tj:
						if Tj not in used:
							if dist(Ti.v, Tj.v) <= Rcl:
								neighbours.append(Tj)

					if len(neighbours) > maxneigh:
						maxneigh = len(neighbours)
						bestT = center
						bestNeighs = neighbours

		if bestT == None:
			break

		C.append(Cluster(bestT, bestNeighs))
		used.append(bestT)
		used += bestNeighs

	# theta = np.linspace( 0 , 2 * np.pi , 150)

	# for Ci in C:
	# 	a = Rcl * np.cos( theta ) + Ci.Center.v[0]
	# 	b = Rcl * np.sin( theta ) + Ci.Center.v[1]

		# plt.plot(a,b, c = 'green')
		# plt.scatter(Ci.Center.v[0], Ci.Center.v[1], c = 'black')

	return C

def Construct_E(C: list[Cluster], base):
	B = Cluster(base, [])
	V = [Vertex(B)] + [Vertex(C[i]) for i in range(len(C))]
	# V = [Vertex(C[i]) for i in range(len(C))]
	L = []
	E: list[Edge] = []

	# for Vi in V:
	# 	P = Vi.V.Center.v
	# 	plt.scatter(P[0], P[1], c = 'black', s = 100)

	for i in range(1, len(V)):
		for j in range(i):
			L.append(Edge(V[i], V[j]))

	L.sort(key = lambda x: x.dist)

	for Li in L:
		V1, V2 = Li.V1, Li.V2

		if V1.deg < V1.V.e or V2.deg < V2.V.e:
			E.append(Li)
			V1.deg += 1
			V2.deg += 1


	for r in E:
		L.remove(r)


	for Ei in E:
		P1 = Ei.V1.V.Center
		P2 = Ei.V2.V.Center
		plt.plot([P1.v[0], P2.v[0]], [P1.v[1], P2.v[1]], c = 'black')


	for i in range(Qmax+1, len(V)):
		G = nx.DiGraph()
		for Ei in E:
			G.add_edge(Ei.V1, Ei.V2, capacity=1)
			G.add_edge(Ei.V2, Ei.V1, capacity=1)

		source = V[i]
		sink = V[0]

		max_flow_value = nx.maximum_flow(G, source, sink)[0]


		if max_flow_value < V[i].V.e:
			while max_flow_value < V[i].V.e:
				E.append(L[0])
				G.add_edge(L[0].V1, L[0].V2, capacity=1)
				G.add_edge(L[0].V2, L[0].V1, capacity=1)
				max_flow_value = nx.maximum_flow(G, source, sink)[0]
				L.remove(L[0])

	return E

def Put_relay_E(A, B, Rn):
	all_sec = False
	all_commu = False
	M1 = []
	M2 = []
	P1 = A.v
	P2 = B.v
	if type(A) != Base:
		RP1 = A.Rsz
		M1 = line_sphere_intersection(P1, RP1, P1, P2)
		# M1 = circle_line_intersection(P1, RP1, P1, P2)

	if type(B) != Base:
		RP2 = B.Rsz
		M2 = line_sphere_intersection(P2, RP2, P1, P2)
		# M2 = circle_line_intersection(P2, RP2, P1, P2)

	if M1:
		M1 = M1[0][:-1]
	
	if M2:
		M2 = M2[0][:-1]



	if M1 and M2:
		if P1[0] < P2[0]:
			if M1[0] >= M2[0]:
				all_sec = True
		elif P1[0] > P2[0]:
			if M1[0] <= M2[0]:
				all_sec = True
		else:
			if P1[1] < P2[1]:
				if M1[1] >= M2[1]:
					all_sec = True
			elif P1[1] > P2[1]:
				if M1[1] <= M2[1]:
					all_sec = True

	if not M1 and not M2:
		if type(A) == Base:
			Ta = []
		else:
			Ta = A.Sensors[0].Targets
		if type(B) == Base:
			Tb = []
		else:
			Tb = B.Sensors[0].Targets

		for T in Ta:
			if dist(T.v, B.v) <= T.Rsz:
				all_sec = True
				break

		for T in Tb:
			if dist(A.v, T.v) <= T.Rsz:
				all_sec = True
				break

		if not all_sec:
			all_commu = True

	if all_sec:
		c = dist(P1, P2)
		add = int((c-0.0001)//(Rsc))
		for j in range(add):
			x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
			y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)
			z = A.v[2] + (j+1)*(B.v[2]-A.v[2])/(add+1)

			sensor = (x, y, z)
			Rn.append(sensor)
	elif all_commu:
		c = dist(P1, P2)
		add = int((c-0.0001)//(Rc))
		for j in range(add):
			x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
			y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)
			z = A.v[2] + (j+1)*(B.v[2]-A.v[2])/(add+1)

			sensor = (x, y, z)

			Rn.append(sensor)

	else:
		put_A = False
		put_B = False
		if M1 and M2:
			Rn.append(M1)
			Rn.append(M2)
			put_A = True
			put_B = True
			c = dist(M1, M2)
			add = int((c-0.0001)//(Rc))

			for j in range(add):
				x = M1[0] + (j+1)*(M2[0]-M1[0])/(add+1)
				y = M1[1] + (j+1)*(M2[1]-M1[1])/(add+1)
				z = M1[2] + (j+1)*(M2[2]-M1[2])/(add+1)

				sensor = (x, y, z)

				Rn.append(sensor)
		elif M1 and not M2:
			Rn.append(M1)
			put_A = True

			c = dist(M1, B.v)
			add = int((c-0.0001)//(Rc))

			for j in range(add):
				x = M1[0] + (j+1)*(B.v[0]-M1[0])/(add+1)
				y = M1[1] + (j+1)*(B.v[1]-M1[1])/(add+1)
				z = M1[2] + (j+1)*(B.v[2]-M1[2])/(add+1)

				sensor = (x, y, z)
				Rn.append(sensor)

		elif M2 and not M1:
			Rn.append(M2)
			put_B = True

			c = dist(A.v, M2)
			add = int((c-0.0001)//(Rc))

			for j in range(add):
				x = A.v[0] + (j+1)*(M2[0]-A.v[0])/(add+1)
				y = A.v[1] + (j+1)*(M2[1]-A.v[1])/(add+1)
				z = A.v[2] + (j+1)*(M2[2]-A.v[2])/(add+1)


				sensor = (x, y, z)
				Rn.append(sensor)

		if put_A:
			c = dist(A.v, M1)
			add = int((c-0.0001)//(Rsc))
			for j in range(add):
				x = A.v[0] + (j+1)*(M1[0]-A.v[0])/(add+1)
				y = A.v[1] + (j+1)*(M1[1]-A.v[1])/(add+1)
				z = A.v[2] + (j+1)*(M1[2]-A.v[2])/(add+1)


				sensor = (x, y, z)
				Rn.append(sensor)

		if put_B:
			c = dist(B.v, M2)
			add = int((c-0.0001)//(Rsc))
			for j in range(add):
				x = B.v[0] + (j+1)*(M2[0]-B.v[0])/(add+1)
				y = B.v[1] + (j+1)*(M2[1]-B.v[1])/(add+1)
				z = B.v[2] + (j+1)*(M2[2]-B.v[2])/(add+1)


				sensor = (x, y, z)
				Rn.append(sensor)

def Put_relay_E(A: Target, B: Target | Base):
	res = []
	if type(B) == Base:
		S = A.Sensors[0]
		MA = []
		for T in S.Targets:
			# MA.extend(circle_line_intersection(T.v, T.Rsz, S.v, B.v))
			MA.extend(line_sphere_intersection(T.v, T.Rsz, S.v, B.v))
		if MA:
			farA = max(MA, key= lambda x: x[-1])

			if dist(S.v, B.v) <= farA[-1]:
				res.extend(addRelay(S.v, B.v, Rsc))
			else:
				res.extend(addRelay(S.v, farA[:-1], Rsc))
				res.extend(addRelay(farA[:-1], B.v, Rc))
	
	else:
		res.extend(Put_Relay(A.Sensors[0], B.Sensors[0]))
	
	return res

def addRelay(A, B, r):
	res = []
	c = dist(A, B)

	add = int((c-0.0001)//(r))

	for j in range(add):
		sensor = []

		for k in range(len(A)):
			sensor.append(A[k] + (j+1)*(B[k]-A[k])/(add+1))

		res.append(sensor)
	
	return res

def Put_Relay(S1: Sensor, S2:Sensor):
	if type(S2) == int or type(S1) == int or S1.v == S2.v:
		return []

	temp_Rn = []

	all_sec = False
	all_commu = False

	M1 = []
	M2 = []

	for T1 in S1.Targets:
		for T2 in S2.Targets:
			P1 = T1.v
			P2 = T2.v

			RP1 = T1.Rsz
			RP2 = T2.Rsz

			# M1 = circle_line_intersection(P1, RP1, S1.v, S2.v)
			# M2 = circle_line_intersection(P2, RP2, S1.v, S2.v)

			M1.extend(line_sphere_intersection(P1, RP1, S1.v, S2.v))
			M2.extend(line_sphere_intersection(P2, RP2, S1.v, S2.v))

	if M1:
		if M2:
			farA = max(M1, key= lambda x: x[-1])
			nearB = min(M2, key= lambda x: x[-1])



			if farA[-1] <= nearB[-1]:
				temp_Rn += addRelay(farA[:-1], nearB[:-1], Rc)


				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)

				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)


			else:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)

		else:
			farA = max(M1, key= lambda x: x[-1])
			
			if dist(S1.v, S2.v) <= farA[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)
				temp_Rn += addRelay(farA[:-1], S2.v, Rc)

	else:
		if M2:
			nearB = min(M2, key= lambda x: x[-1])
			if dist(S1.v, S2.v) < nearB[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, nearB[:-1], Rc)
				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)
		else:
			temp_Rn += addRelay(S1.v, S2.v, Rsc)
			print("?")
			print(S1.v)
			print(S2.v)
			print(*S1.Targets)
			print(*S2.Targets)
			print(Rs)
			print(S1.Targets[0].Rsz, S1.Targets[1].Rsz)
			print(S2.Targets[0].Rsz)
			exit()
			# print("?")
			# print(S1, S2)
			# print(*S1.Targets)
			# print(*S2.Targets)
			# print(M1, M2, RP1, RP2)
			# print("WTF3")
			# exit()
			# return []
	
	return temp_Rn


def main():
	import pickle
	global n, Rs, Rsc, Rc, Rcl, Qmax, is_plot

	change = "R"

	file = "hanoi"

	is_plot = False

	base = Base([0, 0])

	n = 400
	Rs = 40
	Qmax = 5
	Rcl = 100

	res_node = []
	res_time = []

	if change == "N":
		n = 100
	if change == "R":
		Rs = 0
	if change == "Q":
		Qmax = 0


	for _ in range(dataset_num):
		if change == "N":
			n += n_step
		if change == "R":
			Rs += Rs_step
		
		if change == "Q":
			Qmax += Qmax_step


		totalRn = 0
		totaltime = 0
		Rsc = Rs/10
		Rc = Rs*2


		
		for run in range(data_num):
			print(f'{change}//{n}_{Rs}_{Qmax}_{run+1}', end = "\r")

			with open(f'{change}//{n}_{Rs}_{Qmax}_{run+1}.pickle', 'rb') as f:
				T = pickle.load(f)

			starttime = timeit.default_timer()

			C: list[Cluster] = Clustering(T, Rcl)

			Rn = []

			S = []
			for Ti in T:
				for Si in Ti.Sensors:
					if type(Si) == Sensor and Si not in S:
						S.append(Si)

			for Ci in C:
				Ci.Put_relay(S, Rn)

			E = Construct_E(C, base)

			if is_plot:
				Plotdata(H, T, Rs)

			for Ei in E:
				# q = max(len(Ei.V1.V.Center.Sensors), len(Ei.V2.V.Center.Sensors))
				res = Put_relay_E(Ei.V1.V.Center, Ei.V2.V.Center)
				# for __ in range(q):x
				Rn.extend(res)

			endtime = timeit.default_timer()

			totalRn += len(Rn)
			totaltime += endtime - starttime
		
		print()

		totalRn = round(totalRn / data_num)
		totaltime = round(totaltime / data_num, 5)

		# print(n, Rs, Rc, Rsc, Qmax, f'{Rs} - {Rs+Rs}', end = "\t")
		res_node.append(totalRn)
		res_time.append(totaltime)

	print(*res_node, sep = "\n")
	print(*res_time, sep = "\n")


def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	n = 400
	Rs = 40
	Rsc = 4
	Rc = 80
	Qmax = 5
	is_plot = False
	Rcl = 100
	
	base = Base([0, 0, 0])

	file = "sonla"
	with open(f"3D//{file}.pickle", "rb") as f:
		T: list[Target] = pickle.load(f)

	starttime = timeit.default_timer()

	Rn = []

	S = []

	C: list[Cluster] = Clustering(T, Rcl)
	for Ti in T:
		for Si in Ti.Sensors:
			if type(Si) == Sensor and Si not in S:
				S.append(Si)

	for Ci in C:
		Ci.Put_relay(S, Rn)

	E = Construct_E(C, base)

	if is_plot:
		Plotdata(H, T, Rs)

	for Ei in E:
		q = max(len(Ei.V1.V.Center.Sensors), len(Ei.V2.V.Center.Sensors))
		res = Put_relay_E(Ei.V1.V.Center, Ei.V2.V.Center)
		for __ in range(q):
			Rn.extend(res)


	endtime = timeit.default_timer()

	print(len(Rn))
	print(endtime-starttime)

if __name__ == "__main__":
	main()
