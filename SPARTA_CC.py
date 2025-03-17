import numpy as np

from CONSTANT import *
from Q_Coverage import Q_Coverage
from ImportData import *
import random
from math import dist
import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(10000)


random.seed(random_seed)
base = [0, 0, 0]

class Vertex(object):
	def __init__(self, T, index):
		self.T = T
		self.v = T.v
		self.neigh = []
		self.q = T.q
		self.index = index
		self.p = None

def find_set(v, parent):
	if v == parent[v]: 
		return v
	p = find_set(parent[v], parent)
	parent[v] = p
	return p

def union_sets(a, b, parent):
	a = find_set(a, parent)
	b = find_set(b, parent)
	if (a != b):
		parent[b] = a


def Cluster(T, Rs):
	C = []

	n = len(T)
	parent = [i for i in range(n)]


	V = [Vertex(T[i], i) for i in range(n)]
	E = []
	for i in range(n-1):
		for j in range(i+1, n):
			if dist(V[i].v, V[j].v) <= 2*Rs:
				if find_set(i, parent) == find_set(j, parent):
					continue
				union_sets(i, j, parent)

	for i in range(n):
		V[i].p = find_set(i, parent)

	V.sort(key = lambda x: x.p)
	minp = V[0].p
	maxp = V[-1].p
	Vindex = 0
	for p in range(minp, maxp+1):
		C.append([])

		while Vindex < n and V[Vindex].p == p:
			C[p-minp].append(V[Vindex].T)
			Vindex += 1

	temp = C.count([])
	for i in range(temp):
		C.remove([])

	return C


def SPARTA_CC(T, Rs):
	C = Cluster(T, Rs)
	S = []
	Tc = []
	for ii in range(len(C)):
		Tc.append([])
		for jj in range(len(C[ii])):
			Tc[ii].append(C[ii][jj])

	for ii in range(len(C)):
		Sq  = Q_Coverage(Tc[ii], Rs)
		S += Sq

	return S

def Plotdata(W, H, T, S, Rs):
	plt.xlabel('width')
	plt.ylabel("height")
	plt.xlim(0, W+Rs)
	plt.ylim(0, H+Rs)
	# fig, ax = plt.subplots()

	theta = np.linspace(0 , 2 * np.pi , 150 ) 
	radius = Rs

	for i in range(len(T)):
		a = T[i].v[0] + radius * np.cos( theta )
		b = T[i].v[1] + radius * np.sin( theta )

		plt.annotate(T[i].q, (T[i].v[0], T[i].v[1]))
		plt.plot(a, b)

	for i in range(len(S)):
		plt.scatter(S[i].v[0], S[i].v[1], s = 50)

	plt.show()


def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	change = "R"


	base = Base([0, 0])

	n = 200
	Rs = 40
	Qmax = 6

	if change == "N": 
		n = 100
	if change == "R":
		Rs = 40
	if change == "Q":
		Qmax = 0
	
	Rc = 60
	temp = 30

	for _ in range(dataset_num):

		if change == "N":
			n += n_step
		if change == "R":
			Rc += Rs_step
		if change == "Q":
			Qmax += Qmax_step
		
		temp += 10

		totalRn = 0
		totaltime = 0
		# Rsc = Rs/10
		# Rc = Rs*2

		# print(f'{change}//{n}_{Rs}_{Rc}_{Qmax}')

		res = 0

		print(change, n, Rs, Rc, Qmax)

		for run in range(data_num):
			T = []
			# Rsz = [random.random()*temp+temp for i in range(n)]
			Rsz = [0 for i in range(n)]

			for i in range(n):
				x,y = random.random()*(H-5)+5, random.random()*(H-5)+5
				T.append([x,y])

			# Q = [random.randint(1, Qmax) for i in range(n)]

			Q = [Qmax for i in range(n)]
			

			T = [Target(T[i], Q[i], Rsz[i], []) for i in range(len(T))]


			S: list[Sensor] = SPARTA_CC(T, Rs)

			res += len(S)

			with open(f'Temp//{change}//{n}_{Rs}_{Rc}_{Qmax}_{run+1}.pickle', 'wb') as f:
				pickle.dump(T, f)
		print(round(res/data_num))
			

if __name__ == "__main__":
	main()

	
