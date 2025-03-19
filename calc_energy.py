from CONSTANT import Sensor, Relay, Group
import pickle

import matplotlib.pyplot as plt

from math import isclose
import numpy as np


def outage_probability(
        K: int,
        alpha = 1,
        P = 1,
        Rth = .2,
        sigma = 1e-32
    ):
    x = np.log(2) * Rth * (K+1)
    numerator = sigma*sigma*(K+1)*(1-np.exp(x))
    denominator = alpha * P

    return 1 - np.exp(numerator/denominator)

def energy(no_transmit, no_receive, R):
    E_elec = 50*1e-9
    E_freespace = 10*1e-12
    K = 525*8
    result = no_receive*(E_elec)*K + no_transmit*(K*E_elec + K*E_freespace*R*R)
    return result


def calc_energy(data):
    max_E = float('-inf')
    min_E = float('inf')

    list_E = []

    for no_transmit, no_receive, R in data:
        energy_consumption = energy(no_transmit, no_receive, R)

        list_E.append(energy_consumption)

        if energy_consumption > max_E:
            max_E = energy_consumption
        
        if energy_consumption < min_E:
            min_E = energy_consumption
    

    return list_E, sum(list_E)/len(data), max_E-min_E


total_Relay = 0
data_transmit_receive_R = []
data_outage = []

class Node:
    def __init__(self, v):
        self.v: Sensor = v
        self.children: list[Node] = []

        self.n_relays: list = []

        self.relays: list[list] = []

        self.n_relay_to_base = 0

        self.n_transmit = 0
        self.is_leaf = False

    def build_tree(self, group: Group, i: int):
        global total_Relay

        if type(group) != Group:
            raise ValueError("Group must be an instance of Group class")

        for child_idx, child in enumerate(group.childs):

            self.n_relays.append(len(group.next_connection[i][child_idx]))

            self.relays.append(group.next_connection[i][child_idx])

            self.children.append(Node(child.T.Sensors[i]))
            
            self.children[-1].build_tree(child, i)

        # print(self.n_relays, len(group.children))
        t = sum(self.n_relays)
        total_Relay += t
    
    def remove_duplicate(self):
        while True:
            rm_child = []
            for child in self.children:
                if child.v.v == self.v.v:
                    rm_child.append(child)
                
            if rm_child:
                for child in rm_child:
                    idx = self.children.index(child)

                    if self.n_relays[idx] != 0:
                        print("here???")
                        print(self.n_relays[idx])
                        print(self.children[idx], self.v)

                    self.children.pop(idx)
                    self.n_relays.pop(idx)
                    self.relays.pop(idx)

                    self.children.extend(child.children)
                    self.n_relays.extend(child.n_relays)
                    self.relays.extend(child.relays)
            else:
                break
        
        for child in self.children:
            child.remove_duplicate()
        
        for idx in range(len(self.children)):
            if self.n_relays[idx] == 0:
                # print(self.n_relays)
                # print(self.v)
                # print("=====")
                # print(self.children)

                pass 
    
    def calc_data_transmit_receive(self, is_base = False):
        if len(self.children) == 0:
            self.n_transmit = len(self.v.Targets)
            self.n_receive = 0
            self.is_leaf = True

            data_transmit_receive_R.append((self.n_transmit, self.n_receive, 4))
            return 

        for child_idx, child in enumerate(self.children):
            child.n_relay_to_base += self.n_relays[child_idx] + self.n_relay_to_base + 1
            data_outage.append(child.n_relay_to_base)
            child.calc_data_transmit_receive()

        this_transmit = len(self.v.Targets)

        receive_from_child = sum([child.n_transmit for child in self.children])

        self.n_transmit = receive_from_child + this_transmit
        self.n_receive = receive_from_child

        if is_base:
            self.n_transmit = 0

        data_transmit_receive_R.append((self.n_transmit, self.n_receive, 4))

        for idx in range(len(self.children)):
            for relay in self.relays[idx]:
                data_transmit_receive_R.append((self.children[idx].n_transmit, self.children[idx].n_transmit, relay[2]))


    def plot_tree(self, current_depth = 0, max_depth = float('inf')):
        if current_depth >= max_depth:
            return
        for idx, child in enumerate(self.children):
            plt.plot([self.v.v[0], child.v.v[0]], [self.v.v[1], child.v.v[1]], c='green')
            
            plt.scatter(child.v.v[0], child.v.v[1], s=50, c = 'red')

            # for relay in self.relays[idx]:
                # plt.scatter(relay[0], relay[1], s=50, c='blue')


            child.plot_tree(current_depth = current_depth + 1, max_depth = max_depth)
    
    def __repr__(self):
        return f'{self.v}'


def get_tree(GVs):
    base = GVs[0]

    n_path = len(base.T.Sensors)

    trees: list[Node] = []

    tmp = 0
    for i in range(n_path):
        root = Node(base.T.Sensors[i])
        root.build_tree(base, i)
        root.remove_duplicate()
        root.calc_data_transmit_receive(is_base=True)
        tmp += root.n_receive
        trees.append(root)

        if is_plot:
            trees[-1].plot_tree()

    data_transmit_receive_R.sort()

    # print(data_transmit_receive_R)

    # print(tmp)

    # list_E, mean_E, delta_E = calc_energy(data_transmit_receive_R)
    # list_E.sort()
    # total_E = sum(list_E)

    # print(f'{total_E:.5f}, {mean_E:.5f}, {delta_E:.5f}')
    # print(f'{total_E:}')

    # print(np.mean(data_outage), np.max(data_outage), np.min(data_outage))

    # outages = [outage_probability(
    #     K,
    #     alpha=1,
    #     P=1,
    #     Rth=.2,
    #     sigma=1e-20
    #     ) for K in data_outage]

    # print(np.mean(outages), np.max(outages), np.min(outages))

    if is_plot:
        plt.show()


def main():
    global n, Rs, Rsc, Rc, Qmax, is_plot, Dim, data_outage, data_transmit_receive_R, total_Relay
    is_plot = False

    Dim = "3D"
    file = "bacgiang"

    algo = "GHS"

    n = 400
    Rs = 40
    Rc = Rs*2
    Rsc = Rs//10
    Qmax = 5
    Qsz = 80


    if Dim == "3D":
        with open(f"3D/{file}{algo}.pickle", "rb") as f:
            GVs: list[Group] = pickle.load(f)

        get_tree(GVs)  

        print(np.mean(data_outage), np.max(data_outage), np.min(data_outage))

        outages = [outage_probability(
            K,
            alpha=1,
            P=1,
            Rth=.2,
            sigma=1e-32
            ) for K in data_outage
        ]
        
        list_E, mean_E, delta_E = calc_energy(data_transmit_receive_R)
        list_E.sort()
        total_E = sum(list_E)

        print(f'{total_E:.5f}, {mean_E:.5f}, {delta_E:.5f}')

        print(f'{np.mean(outages):.5f}')  

    elif Dim == "2D":
        mean_total_outage = 0
        mean_total_E = 0
        mean_mean_E = 0
        mean_delta_E = 0
        for run in range(1, 21):
            if file == "T":
                filename = f'{file}//{n}_{Rs}_{Qmax}_{Qsz}_{run}.pickle'
            else:
                filename = f'{file}//{n}_{Rs}_{Qmax}_{run}.pickle'
                    
            with open(f'{algo}/{filename}', 'rb') as f:
                GVs: list[Group] = pickle.load(f)
            
            get_tree(GVs)

            # print(np.mean(data_outage), np.max(data_outage), np.min(data_outage))

            outages = [outage_probability(
                K,
                alpha=1,
                P=1,
                Rth=.2,
                sigma=1e-32
                ) for K in data_outage]

            mean_total_outage += np.mean(outages)

            list_E, mean_E, delta_E = calc_energy(data_transmit_receive_R)
            list_E.sort()
            total_E = sum(list_E)

            mean_total_E += total_E
            mean_mean_E += mean_E
            mean_delta_E += delta_E


            data_outage = []
            data_transmit_receive_R = []
            total_Relay = 0
        
        print(f'{mean_total_E/20:.5f}, {mean_mean_E/20:.5f}, {mean_delta_E/20:.5f}')
        print(f'{mean_total_outage/20:.5f}')

if __name__ == "__main__":
    main()
    # print(energy(0, 33, 40))


# (3, 0) -> 0.0008
# (3, 3) -> 0.0015
# (9, 6) -> 0.0037
# (9, 9) -> 0.0044
# (12, 9) -> 0.0052
# (12, 12) -> 0.0058
# (0, 12) -> 0.0025
# (33, 30) -> 0.015
# (33, 33) -> 0.016
# (0, 33) -> 0.007

# i.n_target = 3

# n = 10

# 1 -5> k -5> b
# 2 -5> k -5> b
# 3 -5> k -5> b
# ...
# n -5> k -5> b


# i.n_transmit = 3
# i.n_receive = 0
# -> 0.0008
# x5 time with (3, 3) -> 5*0.0015 = 0.0075

# xn = (0.0008+0.007)*n = 0.0078*n = 0.078

# 4.n_transmit = 3*n + 3 = 33
# 4.n_receive = 3*n = 30
# -> 0.015
# x5 time with (33, 33) -> 5*0.016 = 0.08

# b.n_transmit = 0
# b.n_receive = 33
# -> 0.007

# -> 0.078 + 0.08 + 0.007 = 0.165

# vs

# 1 -11> b
# 2 -11> b
# 3 -11> b
# 4 -5> b

# 1.n_transmit = 3
# 1.n_receive = 0
# -> 0.0008
# x11 time with (3, 3) -> 11*0.0015 = 0.0165

# xn = (0.0008+0.0165)*10 = 0.173

# 3.n_transmit = 3
# 3.n_receive = 0
# -> 0.0008
# x12 time with (3, 3) -> 11*0.0015 = 0.0165

# 4.n_transmit = 3
# 4.n_receive = 0 
# -> 0.0008
# x5 time with (3, 3) -> 5*0.0015 = 0.0075

# b.n_transmit = 0
# b.n_receive = 12
# -> 0.0025

# -> 0.0008*4 + 0.0025 + 0.0165*3 + 0.0075  = 0.0627