from CONSTANT import Sensor, Relay, Group
import pickle

import matplotlib.pyplot as plt

def energy(no_transmit, no_receive, R):
    E_elec = 50*1e-9
    E_freespace = 10*1e-12
    K = 525*8
    return no_receive*(E_elec)*K + no_transmit*(K*E_elec + K*E_freespace*R*R)



def calc_energy(data, R):
    total_E = 0
    max_E = float('-inf')
    min_E = float('inf')

    for values in data:
        no_transmit, no_receive = values

        energy_consumption = energy(no_transmit, no_receive, R)

        total_E += energy_consumption

        if energy_consumption > max_E:
            max_E = energy_consumption
        
        if energy_consumption < min_E:
            min_E = energy_consumption
    
    return total_E


total_Relay = 0

class Node:
    def __init__(self, v):
        self.v: Sensor | Relay = v
        self.childs: list[Node] = []

        self.n_relays: list[int] = []


    def build_tree(self, group: Group, i: int):
        global total_Relay

        if type(group) != Group:
            raise ValueError("Group must be an instance of Group class")


        for child_idx, child in enumerate(group.childs):
            self.n_relays.append(len(group.T.Sensors[i].next_connection[child_idx]))
                
            self.childs.append(Node(child.T.Sensors[i]))
            
            self.childs[-1].build_tree(child, i)

        # print(self.n_relays, len(group.childs))
        total_Relay += sum(self.n_relays)


    def plot_tree(self):
        for child in self.childs:
            plt.plot([self.v.v[0], child.v.v[0]], [self.v.v[1], child.v.v[1]], c='green')
            child.plot_tree()
    
    def __repr__(self):
        return f'{self.v}'


def get_tree(GVs):
    base = GVs[0]

    n_path = len(base.T.Sensors)

    trees: list[Node] = []

    for i in range(n_path):
        root = Node(base.T.Sensors[i])
        root.build_tree(base, i)
        trees.append(root)

        trees[-1].plot_tree()
    
    print(total_Relay)
    # plt.show()




def main():
    global n, Rs, Rsc, Rc, Qmax, is_plot, Dim

    Dim = "3D"
    file = "hanoi"

    algo = "GHS"

    n = 400
    Rs = 40
    Rc = Rs*2
    Rsc = Rs//10
    Qmax = 5

    if Dim == "3D":
        with open(f"3D/{file}{algo}.pickle", "rb") as f:
            GVs: list[Group] = pickle.load(f)

        trees = get_tree(GVs)     

    elif Dim == "2D":
        Rs = 0
        Rc = 0

if __name__ == "__main__":
    main()

