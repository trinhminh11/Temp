data_num = 20     # Run how many time per 1 Dataset
dataset_num = 7  # Dataset
H = 1000		  # Region

n_step = 50		  # step of n
Rs_step = 20		  # step of Rs
Qmax_step = 2	  # step of Qmax

random_seed = 0


from math import sqrt
import matplotlib.pyplot as plt


class Sensor:
    def __init__(self, v, R = 0, Targets = []):
        self.v = v
        self.Targets: list[Target] = Targets
        self.locked = False

    def __repr__(self):
        return f'Sensor: {self.v}'
    
    

class Target:
    def __init__(self, v, q = 0, Rsz = 0, Sensors = []):
        self.v = v
        self.q = q
        self.Sensors: list[Sensor] = Sensors
        self.Rsz = Rsz
    
    def __str__(self) -> str:
        return f'Target: {self.v}'

class Base:
    def __init__(self, v):
        self.v = v

        self.Sensors = []

class Relay:
    def __init__(self, v, R):
        self.v = v
        self.R = R

class Group:
	def __init__(self, T : Target, index):
		self.parent: Group = None
		self.childs: list[Group] = []
		self.index: int = index
		self.T = T
		self.next_connection: list[list] = None
		self.depth = 0

		if type(T) == Target:
			n = len(T.Sensors)
			self.mid = [0] * len(self.T.v)

			
			count = 0
			for i in range(n):
				if self.T.Sensors[i] != 0:
					for j in range(len(self.mid)):
						self.mid[j] += self.T.Sensors[i].v[j]
					count  += 1

			for j in range(len(self.mid)):
				self.mid[j] /= count


		if type(T) == Base:
			self.mid = T.v
	
	def init_connection(self):
		self.next_connection = [[None for _ in range(len(self.childs))] for _ in range(len(self.T.Sensors))] # [i][j] with i is i-th sensor and j is j-th child

	def find_child(self, path, GVs):
		for i in range(len(path)):

			if path[i][0] == self.index and self.parent != GVs[path[i][1]]:
				self.childs.append(GVs[path[i][1]])
				GVs[path[i][1]].parent = self
				GVs[path[i][1]].depth = self.depth + 1


			if path[i][1] == self.index and self.parent != GVs[path[i][0]]:
				self.childs.append(GVs[path[i][0]])
				GVs[path[i][0]].parent = self
				GVs[path[i][0]].depth = self.depth + 1

		for i in range(len(self.childs)):
			self.childs[i].find_child(path, GVs)
		
	def draw_path(self):
		for child in self.childs:
			plt.plot([self.mid[0], child.mid[0]], [self.mid[1], child.mid[1]])

		plt.annotate(str(self.depth), self.mid)	

def circle_line_intersection(circle_center, circle_radius, line_start, line_end):
    try:
        dx = line_end[0] - line_start[0]
        dy = line_end[1] - line_start[1]
        line_length = (dx**2 + dy**2)**.5
        line_direction = (dx/line_length, dy/line_length)

        cx, cy = circle_center
        x1, y1 = line_start
        x2, y2 = line_end

        dx = x2 - x1
        dy = y2 - y1

        a = dx**2 + dy**2
        b = 2 * (dx * (x1 - cx) + dy * (y1 - cy))
        c = (x1 - cx)**2 + (y1 - cy)**2 - circle_radius**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            # No intersection
            return []

        t1 = (-b + (discriminant)**.5) / (2 * a)
        t2 = (-b - (discriminant)**.5) / (2 * a)

        intersections = []

        if 0 <= t1 <= 1:
            intersections.append((x1 + t1*dx, y1 + t1*dy))
        if 0 <= t2 <= 1:
            intersections.append((x1 + t2*dx, y1 + t2*dy))

        return intersections

    except:
        return []

def circle_line_intersection(circle_center, circle_radius, pt1, pt2, full_line=False, tangent_tol=1e-9):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            [cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2]
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]
            
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:  # If line is tangent to circle, return just one point (as both intersections have same location)
            A = pt1
            B = pt2
            C = intersections[0]
            d = sqrt((A[0]-C[0])**2 + (A[1]-C[1])**2)
            vAB = [B[0]-A[0], B[1]-A[1]]
            vAC = [C[0]-A[0], C[1]-A[1]]

            if vAB[0]*vAC[0] < 0 or vAB[1]*vAC[1] < 0:
                raise ValueError('WTF')
                return [intersections[0] + [-d]]
            else:
                return [intersections[0] + [d]]
        else:
            for i in range(len(intersections)):
                A = pt1
                B = pt2
                C = intersections[i]
                d = sqrt((A[0]-C[0])**2 + (A[1]-C[1])**2)
                vAB = [B[0]-A[0], B[1]-A[1]]
                vAC = [C[0]-A[0], C[1]-A[1]]
                if vAB[0]*vAC[0] < 0 or vAB[1]*vAC[1] < 0:
                    raise ValueError('WTF')
                    intersections[i] += [-d]
                else:
                    intersections[i] += [d]

            return intersections

def scalar(p, k):
    res = []
    for a in p:
        res.append(a*k)
    return res

def add(p1, p2):
    res = []
    for a, b in zip(p1, p2):
        res.append(a+b)
    
    return res

def sub(p1, p2):
    res = []
    for a, b in zip(p1, p2):
        res.append(a-b)
    
    return res

def dot(p1, p2):
    res = 0
    for a, b in zip(p1, p2):
        res += a*b
    
    return res


def line_sphere_intersection(center, R, start, end):
    if R == 0:
        return []
    
    u = sub(end, start)

    length = pow(dot(u, u), .5)

    if length == 0:
        return []


    u = scalar(u, 1/length)

    t = sub(start, center)

    
    b = dot(u, t)



    delta = b*b - dot(t, t) + R*R


    lst = []


    if delta == 0:
        lst.append(-b)

    else:
        d1 = -b + pow(delta, .5)
        d2 = -b - pow(delta, .5)

        lst += [d1, d2]

    x = []

    for d in lst:
        if d < 0:
            continue
        x.append(add(start, scalar(u, d))+[d])

    return x

