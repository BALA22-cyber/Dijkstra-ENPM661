
import numpy as np
import cv2 as cv

#Map Space
map = np.zeros((250,400,3), np.uint8)
x_grid = np.arange(400)
y_grid = np.arange(250)

#Hexagon points
hexagon = np.array([[240, 80], [240, 120], [200, 145],[160, 120], [160, 80], [200, 55], [240, 80]], np.int32)
hex_center = np.array([200, 100])

##The Polygon is split up into two triangles
triangle1 = np.array([[31,185], [115,215], [85,180], [31,185]])
triangle1center = np.mean(triangle1[:-1], axis = 0)

triangle2 = np.array([[31,185], [105,95], [85,180], [31,185]])
triangle2center = np.mean(triangle2[:-1], axis = 0)


hex_img = np.array([[240, 129.79], [240, 170.20], [200, 195.41],[160, 170.20], [160, 129.79], [200, 104.58]], np.int32)


poly_img = np.array([[31,65], [115,35], [85, 70], [105,155]], np.int32)

cv.circle(map, (300, 65), 40, (255, 255, 255), thickness = -1)


cv.fillPoly(map, [hex_img],(255,255,255))
cv.fillPoly(map, [poly_img], (255,255,255))



def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

print("Solution started. Estimated wait time 1 minute")

###Defining obstacle space 
def obstacle(pt):
    x, y = pt
    if np.sqrt((x - 300)**2 + (y - 185)*2) <= 45 :
        return True
    
    ret = False
    for i in range(len(hexagon) - 1):
        ret = ret or intersect(pt, hex_center, hexagon[i], hexagon[i+1])
    if not ret: 
        return True

    ret = False
    for i in range(len(triangle1) - 1):
        ret = ret or intersect(pt, triangle1center, triangle1[i], triangle1[i+1])
    if not ret: 
        return True
    
    ret = False
    for i in range(len(triangle2) - 1):
        ret = ret or intersect(pt, triangle2center, triangle2[i], triangle2[i+1])
    if not ret: 
        return True    
    return False

###Defining 8 action_sets
def left (pos):
    i, j = pos
    cost = 1
    if i > 0 and not (obstacle(pos)):
        pos = (i - 1 , j)
        return pos, cost
    else:
        return None

def right (pos):
      i, j = pos
      cost = 1
      if i < 399 and not (obstacle(pos)):
          pos= (i + 1 , j)
          return pos, cost
      else:
          return None


def up (pos):
      i, j = pos
      cost = 1
      if j > 0 and not (obstacle(pos)):
          pos= (i  , j - 1)
          return pos, cost
      else:
          return None

def down (pos):
      i, j = pos
      cost = 1
      if j < 249 and not (obstacle(pos)):
          pos = (i, j + 1)
          return pos, cost
      else:
          return None


def up_left (pos):
      i, j = pos
      cost = 1.4
      if i > 0 and j > 0 and not (obstacle(pos)):
          pos = (i - 1, j - 1)
          return pos, cost
      else:
          return None

def up_right (pos):
      i, j = pos
      cost = 1.4
      if i < 399 and j > 0 and not (obstacle(pos) ):
          pos = (i + 1, j - 1)
          return pos, cost
      else:
          return None

def down_left (pos):
      i, j = pos
      cost = 1.4
      if i > 0 and j < 249 and not (obstacle(pos) ):
          pos= (i - 1, j + 1)
          return pos, cost
      else:
          return None

def down_right (pos):
      i, j = pos
      cost = 1.4
      if i < 399 and j < 249 and not (obstacle(pos) ):
          pos = (i + 1, j + 1)
          return pos, cost
      else:
          return None

all_actions = [up, down, left, right, down_left, down_right, up_left, up_right]


# Creating nodes and storing in a dictionary
def get_node( pos, parent, cost):
    Node  = {'pos': pos, 
             'parent': parent, 
             'cost': cost}
    return Node

def initial_nodes(start_pos):
  open_dict = {}
  for x in x_grid:
    for y in y_grid:    
        open_dict[(x, y)] = get_node((x,y), None, np.inf)
  open_dict[start_pos]['cost'] = 0
  return open_dict


######give the input points over here

##Djikstra Algorithm
def Djikstra (start_pos = (0, 0), goal_pos = (200,220)):
    All_nodes = initial_nodes(start_pos)
    open_dict = {start_pos: 0}
    closed_lis = {start_pos}
    explore=[start_pos]

    while len(open_dict):
        min_pos = min(open_dict, key = open_dict.get)
        closed_lis.add(min_pos)
        open_dict.pop(min_pos)
        min_node = All_nodes[min_pos]
        # gives the minimum position of nodes
        for action in all_actions:
            act_output = action(min_pos)
            if act_output is not None:
                pos, cost = act_output
                if pos not in closed_lis:
                    child_node = All_nodes[pos]
                    new_cost = min_node['cost'] + cost
                    if new_cost < child_node['cost']:
                        child_node['cost'] = new_cost
                        child_node['parent'] = min_pos
                        open_dict[pos] = new_cost
                        explore.append(child_node['pos'])
                    if pos == goal_pos: 
                        print("solution found")
                        return backtrack(All_nodes, pos),explore

#######BackTracking
def backtrack(All_nodes, pos):
    print("Tracking Back")
    A = []
    while All_nodes[pos]['parent'] is not None:
        A.append(pos)
        pos = All_nodes[pos]['parent']
    A.reverse()
    return A

####A = Backtracking
A,explore = Djikstra()

print(A)


for pos in A:
    map[249 - pos[1], pos[0]] = 100
# space = cv.circle(space, [start_pos], radius=0, color=(0, 0, 255), thickness=-1)
# space = cv.circle(space, [goal_pos], radius=0, color=(0, 0, 255), thickness=-1)

#visulizing the path explored nodes
def visualize(path, explore):
    ''' Visualise the exploration and the recently found path
    '''
    img = map
    h, w, _ = img.shape
    out = cv.VideoWriter('outputvideo.mp4',cv.VideoWriter_fourcc(*'mp4v'), 120.0, (w, h))
    
    for i in range(len(explore)):
        pos = (249 - explore[i][1], explore[i][0])
        img[pos] = [0, 0, 255]
        if i%100 == 0:
            out.write(img)
            cv.imshow('hi', img)
            cv.waitKey(1)
    for pos in path:
        pos = (249 - pos[1], pos[0])
        img[pos] = [0, 255, 0]
    for i in range(50): 
        out.write(img)
    out.release()
    cv.imshow('hi', img)
    cv.waitKey(0)

visualize(A, explore)
print("")



cv.imshow('Output',map )
cv.waitKey(0)