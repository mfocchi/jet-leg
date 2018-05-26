# -*- coding: utf-8 -*-
"""
Created on Sat May 26 15:09:24 2018

@author: rorsolino
"""
import cvxopt
from cvxopt import matrix, solvers
import numpy as np
# for plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
        
def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T
    
def getGraspMatrix(r):
    G = np.block([[np.eye(3), np.zeros((3,3))],
                   [skew(r), np.eye(3)]])
    return G
    
def linear_cone_constraint(n, mu):
    m = np.eye(3) - np.dot(n,np.transpose(n))
    u = np.dot(n,mu)
    #cone_constraints = m - np.transpose(u)
    cone_constraints = np.vstack((m - np.transpose(u), - m - np.transpose(u)))
    return cone_constraints
    
def normalize(n):
    norm1 = np.linalg.norm(n)
    n = np.true_divide(n, norm1)
    return n

nc = 3;
g = 9.81
mass = 100
grav = np.array([[0.], [0.], [-g*mass]])
# contact points
r1 = np.array([0.0, 10.0, 2.0])
r2 = np.array([10.0, -10.0, -2.0])
r3 = np.array([-10.0, -10.0, 1.0])
contacts = np.vstack((r1, r2, r3))
# contact surface normals
n1 = np.array([[0.0], [0.0], [1.0]])
n1 = normalize(n1)
n2 = np.array([[0.0], [0.0], [1.0]])
n2 = normalize(n2)
n3 = np.array([[0.0], [0.0], [1.0]])
n3 = normalize(n3)
normals = np.hstack((n1, n2, n3))

friction_coeff = 1.0

cons = np.zeros((0,0))
for j in range(0,nc):
    c = linear_cone_constraint(normals[:,j],friction_coeff)
    cons = np.block([[cons, np.zeros((np.size(cons,0),np.size(c,1)))],
                  [np.zeros((np.size(c,0),np.size(cons,1))), c]])
#print C
Q = 2*matrix(np.zeros((3*nc,3*nc)))
p = matrix(np.ones((3*nc,1)))
print Q, p
print np.size(Q,0), np.size(Q,1)

# Inequality constraints
m_ineq = 6*nc
#A=A.astype(double) 
#cons = cons.astype(np.double)
G = matrix(cons) #matrix([[-1.0,0.0],[0.0,-1.0]])
h = matrix(np.zeros((m_ineq,1)).reshape(m_ineq)) #matrix([0.0,0.0])
print G, h
print np.size(G,0), np.size(G,1)

feasible_points = np.zeros((0,3))
unfeasible_points = np.zeros((0,3))
# Equality constraints
for com_x in np.arange(-25,25,1.):
    for com_y in np.arange(-30,15,1.):
        com = np.array([com_x, com_y, 0.0])
        torque = -np.cross(com, np.transpose(grav))
        A = np.zeros((6,0))
        for j in range(0,nc):
            r = contacts[j,:]
            GraspMat = getGraspMatrix(r)
            A = np.hstack((A, GraspMat[:,0:3]))
        A = matrix(A)
        b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
        #A = matrix([1.0, 1.0], (1,2))
        #b = matrix(1.0)

        sol=solvers.lp(p, G, h, A, b)
        x = sol['x']
        status = sol['status']
        #print status
        if status == 'optimal':
            feasible_points = np.vstack([feasible_points,com])
        else:
            unfeasible_points = np.vstack([unfeasible_points,com])
        #print 'iteration ', com_x

# Plotting the results   
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(contacts[:,0], contacts[:,1], contacts[:,2],c='b',s=70)
for j in range(0,nc):
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[0,j]], [contacts[j,1], contacts[j,1]+normals[1,j]],[contacts[j,2], contacts[j,2]+normals[2,j]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)
    
if np.size(feasible_points,0) != 0:
    ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2],c='g',s=50)
if np.size(unfeasible_points,0) != 0:
    ax.scatter(unfeasible_points[:,0], unfeasible_points[:,1], unfeasible_points[:,2],c='r',s=50)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.draw()
plt.show()

print "bye bye" 
