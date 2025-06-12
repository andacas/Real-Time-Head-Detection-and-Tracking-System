import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# -------- constants copied from the header ----------
R, T, L, Cx = 0.13, 0.02, 0.13, 0.06

def cube(v):
    p = np.array([[v[0],v[1],v[2]],[v[3],v[1],v[2]],
                  [v[3],v[4],v[2]],[v[0],v[4],v[2]],
                  [v[0],v[1],v[5]],[v[3],v[1],v[5]],
                  [v[3],v[4],v[5]],[v[0],v[4],v[5]]])
    faces = [[p[i] for i in f] for f in
             ([0,1,2,3],[4,5,6,7],[0,1,5,4],
              [2,3,7,6],[1,2,6,5],[0,3,7,4])]
    return faces

def add(ax, coords, weight):
    pc = Poly3DCollection(cube(coords),
                          facecolor='crimson' if weight<0 else 'limegreen',
                          edgecolor='k', alpha=.18 if weight<0 else .75,
                          linewidths=.6)
    ax.add_collection3d(pc)

def show(name, boxes):
    fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
    for c,w in boxes: add(ax, c, w)
    for fn in (ax.set_xlim, ax.set_ylim, ax.set_zlim): fn(-.35,.35)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(name, pad=18); plt.tight_layout(); plt.show()

# ---- build the 3-layer gate exactly like in the header ---------------
outer = [(( L-T,-L,-L,  L,  L,  L),-2),((-L,-L,-L,-L+T,L,L),-2),
         ((-L, L-T,-L,  L,  L,  L),-2),((-L,-L,-L, L,-L+T,L),-2),
         ((-L,-L, L-T,  L,  L,  L),-2),((-L,-L,-L, L, L,-L+T),-2)]
corn  = [((-L,-L,-L,-Cx,-Cx, L),-3),(( Cx,-L,-L, L,-Cx, L),-3),
         ((-L, Cx,-L,-Cx, L, L),-3),(( Cx, Cx,-L, L, L, L),-3),
         ((-L,-L, 0,-Cx,-Cx, L),-3),(( Cx,-L, 0, L,-Cx, L),-3),
         ((-L, Cx, 0,-Cx, L, L),-3),(( Cx, Cx, 0, L, L, L),-3)]
mid   = [(( L-2*T,-L, -L,  L-T,  L,  L),+1),((-L,   -L,-L,-L+2*T,L,L),+1),
         ((-L,  L-2*T,-L,  L,  L-T, L),+1),((-L,   -L,-L, L,-L+2*T,L),+1),
         ((-L,   -L, L-2*T, L,   L, L-T),+1),((-L,-L,-L, L,  L,-L+2*T),+1)]
core  = [((-0.09,-0.09,-0.09,0.09,0.09,0.09),-1)]

show("3-layer Sphere Gate  (−  +  −)", outer+corn+mid+core)
