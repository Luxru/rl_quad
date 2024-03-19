from scipy.spatial import ConvexHull
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

rng = np.random.default_rng()
points =  rng.random((30, 3))
points_test =  rng.random((30, 3))+1
t = time.time_ns()
hull = ConvexHull(points)
at = time.time_ns()
print(f"cost:{at-t}ns")

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:,0], points[:,1],points[:,2], 'r--', lw=2)
ax.scatter(points_test[:,0], points_test[:,1],points_test[:,2], 'b--', lw=2)

for i in hull.simplices:
    ax.plot_trisurf(points[i,0],points[i,1],points[i,2],alpha=0.5)

for i,pt in enumerate(points):
    in_hull = True
    for eq in hull.equations:
        if(pt[0]*eq[0]+pt[1]*eq[1]+pt[2]*eq[2]+eq[3]>1e-5):
            in_hull = False
    print(f"i:{i},is_in:{in_hull}")

for i,pt in enumerate(points_test):
    in_hull = True
    for eq in hull.equations:
        if(pt[0]*eq[0]+pt[1]*eq[1]+pt[2]*eq[2]+eq[3]>1e-5):
            in_hull = False
    print(f"i:{i},is_in:{in_hull}")

plt.show()
