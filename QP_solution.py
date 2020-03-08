from gekko import GEKKO
import numpy as np
from tabulate import tabulate

tau_cmd = np.array([3.0, 38.0, 0]) * 1e-2
# tau_cmd = np.array([3.0, 10.0, 0]) * 1e-2

M = np.array([[23.8, -23.8, 123.0, -123.0, 41.8, -41.8, 3.6],
              [-698.0, -698.0, 99.4, 99.4, -55.2, -55.2, 0.0],
              [-30.9, 30.9, 0.0, 0.0, -17.4, 17.4, -56.2]])
M = M * 1e-5
[n_t, n_u] = M.shape

u_min = np.array([-14.0, -14.0, -8.0, -8.0, -30.0, -30.0, -30.0 ])
u_max = np.array([10.5, 10.5, 45.0, 45.0, 30.0, 30.0, 30.0])

u_min = u_min.reshape(n_u, 1)
u_max = u_max.reshape(n_u, 1)

u_min = np.squeeze(u_min)
u_max = np.squeeze(u_max)
model = GEKKO()

u1 = model.Var(value = 0.0, lb = u_min[0], ub = u_max[0])
u2 = model.Var(value = 0.0, lb = u_min[1], ub = u_max[1])
u3 = model.Var(value = 0.0, lb = u_min[2], ub = u_max[2])
u4 = model.Var(value = 0.0, lb = u_min[3], ub = u_max[3])
u5 = model.Var(value = 0.0, lb = u_min[4], ub = u_max[4])
u6 = model.Var(value = 0.0, lb = u_min[5], ub = u_max[5])
u7 = model.Var(value = 0.0, lb = u_min[6], ub = u_max[6])

u = [u1,u2,u3,u4,u5,u6,u7]
u = np.array(u)
u = u.reshape(n_u,1)
u = np.squeeze(u)
tau_cmd = np.squeeze(tau_cmd)
totEquations = M@u
objSoftCostraints = tau_cmd[0] * 0

for ii in range(0,len(tau_cmd)):
    SoftConstraints = 1e13*(totEquations[ii]-tau_cmd[ii])**2
    objSoftCostraints = objSoftCostraints + SoftConstraints

myObj = sum(np.square(u))
myObj = myObj + objSoftCostraints
model.Obj(myObj)
model.solve(disp=False)

u_next = np.array([])
for element in u:
    u_next=np.append(u_next,element.value)

u_min = np.squeeze(u_min)
u_max = np.squeeze(u_max)
u_next = np.squeeze(u_next)

u_min = np.transpose(u_min)
u_next = np.transpose(u_next)
u_max = np.transpose(u_max)

data = [u_min,u_next,u_max]
data = np.transpose(data)

headers = ["u_min", "u_next", "u_max"]
print(tabulate(data,headers,tablefmt="psql", floatfmt=".7f"))
