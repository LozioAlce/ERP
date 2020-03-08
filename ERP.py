import numpy as np
from numpy.linalg import pinv
from numpy.linalg import matrix_rank
from tabulate import tabulate

np.seterr(divide='ignore', invalid='ignore')

tau_cmd = np.array([3.0, 28.0, 0]) * 1e-2 #feasible
# tau_cmd = np.array([3.0, 10.0, 0]) * 1e-2
# tau_cmd = np.array([3.0, 38.0, 0]) * 1e-2 #infeasible
# tau_cmd = np.array([1.0, 1.0, 0]) * 1e-2

M = np.array([[23.8, -23.8, 123.0, -123.0, 41.8, -41.8, 3.6],
              [-698.0, -698.0, 99.4, 99.4, -55.2, -55.2, 0.0],
              [-30.9, 30.9, 0.0, 0.0, -17.4, 17.4, -56.2]])
M = M * 1e-5
[n_t, n_u] = M.shape

u_min = np.array([-14.0, -14.0, -8.0, -8.0, -30.0, -30.0, -30.0, ])
u_max = np.array([10.5, 10.5, 45.0, 45.0, 30.0, 30.0, 30.0])

u_min = u_min.reshape(n_u, 1)
u_max = u_max.reshape(n_u, 1)


tau_cmd = tau_cmd.reshape(n_t, 1)

tau_0 = np.zeros(shape=(n_t, 1))
u_0 = pinv(M) @ tau_0

tau_acc = tau_cmd - tau_0
N_eps_k = np.ones(shape=(n_u, 1))
M_eps_k = M

c_prev = 0
u_next = u_0
kk = 0

c_history = np.array(c_prev)
Solution = np.zeros(shape=(n_u, n_u))
while c_prev<1: # ERP modified
# while c_prev<1 and matrix_rank(M_eps_k)==n_t: #old implementation ERP original
    u_prev = u_next
    u_acc_k = pinv(M_eps_k) @ tau_acc

    d_i_k_Max = np.divide(u_max - u_prev, u_acc_k)
    d_i_k_Min = np.divide(u_min - u_prev, u_acc_k)

    idx_Max = np.where(np.squeeze(u_acc_k) > 0)
    idx_Min = np.where(np.squeeze(u_acc_k) < 0)
    idx_Max = np.squeeze(idx_Max)
    idx_Min = np.squeeze(idx_Min)
    d_i_k = np.empty_like(d_i_k_Max)

    d_i_k[idx_Max] = d_i_k_Max[idx_Max]
    d_i_k[idx_Min] = d_i_k_Min[idx_Min]

    index_N_eps_actuator = np.where(np.squeeze(N_eps_k) == 1)
    index_N_eps_actuator = np.squeeze(index_N_eps_actuator)

    d_i_k = d_i_k[index_N_eps_actuator]
    d_max = min(d_i_k)

if d_max > 1 - c_prev:
    d_next = 1 - c_prev
        d_next = d_max
    else:

    kk = kk + 1

    u_next = u_prev + d_next * u_acc_k
    c_prev = c_prev + d_next

    idx_min = np.where(np.squeeze(u_next) <= np.squeeze(u_min) + 2e-14)
    idx_max = np.where(np.squeeze(u_next) >= np.squeeze(u_max) - 2e-14)
    idx_max = np.squeeze(idx_max)
    idx_min = np.squeeze(idx_min)
    idx = np.append(idx_max, idx_min)
    idx_prev = np.where(np.squeeze(N_eps_k) == 0)
    idx_prev = np.squeeze(idx_prev)

    idx_to_Remove = np.setdiff1d(idx, idx_prev)
    if not (len(idx_to_Remove) == 0):
        idx_to_Remove = min(idx_to_Remove)
    else:
        pass

N_eps_k[idx_to_Remove] = 0
    M_eps_prev = M_eps_k
    index = np.where(np.squeeze(N_eps_k) == 0)
    index = np.squeeze(index)
    M_eps_k[:, index] = 0

    c_history = np.append(c_history, c_prev)
    Solution[:, kk - 1] = np.squeeze(u_next)

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
