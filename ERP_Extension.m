%% Example taken from Constrained Dynamic Control Allocation in the Presence of Singularity and Infeasible Solutions - Section IV
clear
M= [ 1 1 1 1 1 
     1 1 1 0 0
     1 0 0 0 0];
[n_t, n_u] = size(M);

u_min =[-1   0.2 -1 -0.4 -0.2]';
u_max =[ 1.2 1.0  0  0.6  0.1]';

tau_cmd = [1.4 1 -1]';

tau_0 = zeros(n_t,1);
u_0   = pinv(M)*tau_0;      %%Eq 11.1
tau_acc = tau_cmd -tau_0;   %%from Eq 6
N_eps_k = ones(n_u,1);      %%From Eq 13.1    
M_eps_k = M;                %%From Eq 14

c_prev = 0;                 %% from Algorithm section initialization
u_next = u_0;               %% for iterative formulation see Eq 11
kk =1;
tic

while (c_prev < 1)
% while ((c_prev < 1) && rank(M_eps_k) == n_t ) %% equivalent to while not ((c_prev == 1) || rank(M_eps_k) < n_t ) in ERP.m 
    u_prev = u_next;
    u_acc_k = pinv(M_eps_k)*tau_acc;            %% Eq_11.2
%% Start Computing d_max    
        d_i_k_Max = (u_max - u_prev)./u_acc_k;  %% Eq 18
        d_i_k_Min = (u_min - u_prev)./u_acc_k;  %% Eq 18

        idx_Max = find(u_acc_k>0);
        idx_Min = find(u_acc_k<0);

        d_i_k = [];
        d_i_k(idx_Max,1) = d_i_k_Max(idx_Max);
        d_i_k(idx_Min,1) = d_i_k_Min(idx_Min);

        index_N_eps_actuator = find(N_eps_k==1);
        d_i_k = d_i_k(index_N_eps_actuator);
        d_max = min(d_i_k);                     %% Eq 18

        if d_max > 1-c_prev                     %% Eq 16
            d_next = 1-c_prev;
        else
            d_next = d_max;
        end
	d_next;

    kk = kk+1;
    
    u_next = u_prev + d_next*u_acc_k ;          %% Eq 11.3
    c_prev = c_prev + d_next;                   %% Eq 12
    
%% Start Computing N_eps_k    

        idx_max = find(u_next == u_max);        %% Eq 13
        idx_min = find(u_next == u_min);

        idx = vertcat(idx_max,idx_min);

        idx_prev = find(N_eps_k==0);
        idx_to_Remove = setdiff(idx,idx_prev);
        idx_to_Remove = min(idx_to_Remove );

	N_eps_k(idx_to_Remove) = 0;

    M_eps_k(:,N_eps_k==0) = 0;                  %% Eq 14
    
    Solution(:,kk-1) = u_next;
end

numIteration = kk;
finalTime = toc;


fprintf('\n number of Iterations = %5d \n',numIteration)
fprintf('\n Time for solution = %5.3f [ms] \n',finalTime*1000)

fprintf('\nSolution is =[')
fprintf(' %10.5f \n \t      ', u_next(1:end-1)')
fprintf(' %10.5f]\n',u_next(end))
%% Solution(:,2) corresponds to the original ERP method - simply modify the uncomment the second *while* and comment the first while
fprintf('\n ERP original ||M*u_next-tau_cmd|| = %.5f \n',norm(M*Solution(:,2)-tau_cmd))
fprintf('\n Update1      ||M*u_next-tau_cmd|| = %.5f \n',norm(M*u_next-tau_cmd))
