%% Example taken from Fast Exact Redistributed Pseudoinverse Method for Linear Actuation Systems - Johannes Stephan and Walter Fichter
clear
M= [  23.8  -23.8 123.0 -123.0  41.8 -41.8   3.6
    -698.0 -698.0  99.4   99.4 -55.2 -55.2   0.0
     -30.9   30.9   0.0    0.0 -17.4  17.4 -56.2];
M = M.*1e-5;
[n_t, n_u] = size(M);

u_min =[-14.0  -14.0 -8.0 -8.0 -30.0 -30.0 -30.0]';
u_max =[ 10.5   10.5 45.0 45.0  30.0  30.0  30.0]';

% tau_cmd = [3.0 28.0 0]' .*1e-2; %% feasible Fig 6
tau_cmd = [3.0 38.0 0]' .*1e-2; %% feasible Fig 8

%
tau_0 = zeros(n_t,1);
u_0   = pinv(M)*tau_0;      %%Eq 11.1
tau_acc = tau_cmd -tau_0;   %%from Eq 6
N_eps_k = ones(n_u,1);      %%From Eq 13.1    
M_eps_k = M;                %%From Eq 14

c_prev = 0;                 %% from Algorithm section initialization
u_next = u_0;               %% for iterative formulation see Eq 11
kk =1;
tic

while not ((c_prev == 1) || rank(M_eps_k) < n_t )   %% Eq 19

    u_prev = u_next;
    u_acc_k = pinv(M_eps_k)*tau_acc;            %% Eq 11.2
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
