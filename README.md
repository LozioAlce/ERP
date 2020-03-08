# ERP
An implementation of the ERP method- Exact Redistributed Pseudoinverse Method 

This is a code implementation based on the paper of Johannes Stephan and Walter Fichter:<br/>
Title: "Fast Exact Redistributed Pseudoinverse Method for Linear Actuation Systems" <br/>
DOI: 10.1109/TCST.2017.2765622<br/>

It solves the problem of control allocation when the request τ may lead to some of the control inputs to be saturated.<br/>
It clips the saturated actuator to its maximum and redistributes the remain part of the request among the other actuators.

# Update 1
An extension is introduced:<br/>
In the original implementation of ERP method the pseudo-inverse is computed using Eq 15 (only if the reduced control effectiveness matrix is not rank deficient ).<br/>
The **SVD** decomposition exists even if the reduced control effectiveness matrix is rank deficient (*M_eps_k* in the code).<br/>
<br/>
Using SVD decomposition (used by MATLAB pinv Moore-Penrose pseudoinverse) can increase the iterations and reduce the norm between the request τ and the possible deliverable τ.<br/>
The only required modification is that the while condition has to change into: <br/> while(c_next < 1)<br/>

This result overlaps with the one from the paper of David Buzorgnia and Ali Khaki-Sedigh:<br/>
Title: "Constrained Dynamic Control Allocation in the Presence of Singularity and Infeasible Solutions"<br/>
https://arxiv.org/abs/1607.05209<br/>

The same solution for the example at section IV is found with this simple modification.

# Update 2
Added a Quadratic Programming example to compare the solutions.<br/> 
QP solution is based on the python package GEKKO.<br/> <br/> 
To install GEKKO:<br/> 
pip3 install gekko<br/> <br/> 
To install tabulate python package (for printing final results):<br/>
pip3 install tabulate<br/> 
