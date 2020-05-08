function [x, status] = mpcCustomSolverCodeGen(H, f, A, b, x0)
%% Interface between linear MPC and FORCES QP Solver (internal)
%
% Author(s): Rong Chen, MathWorks Inc.
%
% Copyright (c) 2019, The MathWorks, Inc. 
%
% All rights reserved. 
%
% Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 
%
% 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
%
% 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
%
% 3. In all cases, the software is, and all modifications and derivatives of the software shall be, licensed to you solely for use in conjunction with MathWorks products and service offerings.  
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
%
%
%% Help
% mpcCustomSolverCodeGen allows the user to specify a custom (QP) solver
% written in Embedded MATLAB to be used by MPC controller in code generation.
%
% Workflow:
%   (1) Copy this template file to your work folder and rename it to
%       "mpcCustomSolverCodeGen.m".  The work folder must be on the path. 
%   (2) Modify the "mpcCustomSolverCodeGen.m" to use your solver.
%       Note that your Embedded MATLAB solver must use only fixed-size data.
%   (3) Set "mpcobj.Optimizer.CustomSolverCodeGen = true" to tell the MPC
%       controller to use the solver in code generation.
% To generate code:
%   In MATLAB, use "codegen" command with "mpcmoveCodeGeneration" (require MATLAB Coder) 
%   In Simulink, generate code with MPC and Adaptive MPC blocks (require Simuink Coder products)
%
% To use this solver for simulation in MATLAB and Simulink, you need to:
%   (1) Copy "mpcCustomSolver.txt" template file to your work folder and 
%       rename it to "mpcCustomSolver.m".  The work folder must be on the path. 
%   (2) Modify the "mpcCustomSolver.m" to use your solver.
%   (3) Set "mpcobj.Optimizer.CustomSolver = true" to tell the MPC
%       controller to use the solver in simulation.
%
% The MPC QP problem is defined as follows: 
%
%       min J(x) = 0.5*x'*H*x + f'*x, s.t. A*x >= b.
%
% Inputs (provided by MPC controller at run-time):
%       H: a n-by-n Hessian matrix, which is symmetric and positive definite.
%       f: a n-by-1 column vector.
%       A: a m-by-n matrix of inequality constraint coefficients.
%       b: a m-by-1 vector of the right-hand side of inequality constraints.
%      x0: a n-by-1 vector of the initial guess of the optimal solution.
%
% Outputs (sent back to MPC controller at run-time):
%       x: must be a n-by-1 vector of optimal solution. 
%  status: must be an integer of:
%           positive value: number of iterations used in computation
%                        0: maximum number of iterations reached
%                       -1: QP is infeasible
%                       -2: Failed to find a solution due to other reasons
% Note that:
%   (1) When solver fails to find an optimal solution (status<=0), "x" 
%       still needs to be returned.
%   (2) To use sub-optimal QP solution in MPC, return the sub-optimal "x" 
%       with "status = 0".  In addition, you need to set
%       "mpcobj.Optimizer.UseSuboptimalSolution = true" in MPC controller. 
%
