function [x, status] = mpcCustomSolver(H, f, A, b, x0)
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
%% Help
% mpcCustomSolver allows user to specify a custom quadratic programming
% (QP) solver to solve the QP problem formulated by MPC controller.  When
% the "mpcobj.Optimizer.CustomSolver" property is set true, instead of
% using the built-in QP solver, MPC controller will now use the customer QP
% solver defined in this function for simulations in MATLAB and Simulink.
%
% The MPC QP problem is defined as follows:
%   Find an optimal solution, x, that minimizes the quadratic objective
%   function, J = 0.5*x'*H*x + f'*x, subject to linear inequality
%   constraints, A*x >= b.
%
% Inputs (provided by MPC controller at run-time):
%       H: a n-by-n Hessian matrix, which is symmetric and positive definite.
%       f: a n-by-1 column vector.
%       A: a m-by-n matrix of inequality constraint coefficients.
%       b: a m-by-1 vector of the right-hand side of inequality constraints.
%      x0: a n-by-1 vector of the initial guess of the optimal solution.
%
% Outputs (fed back to MPC controller at run-time):
%       x: must be a n-by-1 vector of optimal solution. 
%  status: must be an finite integer of:
%           positive value: number of iterations used in computation
%                        0: maximum number of iterations reached
%                       -1: QP is infeasible
%                       -2: Failed to find a solution due to other reasons
% Note that even if solver failed to find an optimal solution, "x" must be
% returned as a n-by-1 vector (i.e. set it to the initial guess x0)
%
