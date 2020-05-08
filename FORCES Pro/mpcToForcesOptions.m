%% Interface between linear MPC and FORCES QP Solver
%
%  This function allows you to choose Dense QP or Sparse QP formulation for
%  custom solver and specify corresponding MPC features and FORCES QP
%  solver settings.
%
%  options = mpcToForcesOptions() or options = mpcToForcesOptions('sparse')
%  generate a Sparse QP problem where manipulated variables (MVs), outputs
%  (OVs) and states are decision variables.  Use Sparse QP for large MPC
%  problem with long horizons and large amount of constraints.  Sparse QP
%  also allows you to use long horizons even if the plant is unstable.
%
%  options = mpcToForcesOptions('dense') generates a Dense QP problem
%  where only manipulated variables (MVs) are used as decision variables.
%  Use Dense QP for small MPC problems with short horizons and small amount
%  of constraints. 
%
% See also mpcToForces, mpcmoveForces.
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

