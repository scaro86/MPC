%% Interface between MPC Toolbox and FORCES PRO Solver
%
%  This function generates customized FORCES QP solver from @mpc object.
%
%   [coredata, statedata, onlinedata] = mpcToForces(mpcobj, options) 
%
%   Inputs:
%
%       mpcobj:     an MPC controller object created by the "mpc" command
%                   and designed with the MPC Toolbox from MathWorks
%
%       options:    specify solver generation settings created by the
%                   "mpcToForcesOptions" command.  You can choose either a
%                   "sparse" QP formulation or a "dense" QP formulation.  
%
%   Outputs:
%
%       coredata:   a structure of MPC settings (constant)
%
%       statedata:  a structure of plant states, disturbance model states, noise model states and the last optimal MV
%
%       onlinedata: a structure of signals (references, measured output, external MV, online constraints, online weights, etc.)
%
%  The following LTI MPC features are supported when using the FORCES QP
%  solver:  
%
%       Plant model (continuous time or discrete time)
%       Block control moves 
%       Measured disturbances (MD)
%       Unmeasured disturbances (UD)
%       Output/MV/MVRate/ECR weights (uniform or time-varying)
%       Output/MV/MVRate bounds (uniform or time-varying)
%       Soft constraints           
%       Signal previewing on output references and measured disturbances
%       Signal previewing on MV references (only available using sparse QP)
%       Scale factors
%       Nominal values 
%       Online update of constraints and weights.
%       Build-in and custom state estimation
%
%  Unsupported MPC features include:
%       Adaptive MPC and LTV MPC where plant model changes online
%       Single precision
%       Mixed input and output constraints
%       Suboptimal solution when using the sparse QP formulation
%       Alternative cost function with off-diagonal weights
%
% See also mpcmoveForces, mpcToForcesOptions.
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
