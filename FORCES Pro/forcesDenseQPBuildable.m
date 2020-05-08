classdef forcesDenseQPBuildable < coder.ExternalDependency
%% Interface between lineat MPC and FORCES QP Solver (internal)
%
%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019 The MathWorks, Inc.
%
%   All rights reserved. 
%
%   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 
%
%   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
%
%   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
%
%   3. In all cases, the software is, and all modifications and derivatives of the software shall be, licensed to you solely for use in conjunction with MathWorks products and service offerings.  
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'forcesDenseQPBuildable';
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end        
        
        function updateBuildInfo(buildInfo, ~)
            % forces path
            forcespath = fileparts(which('FORCESversion'));
            % solver name
            solvername = 'customForcesDenseQP';
            % solver header
            headerPath = ['$(START_DIR)' filesep solvername filesep 'include'];
            buildInfo.addIncludePaths(headerPath);
            % solver library
            libPriority = '';
            libPreCompiled = true;
            libLinkOnly = true;
            if( ismac || isunix )
                libName = ['lib' solvername '.a'];
            else
                libName = [solvername '_static.lib'];  
            end
            libPath = ['$(START_DIR)' filesep solvername filesep 'lib'];
            buildInfo.addLinkObjects(libName, libPath, libPriority, libPreCompiled, libLinkOnly);
            % additional standard library
            if ispc
                libPathExtra = [forcespath filesep 'libs_Intel' filesep 'win64'];
                buildInfo.addLinkObjects('libmmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libirc.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('svml_dispmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libdecimal.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
            end
            buildInfo.addLinkObjects('iphlpapi.lib', ['$(MATLAB_ROOT)' filesep 'sys' filesep 'lcc64' filesep 'lcc64' filesep 'lib64'], libPriority, libPreCompiled, libLinkOnly);
        end
        
        function [x, status] = forcesDenseQP(H, f, A, b, x0)
            FORCES_SolverName = 'customForcesDenseQP';
            headerName = [FORCES_SolverName '.h'];
            coder.cinclude(headerName);
            % inputs based on C interface defined in the header file
            params = struct('H',H,'f',f,'A',-A,'b',-b);
            coder.cstructname(params,[FORCES_SolverName '_params'],'extern','HeaderFile',headerName);
            output = struct('DecisionVariables', zeros(length(f),1));
            coder.cstructname(output,[FORCES_SolverName '_output'],'extern','HeaderFile',headerName);
            SolverInfo = struct('it', int32(0), 'it2opt', int32(0), 'res_eq', 0, 'res_ineq', 0, 'pobj', 0, 'dobj', 0, 'dgap', 0, 'rdgap', 0, 'mu', 0, 'mu_aff', 0, 'sigma', 0, 'lsit_aff', int32(0), 'lsit_cc', int32(0), 'step_aff', 0, 'step_cc', 0, 'solvetime', 0);
            coder.cstructname(SolverInfo,[FORCES_SolverName '_info'],'extern','HeaderFile',headerName);
            FILE = coder.opaque('FILE *','NULL','HeaderFile',headerName);
            exitflag = int32(0); %#ok<NASGU>
            % generate code with solver DLL/LIB
            exitflag = coder.ceval([FORCES_SolverName '_solve'],coder.ref(params),coder.ref(output),coder.ref(SolverInfo),FILE);
            % get solution
            x = output.DecisionVariables;
            % Converts the "flag" output to "status" required by the MPC controller.
            switch double(exitflag)
                case 1
                    status = double(SolverInfo.it);
                case 0
                    status = 0;
                otherwise
                    status = -2;
            end
            % Always return a non-empty x of the correct size.  When the solver fails,
            % one convenient solution is to set x to the initial guess.
            if status <= 0
                x = x0;
            end
        end
    end
end
