% Dumps a FORCES PRO problem formulation into a file to allow to exactly 
% reproduce the issues with the exported code.
%
%   ForcesDumpFormulation(FORMULATION, OPTIONS, LABEL) stores the problem
%   formulation into a mat file with standardized naming. It returns a 
%   tag string that should be passed to ForcesDumpProblem when dumping
%   actual problem instances.
%
%       FORMULATION:   formulation struct as returned by FORCES_NLP
%       OPTIONS:       codeoptions as provided to FORCES_NLP
%       OUTPUTS:       outputs as provided to FORCES_NLP
%       LABEL:         optional, a custom label used inside the filename
%
% See also FORCES_NLP, ForcesDumpProblem
%   
% This file is part of the FORCES PRO client software for Matlab.
% (c) embotech AG, 2013-2020, Zurich, Switzerland. All rights reserved.
