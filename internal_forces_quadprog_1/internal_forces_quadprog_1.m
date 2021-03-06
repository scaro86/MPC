% internal_forces_quadprog_1 - a fast solver generated by FORCES PRO v3.0.0
%
%   OUTPUT = internal_forces_quadprog_1(PARAMS) solves a multistage problem
%   subject to the parameters supplied in the following struct:
%       PARAMS.p_1 - column vector of length 6
%
%   OUTPUT returns the values of the last iteration of the solver where
%       OUTPUT.o_1 - column vector of size 1
%       OUTPUT.o_2 - column vector of size 1
%
%   [OUTPUT, EXITFLAG] = internal_forces_quadprog_1(PARAMS) returns additionally
%   the integer EXITFLAG indicating the state of the solution with 
%       1 - OPTIMAL solution has been found (subject to desired accuracy)
%       0 - Timeout - maximum number of iterations reached
%      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
%    -100 - License error
%
%   [OUTPUT, EXITFLAG, INFO] = internal_forces_quadprog_1(PARAMS) returns 
%   additional information about the last iterate:
%       INFO.it        - number of iterations that lead to this result
%       INFO.res_eq    - max. equality constraint residual
%       INFO.res_ineq  - max. inequality constraint residual
%       INFO.pobj      - primal objective
%       INFO.dobj      - dual objective
%       INFO.dgap      - duality gap := pobj - dobj
%       INFO.rdgap     - relative duality gap := |dgap / pobj|
%       INFO.mu        - duality measure
%       INFO.sigma     - centering parameter
%       INFO.lsit_aff  - iterations of affine line search
%       INFO.lsit_cc   - iterations of line search (combined direction)
%       INFO.step_aff  - step size (affine direction)
%       INFO.step_cc   - step size (centering direction)
%       INFO.solvetime - Time needed for solve (wall clock time)
%
% See also COPYING
