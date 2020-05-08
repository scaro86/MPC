% Receive license file from the FORCES PRO server to validate the 
% generated FORCES PRO solvers
%
%    FORCESGETLICENSEFILE() receives a license file from the default
%    FORCES PRO server and saves it to FORCES_PRO.license
%
%    FORCESGETLICENSEFILE(LICENSE_FILE_NAME) receives a license file 
%    from the default FORCES PRO server and saves it to the selected 
%    filename
%
%    FORCESGETLICENSEFILE(LICENSE_FILE_NAME, FORCESURL) receives a 
%    license file from the selected FORCES PRO server and saves it 
%    to the selected filename
%
%    FORCESGETLICENSEFILE(LICENSE_FILE_NAME, FORCESURL, USERID) 
%    receives a license file for the selected userid from the 
%    selected FORCES PRO server and saves it to the selected filename. 
%
%    FORCESGETLICENSEFILE(LICENSE_FILE_NAME, FORCESURL, USERID, DATABASE)  
%    receives a license file for the selected userid from the 
%    selected FORCES PRO server and saves it to the selected filename. 
%    The user performing the request will be checked in the selected 
%    database. Available values for database are 'default', 'portal', 
%    'old'. If not sure, use 'default'.
%
% This file is part of the FORCES PRO client software for Matlab.
% (c) embotech AG, 2013-2020, Zurich, Switzerland. All rights reserved.
