@echo off
set MATLAB=D:\Program Files\MATLAB\R2020a

call  "\\DESKTOP-TNPJABE\D$\Program Files\MATLAB\R2020a\bin\win64\checkMATLABRootForDriveMap.exe" "\\DESKTOP-TNPJABE\D$\Program Files\MATLAB\R2020a"  > mlEnv.txt
for /f %%a in (mlEnv.txt) do set "%%a"\n
"%MATLAB%\bin\win64\gmake" -f rtwshared.mk MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  GENERATE_ASAP2=0 TMW_EXTMODE_TESTING=0 OPTS="-DNRT"
