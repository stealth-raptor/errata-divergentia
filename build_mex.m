function build_mex()
%BUILD_MEX  Compile SIMULATE_MEX, the C version of the closed-loop simulation.
%
%   BUILD_MEX()  builds simulate_mex.c in this folder with MATLAB's MEX.
%   Afterwards SIMULATE_CLOSED_LOOP uses it automatically: the same model
%   and the same results as the .m loop (tools/check_mex.m), several hundred
%   times faster, which is what makes the optimiser comparisons practical.
%
%   Needs a C compiler that MATLAB supports (run "mex -setup C" once):
%   on Windows MinGW-w64 (free add-on) or Microsoft Visual C++, on Linux gcc,
%   on macOS the Xcode command line tools.  The source is plain C with
%   fixed-size arrays only, so every one of them compiles it.
%
%   The binary uses the separate-complex MEX API (mxGetPr), so on MATLAB
%   R2018a and newer it is built with -R2017b; older releases use it anyway.
%
%   See also SIMULATE_CLOSED_LOOP, TUNE_FOPID_HYBRID.

here = fileparts(mfilename('fullpath'));
src = fullfile(here, 'simulate_mex.c');
args = {'-O', '-outdir', here, src};
v = sscanf(version, '%d.%d');           % e.g. '9.14.0.2206163 (R2023a)' -> [9; 14]
if v(1) > 9 || (v(1) == 9 && v(2) >= 4) % R2018a (9.4) and newer: choose the API
    args = [{'-R2017b'}, args];
end
mex(args{:});
clear('simulate_mex');                  % make sure the new binary is picked up
fprintf('built simulate_mex (%s)\n', mexext);
end
