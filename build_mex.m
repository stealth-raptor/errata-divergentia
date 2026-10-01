function build_mex()
%BUILD_MEX  Compile SIMULATE_MEX, the C version of the closed-loop simulation.
%
%   BUILD_MEX()  builds simulate_mex.c in this folder with Octave's mkoctfile
%   or MATLAB's mex.  Afterwards SIMULATE_CLOSED_LOOP uses it automatically:
%   the same model and results as the .m loop (tools/check_mex.m), about 1000x
%   faster in Octave, which is what makes the FOPSO-GWO tuning practical.
%
%   Needs a C compiler: Octave on Windows ships one; on Linux install the
%   Octave development package (e.g. apt install octave-dev); on macOS the
%   Xcode command line tools.
%
%   See also SIMULATE_CLOSED_LOOP, TUNE_FOPID_HYBRID.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
if exist('OCTAVE_VERSION', 'builtin')
    mkoctfile('--mex', '-O3', 'simulate_mex.c');
else
    mex('-O', 'simulate_mex.c');
end
clear('simulate_mex');                 % make sure the new binary is picked up
fprintf('built simulate_mex\n');
end
