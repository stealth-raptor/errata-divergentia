function build_mex()
%BUILD_MEX  Compile SIMULATE_MEX, the C version of the closed-loop simulation.
%
%   BUILD_MEX()  builds simulate_mex.c in this folder with MATLAB's MEX or
%   Octave's mkoctfile.  Afterwards SIMULATE_CLOSED_LOOP uses it automatically
%   (identical results, roughly 100x faster; needed for tuning under Octave).
%   A C compiler is required (Xcode command line tools on macOS).
%
%   See also SIMULATE_CLOSED_LOOP.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
if exist('OCTAVE_VERSION', 'builtin')
    mkoctfile('--mex', '-O3', 'simulate_mex.c');
else
    mex('-O', 'simulate_mex.c');
end
fprintf('built simulate_mex\n');
end
