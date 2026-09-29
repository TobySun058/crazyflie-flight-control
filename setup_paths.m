function setup_paths()
%SETUP_PATHS Add project MATLAB source directories to the active path.
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'matlab')));
fprintf('Crazyflie project paths added from %s\n', root);
end
