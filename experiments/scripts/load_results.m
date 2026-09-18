function load_results()
%LOAD_RESULTS  List every logged .mat file under experiments/ and its variables.
%
%   Prints the name, size and class of each variable stored in the result
%   files, so a file can be loaded knowing what it contains.
%
%   Usage:
%       cd experiments/scripts
%       load_results
%
%   Then load a file directly, e.g.:
%       load('../super_twisting/xcircle.mat');
%
%   Sample time is Ts = 0.02 s; sample index * 0.02 gives seconds.

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');

folders = {'pd', 'sliding_mode', 'super_twisting'};

for k = 1:numel(folders)
    d = dir(fullfile(root, folders{k}, '*.mat'));
    if isempty(d)
        continue
    end

    fprintf('\n=== %s ===\n', folders{k});

    for i = 1:numel(d)
        f = fullfile(d(i).folder, d(i).name);
        info = whos('-file', f);

        names = cell(1, numel(info));
        for j = 1:numel(info)
            names{j} = sprintf('%s (%s %s)', ...
                info(j).name, ...
                mat2str(info(j).size), ...
                info(j).class);
        end

        fprintf('  %-16s %s\n', d(i).name, strjoin(names, ', '));
    end
end

fprintf('\n');

end
