 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %
 % Software License Agreement (BSD License)
 %
 % Copyright (c) 2012, Max-Planck-Gesellschaft
 % Copyright (c) 2012-2015, Inria
 % All rights reserved.
 %
 % Redistribution and use in source and binary forms, with or without
 % modification, are permitted provided that the following conditions
 % are met:
 %
 %  * Redistributions of source code must retain the above copyright
 %    notice, this list of conditions and the following disclaimer.
 %  * Redistributions in binary form must reproduce the above
 %    copyright notice, this list of conditions and the following
 %    disclaimer in the documentation and/or other materials provided
 %    with the distribution.
 %  * Neither the name of the copyright holder nor the names of its
 %    contributors may be used to endorse or promote products derived
 %    from this software without specific prior written permission.
 %
 % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 % "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 % LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 % FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 % COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 % INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 % BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 % LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 % CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 % LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 % ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 % POSSIBILITY OF SUCH DAMAGE.
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


lib_name = 'ros_msgs';
packages = {... 
@INSTALLED_MRB_PACKAGES@ ...
};

mdl_paths = {... 
@MDL_PATHS@ ...
};

src_paths = {... 
@SRC_PATHS@ ...
};

source_path = fileparts('@SRC_PATH@');
sfun_path = fileparts('@SFUN_PATH@');

if strcmp(sfun_path(end-numel('matlab_ros_bridge')+1:end), 'matlab_ros_bridge')
    sfun_path = sfun_path(1:end-numel('matlab_ros_bridge'));
end

disp('Creating main library file. This may take a while.');
new_system(lib_name,'Library');

fid = fopen( 'setup.m', 'wt' );

disp('Loading model...');
load_system(fullfile(source_path, 'matlab_ros_bridge', 'models', 'subsystem'));

for j = 1:numel(packages)
    disp(['Generating library for package ' packages{j}]);
    addpath(fullfile(fileparts(mdl_paths{j})));
    fprintf(fid, 'addpath(fullfile(''%s''));\n', fileparts(mdl_paths{j}));
    fprintf(fid, 'addpath(fullfile(''%s''));\n', src_paths{j});
    fprintf(fid, 'addpath(fullfile(''%s'', ''%s'', ''lib''));\n', sfun_path, packages{j});
    
    [~, mdl] = fileparts(mdl_paths{j});
    load_system(mdl);
    add_block( 'subsystem/Subsystem', [lib_name '/' mdl], 'Position', [0 100*j 200 100*j+80] )
    list = find_system(mdl, 'regexp', 'on');
    for k = 2:numel(list)
        add_block( list{k}, [lib_name '/' list{k}], 'Position', [0 100*(k-2) 200 100*(k-2)+80] );
    end
        
%     add_block( [packages{j} '/' packages{j}], [lib_name '/' packages{j}], 'Position', [0 100*j 200 100*j+80] )
end
save_system(lib_name);
close_system(lib_name);
