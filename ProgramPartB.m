%{ 
    Author:     
        Neco Kriel
    Unit:
        EGH446 - Autonomous Control Project Part B
    Purpose:
        Initialise the workspace for running a ground vehicle simulation. This script 
        calculates a solution path between NUM_WPs number of waypoints using an A* algoirthm.
    Sources:
        E. S. Ueland, R. Skjetne, A. R. Dahl, (2017) Marine Autonomous Exploration Using a Lidar and SLAM
%}

%% Prepare the Workspace
set(0, 'DefaultFigureWindowStyle', 'docked') % undo by swapping 'docked' with 'normal'
clc, clear, close all
format shortg
format compact

%% Startup the Simulink Model
% disp("Loading Simulink Model...")
% % close the vehicle model if it is already open
% close_system('sl_groundvehicleDynamics',0)
% % add the relevant directories to the active directories
% homedir = pwd;
% addpath( genpath(strcat(homedir,[filesep,'rvctools'])))
% addpath( genpath(strcat(homedir,[filesep, 'slprj'])))
% addpath( genpath(strcat(homedir,[filesep,'matfiles'])))
% % startup the RVC model
% cd rvctools
% startup_rvc
% % open the simulation toolbox
% cd MRTB;
% startMobileRoboticsSimulationToolbox;
% % startup the vehicle's model
% cd ..
% open_system('sl_groundvehicleDynamics'); % differential robot
% % reset the working directory
% cd(homedir)

%% User-defined Parameters that affect the simulation
NUM_WPs = 5;

%% Load Data
disp(" ")
disp('Loading Data...')
% important file names
file_wall_locations  = 'logical_occupancy_map.mat';
file_map_information = 'complexMap.mat';
file_obstacles       = 'obstacles.mat';
% check that the files exist
CheckFileExists(file_wall_locations)
CheckFileExists(file_map_information)
CheckFileExists(file_obstacles)
% read wall locationsn (matrix dimensions)
struct_wall_locations = load(file_wall_locations);
map_walls = flip(logical(struct_wall_locations.map_logical_values), 1); % orientate the map
% read map dimension information (world scale)
struct_map_info = load(file_map_information);
struct_map_info = struct_map_info.map;
% map resolution (i.e. number of matrix cells per meter)
MAP_RES = struct_map_info.Resolution;
% save matrix dimensions
MAP_DIM_ROWS = struct_map_info.GridSize(1);
MAP_DIM_COLS = struct_map_info.GridSize(2);
% save real world scale
MAP_WIDTH = struct_map_info.XWorldLimits(2);
MAP_HEIGHT = struct_map_info.YWorldLimits(2);
% initialise the vehicle's starting position (relative to the world scale)
MAP_START_ROW = 2;
MAP_START_COL = 2;
% save the vehicle's starting position (in world scale)
START_ROW = MAP_START_ROW;
START_COL = MAP_START_COL;
% read the obstacles' locations (world scale)
struct_obstacles = load(file_obstacles);
struct_obstacles = struct_obstacles.obstacles;
disp("The obstacles are located at (x, y):")
disp([struct_obstacles(:, 1), struct_obstacles(:, 2)]) % print in world scale

%% Visualise Walls (Map)
% visualise map
figure
imagesc(int8(map_walls))
colormap(flip(gray(256)))
title("Wall Locations (matrix dimensions)", "FontSize", 17)
% flip matrix (around x-axis) to restore normal view
set(gca,'YDir','normal')
% label plot
xlabel("matrix columns", "FontSize", 15)
ylabel("matrix rows", "FontSize", 15)
% scale plot
axis tight
axis equal
% add a buffer around the walls
buff_walls_mat = map_walls;
buff_walls_mat(1:end-1, :) = buff_walls_mat(1:end-1, :) + map_walls(2:end, :);   % pad north
buff_walls_mat(2:end, :)   = buff_walls_mat(2:end, :)   + map_walls(1:end-1, :); % pad south
buff_walls_mat(:, 1:end-1) = buff_walls_mat(:, 1:end-1) + map_walls(:, 2:end);   % pad west
buff_walls_mat(:, 2:end)   = buff_walls_mat(:, 2:end)   + map_walls(:, 1:end-1); % pad east
buff_walls_mat(buff_walls_mat > 0) = 1;
% visualise the buffered map
figure
imagesc(int8(buff_walls_mat))
colormap(flip(gray(256)))
title("Buffered Walls (matrix dimensions)", "FontSize", 17)
% flip matrix (around x-axis) to restore normal view
set(gca,'YDir','normal')
% label plot
xlabel("matrix columns", "FontSize", 15)
ylabel("matrix rows", "FontSize", 15)
% scale plot
axis tight
axis equal

%% Generate Waypoints
disp(" ")
disp("Generating Waypoints...")
% generate random waypoints that don't overlap with eachother or maze-walls (matrix dimensions)
[WP_rows, WP_cols] = GenerateWaypoints(NUM_WPs, MAP_DIM_ROWS, MAP_DIM_COLS, ...
                            buff_walls_mat, MAP_RES*struct_obstacles);
disp("The waypoints are located at (x, y):")
disp([WP_cols/MAP_RES, WP_rows/MAP_RES]) % print the WP locations in world scale

%% Visualise the Scene
figure, hold on
imagesc(int8(map_walls))
colormap(flip(gray(256)))
% vehicle starting position
plot(MAP_RES*START_COL, MAP_RES*START_ROW, 'bs', 'MarkerSize', 15)
plot(MAP_RES*START_COL, MAP_RES*START_ROW, 'bs', 'MarkerSize', 20)
% waypoint locations
plot(WP_cols, WP_rows, 'bo', 'MarkerSize', 15)
plot(WP_cols, WP_rows, 'bo', 'MarkerSize', 20)
% plot the obstacles
plot(MAP_RES*struct_obstacles(:, 1), MAP_RES*struct_obstacles(:, 2), 'rx', 'MarkerSize', 20)
% label plot
title("Solution Paths Between Waypoints (world scale)", "FontSize", 17)
xlabel("x [meters]", "FontSize", 15)
ylabel("y [meters]", "FontSize", 15)
axis on
set(gca, 'XTick', round(linspace(0, MAP_DIM_COLS, 6)))
set(gca, 'YTick', round(linspace(0, MAP_DIM_ROWS, 10)))
set(gca, 'XTickLabel', round(linspace(0, MAP_WIDTH, 6)))
set(gca, 'YTickLabel', round(linspace(0, MAP_HEIGHT, 10)))
% scale plot
axis tight
axis equal

%% Calculate the Solution Path using A*
disp(" ")
disp("Calculating Optimal Path...")
% calculate the solution path (in matrix dimensions)
IWP_coords = AStarEntry(buff_walls_mat, WP_rows, WP_cols, MAP_RES*START_ROW, MAP_RES*START_COL);
% change the default colours
colororder(parula(2*(NUM_WPs+1)))

%% Scale Solution Path to Real World Scale
disp(" ")
disp("Finished.")
% scale the solution path (IWP) and WP locations to world scale
IWP_coords = IWP_coords / MAP_RES;
WP_coords = [WP_cols, WP_rows] / MAP_RES;
% add a flag to each IWP to indicate if it is a WP
IWP_WP_flag = zeros(size(IWP_coords,1), 1);
for i = 1:size(IWP_coords,1)
    IWP_WP_flag(i) = any( (WP_coords(:,1) == IWP_coords(i,1)) &...
                          (WP_coords(:,2) == IWP_coords(i,2)) );
end
IWP_mat = [IWP_coords, IWP_WP_flag];
disp('Solution path is stored in: IWP_mat')

%% Solution Plot in World Scale
figure, hold on
plot(START_COL, START_ROW, 'ks', 'MarkerSize', 20)
plot(START_COL, START_ROW, 'ks', 'MarkerSize', 15)
plot(IWP_coords(:,1), IWP_coords(:,2), 'b-', 'LineWidth', 3)
plot(WP_coords(:,1), WP_coords(:,2), 'ko', 'MarkerSize', 20)
plot(WP_coords(:,1), WP_coords(:,2), 'ko', 'MarkerSize', 15)
plot(struct_obstacles(:,1), struct_obstacles(:,2), 'rx', 'MarkerSize', 20)
% label plot
title("Solution Path", "FontSize", 17)
xlabel("x [meters]", "FontSize", 15)
ylabel("y [meters]", "FontSize", 15)
% scale plot
axis equal
axis([0, MAP_WIDTH, 0, MAP_HEIGHT])

%% Functions
function CheckFileExists(file_name)
% Check to make sure that the file exists in the current folder, otherwise throw an error
    if (~isfile(file_name))
        uiwait(warndlg(sprintf("Error: the file '%s' doesn't exist.", file_name)));
    end
end

function [WP_rows, WP_cols] = GenerateWaypoints(num_points, MAP_DIM_ROWS, MAP_DIM_COLS, ...
                                    map_walls, struct_obstacles)
    % find indices for the walls
    [wp_index_rows, wp_index_cols] = find(map_walls == 1);
    % initialise waypoint arrays
    WP_rows = zeros(num_points, 1);
    WP_cols = zeros(num_points, 1);
    for i = 1:num_points
        bool_valid_point = false;
        while (~bool_valid_point)
            % generate random point
            WP_row = randi(MAP_DIM_ROWS);
            WP_col = randi(MAP_DIM_COLS);
            % check that the point is not overlapping with any walls
            overlap_walls_rows = find( (wp_index_rows-WP_row) == 0 );
            overlap_walls_cols = find( (wp_index_cols-WP_col) == 0 );
            % check that the point is not overlapping with any previous points
            overlap_points_rows = find( (WP_rows-WP_row) == 0 );
            overlap_points_cols = find( (WP_cols-WP_col) == 0 );
            % check that the point is not overlapping with any obstacles
            % find the location of all obstacles within a radius of the point
            overlap_obst_rows = find( abs(struct_obstacles(:,2)-WP_row) <= 1 );
            overlap_obst_cols = find( abs(struct_obstacles(:,1)-WP_col) <= 1 );
            % evaluate check
            if (isempty(intersect(overlap_walls_rows,  overlap_walls_cols)) && ...
                isempty(intersect(overlap_points_rows, overlap_points_cols)) && ...
                isempty(intersect(overlap_obst_rows,   overlap_obst_cols)))
                bool_valid_point = true;
            end
        end
        % save the waypoint
        WP_rows(i) = WP_row;
        WP_cols(i) = WP_col;
    end
end

%{ 
    A* Entry Function
    The entry point to the A* algorithm
%}
function total_path = AStarEntry(buff_walls_mat, WP_rows, WP_cols, START_ROW, START_COL)
    % time how long it takes to find the solution path
    tic
    % initialise final solution path
    total_path = [];
    while (~isempty(WP_rows))
        % make the closest waypoint the next target
        [~, WP_next_index] = min(sqrt((WP_rows-START_ROW).^2 + (WP_cols-START_COL).^2));
        % find a path to the target waypoint
        path_pos = AstarSearch(buff_walls_mat, ...
                               WP_rows(WP_next_index), WP_cols(WP_next_index), ...
                               START_ROW, START_COL);
        % concatinate solution path between waypoints onto the final solution path
        total_path = [total_path; flip(flip(path_pos, 1), 2)];
        % next time, start from the previously reached waypoint
        START_ROW = path_pos(1, 1);
        START_COL = path_pos(1, 2);
        % remove the waypoint already reached from the list of waypoints
        wp_index_rows = find(WP_rows == START_ROW);
        wp_index_cols = find(WP_cols == START_COL);
        WP_rows(intersect(wp_index_rows, wp_index_cols)) = [];
        WP_cols(intersect(wp_index_rows, wp_index_cols)) = [];
        % plot the path between waypoints
        plot(path_pos(:, 2), path_pos(:, 1), '-', 'LineWidth', 3)
        plot(path_pos(:, 2), path_pos(:, 1), 'k.', 'MarkerSize', 15)
    end
    disp("A* calculated a solution in " + num2str(toc) + " seconds.")
end

%{ 
    A* Solving Function
    Reference:  
        E. S. Ueland, R. Skjetne, A. R. Dahl, (2017) Marine Autonomous Exploration Using a Lidar and SLAM
    Input:      
        buff_walls_mat:  logical matrix indicating locations of walls (1's) and open-areas (0's) are.
        START_ROW, START_COL:      row vector storing the vehicle's starting index.
        final_mat:      matrix storing the final goal positions (indices), where
                                rows indicate different waypoints. 
%}
function optimal_path = AstarSearch(buff_walls_mat, WP_ROW, WP_COL, START_ROW, START_COL)
    % create and initialise important matrix variables
    f_scores   = nan(size(buff_walls_mat));   % f-scores: g + h
    g_scores   = zeros(size(buff_walls_mat)); % g-scores: cost of moving
    h_scores   = zeros(size(buff_walls_mat)); % heuristic: estimate of cost to reach final destination
    open_mat   = zeros(size(buff_walls_mat)); % matrix indicating 
    closed_mat = zeros(size(buff_walls_mat)); % matrix keeping a record of locations already evaluated
    parent_col = zeros(size(buff_walls_mat));
    parent_row = zeros(size(buff_walls_mat));
    % initialise the starting position
    open_mat(START_ROW, START_COL) = 1;
    f_scores(START_ROW, START_COL) = h_scores(START_ROW, START_COL);
    % add the wall locations to the open list
    closed_mat(buff_walls_mat == 1) = 1;
    % create the stensil indicating which neighbouring cells should be investigated
    nbr_stensil = ones(3); % 3x3 stensil
    nbr_stensil(2, 2) = 0; % ignore the position the vehicle is currently in
    [nbr_rows, nbr_cols] = find(nbr_stensil == 1);
    nbr_stensil = [nbr_rows, nbr_cols] - 2; % centre the sensil around the vehicle
    % calculate the heuristic matrix
    % loop over the map and find the closest point
    map_rows = 1:size(buff_walls_mat, 1);
    map_cols = 1:size(buff_walls_mat, 2);
    h_scores = sqrt( ( WP_ROW - repelem(map_rows, length(map_cols)) ).^2 + ...
                     ( WP_COL - repmat(map_cols, 1, length(map_rows)) ).^2 );
    h_scores = reshape(h_scores, size(buff_walls_mat, 2), size(buff_walls_mat, 1))';
    h_scores(buff_walls_mat ~= 0) = 0;
    % brute force search for the optimal path to the waypoint
    while (true)
        % check that an optimal path exists
        min_f_score = min(min(f_scores)); % find the smallest f score
        if (isnan(min_f_score))
            % if no optimal path exists, then stop
            uiwait(warndlg("Error: Could not find an optimal path."));
            return
        end
        % find the current location of the vehicle
        [cur_row, cur_col] = find(f_scores == min_f_score);
        % The vehicle is at the end of the path connecting the minimum f_cost locations
        cur_row = cur_row(1);
        cur_col = cur_col(1);
        % check if the vehicle has reached the waypoint
        if ( (cur_row == WP_ROW) && (cur_col == WP_COL) )
            break
        end
        % calculate the next positions
        open_mat(cur_row, cur_col) = 0;
        closed_mat(cur_row, cur_col) = 1;
        f_scores(cur_row, cur_col) = nan;
        % for each of the neighbouring cells
        for nbr_index = 1:size(nbr_stensil, 1)
            nbr_row = nbr_stensil(nbr_index, 1);
            nbr_col = nbr_stensil(nbr_index, 2);
            nxt_pnt_row = cur_row + nbr_row;
            nxt_pnt_col = cur_col + nbr_col;
            % check that the neighbouring cell is open and also not outside the map bounds
            if ( closed_mat(nxt_pnt_row, nxt_pnt_col) == 0  && ...
                ~(nxt_pnt_row < 1 || nxt_pnt_row > size(buff_walls_mat, 1) ||...
                  nxt_pnt_col < 1 || nxt_pnt_col > size(buff_walls_mat, 2)) )
                temp_g_score = g_scores(cur_row, cur_col) + sqrt(nbr_row^2 + nbr_col^2);
                % check that the neighbouring cell is open
                if (open_mat(nxt_pnt_row, nxt_pnt_col) == 0)
                    open_mat(nxt_pnt_row, nxt_pnt_col) = 1; % close the cell
                elseif (temp_g_score >= g_scores(nxt_pnt_row, nxt_pnt_col))
                    continue % if the neighbourning cell costs too much, then don't 
                end
                parent_row(nxt_pnt_row, nxt_pnt_col) = cur_row;
                parent_col(nxt_pnt_row, nxt_pnt_col) = cur_col;
                g_scores(nxt_pnt_row, nxt_pnt_col)   = temp_g_score;
                f_scores(nxt_pnt_row, nxt_pnt_col)   = temp_g_score + h_scores(nxt_pnt_row, nxt_pnt_col);
            end
        end
    end
    % If a solution path was found, then create the solution vector. Trace the solution 
    % from the end point back to the starting position.
    optimal_path(1, :) = [cur_row, cur_col];
    temp_index = 2;
    while ( (cur_row ~= START_ROW) || (cur_col ~= START_COL) ) % De Morgan's laws
        temp_col = parent_col(cur_row, cur_col);
        cur_row  = parent_row(cur_row, cur_col);
        cur_col  = temp_col;
        optimal_path(temp_index, :) = [cur_row, cur_col];
        temp_index = temp_index + 1;
    end
end
