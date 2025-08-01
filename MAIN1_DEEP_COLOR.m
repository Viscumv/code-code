%% Static Initial Scheduling - Complete Implementation for Seabed Terrain Modeling
%% 20240216 (Modified Date)

clc; clear all; close all; warning off;

%% Initialize Random Seed
noRng = 1;
rng('default');
rng(noRng);

%% Environment Parameters Setup
data.mapsize = [100, 100]; % Map dimensions
temp = zeros(data.mapsize);
[p1, p2] = find(temp == 0);
data.index = sub2ind(data.mapsize, p1, p2);
data.node = [p1, p2];
data.maxH = 20; % Maximum altitude (m)
data.minH = 1; % Minimum altitude (m)
data.vmax = 100; % Maximum speed (m/s)
data.vmin = 50; % Minimum speed (m/s)
data.Rmin = 2; % Minimum turning radius (m)
data.minGap = [50, 50, 20]; % Minimum separation requirement
data.numAUV = 4; % Number of AUVs

%% ===== Add Ocean Current Field Data =====
% Create example ocean current field
[data.current_x, data.current_y] = meshgrid(1:10:100, 1:10:100); % Sparse grid points
data.current_u = sin(data.current_y/20) .* cos(data.current_x/15); % X-direction current
data.current_v = cos(data.current_x/25) .* sin(data.current_y/18); % Y-direction current
data.current_w = zeros(size(data.current_u)); % Z-direction current (assumed 0)

%% AUV Start and End Points Setup
data.AUV_S = [20, 10, 5;
              10, 20, 5;
              15, 20, 5;
              20, 20, 5;];
              
data.AUV_E = [90, 90, 5;
              90, 90, 5;
              90, 90, 5;
              90, 90, 5;];

%% Dynamic Obstacles Initialization
data.Ob_M = [60, 80, 5, 0, 5, 0, 0.3, 80, 80; % [x,y,z,vx,vy,radius,chaos,goal_x,goal_y]
             80, 60, 0, 5, 5, 0, 0.4, 80, 80;
             50, 50, 0, -5, 2, 0, 0.45, 50, 10;
             45, 40, -5, 0, 2, 0, 0.35, 10, 40;
             55, 50, -5, 0, 2, 0, 0.35, 10, 50;
             60, 60, -5, 0, 3, 0, 0.35, 10, 60;
             70, 70, -5, 0, 4, 0, 0.35, 10, 70;];
             
data.unitT = 1; % Unit time (s)
data.nita = 1; % Viscosity coefficient

%% ===== Fix 1: Correct Definition of Mountain Obstacles =====
% Mountain obstacle parameters: [x, y, height, x-decay, y-decay]
data.Ob = [40, 20, 7, 15, 10;
           20, 40, 7, 6, 5;
           60, 80, 5, 10, 10;
           80, 60, 7, 6, 5;
           20, 50, 5, 10, 10;
           70, 20, 5, 6, 5;
           10, 10, 7, 6, 5;
           60, 60, 5, 6, 6;
           50, 60, 5, 6, 6;
           60, 50, 7, 6, 6;];
           
data.numOb = size(data.Ob, 1); % Number of mountains

%% ===== Fix 2: Correct Definition of Other Obstacles =====
% Cylinder obstacles: [x, y, radius, height]
data.OB_C = [70, 20, 10, 30; % [x,y,radius,height]
             40, 60, 8, 20];
             
% Rectangular obstacles: [x, y, length, width, height]
data.OB_R = [10, 55, 20, 20, 25; % [x,y,length,width,height]
             30, 30, 15, 15, 17];
             
%% ===== Fix 1: Terrain Visualization with Correct Variables =====
%% Terrain Generation
t = 1:100;
[x, y] = meshgrid(t); % Note: Use lowercase x and y

%% Base Terrain Modeling - Modified: Increased terrain undulation
h1 = 2.0 * sin(y+10) + ... % Amplitude increased from 1.0 to 2.0
     0.5 * sin(x) + ... % Amplitude increased from 0.2 to 0.5
     0.3 * cos(0.6 * sqrt(x.^2 + y.^2)) + ... % Amplitude increased from 0.1 to 0.3
     1.8 * cos(y) + ... % Amplitude increased from 1.1 to 1.8
     1.5 * sin(0.1 * sqrt(x.^2 + y.^2)); % Amplitude increased from 1.0 to 1.5

% Increase noise intensity (from 200% to 250%)
noise_intensity = 2.5 * std(h1(:));
h1 = h1 + noise_intensity * randn(size(h1));

% Gaussian smoothing (reduce smoothing coefficient to preserve details)
h1 = imgaussfilt(h1, 2); % Reduced from 3 to 2

%% Mountain Obstacle Modeling - Modified: Increased mountain height
h3 = zeros(100); % Pre-initialization
for i = 1:data.numOb
    % Get parameters
    x0 = data.Ob(i, 1);
    y0 = data.Ob(i, 2);
    h_val = data.Ob(i, 3) * 1.8; % Height increased by 80%
    x_sigma = data.Ob(i, 4) * 0.7; % Decay coefficient reduced for steeper mountains
    y_sigma = data.Ob(i, 5) * 0.7; % Decay coefficient reduced for steeper mountains
    
    % Gaussian distribution calculation
    exponent = -((x - x0).^2/(2*x_sigma^2) + (y - y0).^2/(2*y_sigma^2));
    h2 = h_val * exp(exponent);
    h3 = max(h3, h2); % Keep maximum value
end

% Composite terrain height
z = max(h1, h3);

% Increase overall terrain height
z = z * 1.5; % Overall height increased by 50%

%% Cylinder Obstacle Modeling - Modified: Increased obstacle height
for i = 1:size(data.OB_C, 1)
    x0 = data.OB_C(i, 1);
    y0 = data.OB_C(i, 2);
    R = data.OB_C(i, 3);
    H = data.OB_C(i, 4) * 1.5; % Height increased by 50%
    dist_matrix = sqrt((x - x0).^2 + (y - y0).^2);
    z(dist_matrix <= R) = H;
end

%% Rectangular Obstacle Modeling - Modified: Increased obstacle height
for i = 1:size(data.OB_R, 1)
    x0 = data.OB_R(i, 1);
    y0 = data.OB_R(i, 2);
    Lx = data.OB_R(i, 3);
    Ly = data.OB_R(i, 4);
    H = data.OB_R(i, 5) * 1.5; % Height increased by 50%
    in_rect = (x >= x0) & (x <= x0 + Lx) & (y >= y0) & (y <= y0 + Ly);
    z(in_rect) = H;
end

% Save terrain data
data.map = z; % 100×100 matrix
data.map_x = x; % Use lowercase x
data.map_y = y; % Use lowercase y

%% ===== Blue to Yellow Color Gradient for Terrain =====
% Create elevation-based colormap (blue to yellow)
color_points = [
    0.0, 0.0, 0.5;    % Deep blue (low elevation)
    0.0, 0.3, 0.8;    % Medium blue
    0.2, 0.6, 0.9;    % Light blue
    0.4, 0.8, 0.7;    % Blue-green
    0.7, 0.9, 0.4;    % Green-yellow
    0.9, 0.9, 0.2;    % Yellow
    1.0, 1.0, 0.0     % Bright yellow (high elevation)
];

num_levels = 256;
elevation_colormap = interp1(linspace(0,1,size(color_points,1)), color_points, linspace(0,1,num_levels));

%% Terrain Visualization - Blue to Yellow Gradient
figure('Position', [100, 100, 1200, 800], 'Color', 'white');

% Terrain surface plot
h_surf = surf(data.map_x, data.map_y, data.map, 'FaceAlpha', 0.85);
colormap(elevation_colormap);
shading interp;

% Create colorbar
c = colorbar('southoutside', 'Ticks', 0:10:80, 'TickLabels', ...
    {'Deep Sea', '', '', 'Mid-Depth', '', 'Shallow', '', '', 'Peaks'});
c.Label.String = 'Elevation (m)';
c.Label.FontSize = 12;
hold on;

% Add lighting for 3D effect
light('Position', [0 0 1000], 'Style', 'infinite');
lighting gouraud;
material([0.4 0.6 0.5 10 0.3]);

% Adjust z-axis range for new terrain height
axis([0, 100, 0, 100, 0, max(data.map(:)) + 10]); % Changed from 80 to dynamic max+10
shading interp;
title('Composite Underwater Terrain with Elevation Gradient');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Elevation (m)');
axis([0, 100, 0, 100, 0, 80]);

%% Save Terrain Data
data.map_z = z;
data.sizeMap = size(data.map_z);

%% AUV Flight Parameters Setup
data.H0 = 4; % Default altitude (m)
data.maxAlpha = 60; % Maximum turn angle (°)
data.maxBeta = 45; % Maximum pitch angle (°)

data.aimP = [90, 60, 0;
             80, 70, 0;
             70, 80, 0;
             60, 90, 0;];
             
data.S = [10, 25, 0;
          15, 20, 0;
          20, 15, 0;
          25, 10, 0;];
          
data.SE = [1, 2, 3, 4];
data.numT = 1; % Number of AUVs per task

%% Set Target Point Heights
for i = 1:size(data.aimP, 1)
    x_pos = data.aimP(i, 1);
    y_pos = data.aimP(i, 2);
    % Ensure coordinates are within range
    x_pos = max(1, min(100, round(x_pos)));
    y_pos = max(1, min(100, round(y_pos)));
    H = data.map(y_pos, x_pos) + data.H0;
    data.aimP(i, 3) = H;
end

%% Set Start Point Heights
for i = 1:size(data.S, 1)
    x_pos = data.S(i, 1);
    y_pos = data.S(i, 2);
    % Ensure coordinates are within range
    x_pos = max(1, min(100, round(x_pos)));
    y_pos = max(1, min(100, round(y_pos)));
    H = data.map(y_pos, x_pos) + data.H0;
    data.S(i, 3) = H;
end

%% Visualize Start and End Points with Blue-Yellow Gradient
figure('Position', [100, 100, 1200, 800], 'Color', 'white');

% Terrain surface plot
h_surf = surf(data.map_x, data.map_y, data.map_z, 'FaceAlpha', 0.85);
colormap(elevation_colormap);
shading interp;
hold on;

% Add lighting
light('Position', [0 0 1000], 'Style', 'infinite');
lighting gouraud;

% Plot ocean current field
h_quiver = quiver3(data.current_x, data.current_y, zeros(size(data.current_u)) + 5,...
                  data.current_u, data.current_v, data.current_w,...
                  'Color', [0.1 0.5 0.8], 'LineWidth', 1.2, 'MaxHeadSize', 0.8,...
                  'AutoScaleFactor', 1.5);

% Plot start and end points
h_start = plot3(data.S(:,1), data.S(:,2), data.S(:,3),...
             'o', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
h_goal = plot3(data.aimP(:,1), data.aimP(:,2), data.aimP(:,3),...
             'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% Add ocean surface effect
[x_surf, y_surf] = meshgrid(0:5:100, 0:5:100);
z_surf = zeros(size(x_surf)) + max(data.map_z(:)) + 5;
h_surface = surf(x_surf, y_surf, z_surf,...
                'FaceColor', [0.3 0.6 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');

% Set view and labels
view(-35, 50);
axis equal;
grid on;
box on;
title('AUV Start and Goal Points with Elevation Gradient');
xlabel('East Distance (m)');
ylabel('North Distance (m)');
zlabel('Depth (m)');

% Create legend
legend([h_start(1), h_goal(1), h_quiver],...
       {'Start Points', 'Goal Points', 'Ocean Current'},...
       'Location', 'northeastoutside',...
       'FontSize', 10);

% Add compass
text(5, 95, max(data.map_z(:)) + 15, 'N',...
      'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
plot3([5,5], [95,90], [max(data.map_z(:)) + 10, max(data.map_z(:)) + 10],...
      'k-', 'LineWidth', 2);

%% Optimization Algorithm Parameters Setup
data.numV = size(data.S, 1);
data.mapSize = size(data.map);
lb = 0;
ub = 1;
dim = prod(data.mapSize);
option.lb = lb;
option.ub = ub;
option.dim = dim;

if length(option.lb) == 1
    option.lb = ones(1, option.dim) * option.lb;
    option.ub = ones(1, option.dim) * option.ub;
end

option.fobj = @aimFcn_2test; % Objective function
option.showIter = 0;

%% Algorithm Parameters Setup
option.numAgent = 100; % Population size
option.maxIteration = 50; % Maximum iterations

%% ===== Add Genetic Algorithm Parameters =====
option.p1_GA = 0.9; % Selection/crossover probability
option.p2_GA = 0.01; % Mutation probability

%% ===== Add IPSO3 Specific Parameters =====
option.numBase = 10; % Base boundary particle count
option.k = 0.8; % Sensitivity coefficient (0.5~1.0)
option.max_density = 0.3; % Maximum obstacle density threshold

% Particle swarm parameters
option.w_pso = 0.6; % Initial inertia weight
option.c1_pso = 2; % Individual learning factor
option.c2_pso = 2; % Social learning factor
option.eliteRatio = 0.3; % Elite ratio
option.levyBeta = 1.5; % Lévy distribution parameter
option.levyAlpha = 0.1; % Lévy step factor

% Dynamic parameter ranges
option.W_max = 0.9; % Maximum inertia weight
option.W_min = 0.4; % Minimum inertia weight
option.c1_min = 0.5; % c1 minimum
option.c1_max = 2.0; % c1 maximum
option.c2_min = 0.5; % c2 minimum
option.c2_max = 2.0; % c2 maximum

%% Algorithm Comparison Setup
str_legend = {'GA', 'BOA',  'GWO','PSO' 'IMOPSO', };
aimFcn = { @GA,  @BOA,  @GWO,  @PSO, @IMOPSO };

%str_legend = {'PSO' };
%aimFcn = {  @PSO  };

%% Initialize Population
x = ones(option.numAgent, option.dim);
y = ones(option.numAgent, 1);

for i = 1:option.numAgent
    x(i, :) = rand(1, option.dim) .* (option.ub - option.lb) + option.lb;
    y(i) = option.fobj(x(i,:), option, data, 0); % Initial without penalty
end

%% Run Optimization Algorithms
bestX = x;
for i = 1:length(aimFcn)
    rng(noRng); % Reset random seed
    tic;
    [bestY(i, :), bestX(i, :), recording(i)] = aimFcn{i}(x, y, option, data);
    tt(i) = toc;
end

%% Plot Fitness Curves
figure;
hold on;
for i = 1:length(aimFcn)
    plot(recording(i).bestFit, 'LineWidth', 2);
end
title('Algorithm Fitness Curves');
xlabel('Iterations');
ylabel('Fitness Value');
grid on;
box on;
legend(str_legend);

%% Visualize Optimization Results with Blue-Yellow Gradient
global_minZ = min(data.map_z(:));
global_maxZ = max(data.map_z(:));
auv_colors = lines(data.numAUV); % Unique colors for each AUV

% Pre-initialize result0 as struct array
result0 = struct('name', {}, 'fit', {}, 'sumD', {}, 'sumE', {}, 'smooth', {}, 'threat', {}, 'path', {});

for i = 1:length(str_legend)
    % Call objective function
    [fit, temp_result] = option.fobj(bestX(i, :), option, data);
    
    % Create named structure
    result_struct = struct(...
        'name', str_legend{i}, ...
        'fit', fit, ...
        'sumD', temp_result.sumD, ...
        'sumE', temp_result.sumE, ...
        'smooth', temp_result.smooth, ...
        'threat', temp_result.threat, ...
        'path', {temp_result.path} ...
    );
    
    % Add to result array
    result0(i) = result_struct;
    
  %% ===== 3D View (Current Perspective) ====
figure('Position', [100, 100, 1200, 800], 'Color', 'white', 'Name', [str_legend{i} ' - 3D View']);

%% 海洋色渐变 (与动态绘图一致)
ocean_colormap = [
    linspace(0.7, 0.3, 64)' ...   % R: 浅蓝到深蓝
    linspace(0.9, 0.5, 64)' ...   % G: 浅青到深青
    linspace(1.0, 0.8, 64)'       % B: 浅蓝到深蓝
];

%% 障碍物颜色 (与动态绘图一致)
obstacleColor = [0.8, 0.3, 0.2];     % 圆柱障碍物颜色 (橙色)
rectObstacleColor = [0.7, 0.4, 0.1]; % 长方体障碍物颜色 (棕色)

%% 地形曲面绘制 (使用海洋色渐变)
h_surf = surf(data.map_x, data.map_y, data.map_z, 'FaceAlpha', 0.85);
colormap(ocean_colormap);
shading interp;

% 设置colorbar (水深标签)
c = colorbar('southoutside', 'Ticks', 0:10:80, 'TickLabels', ...
    {'Deep sea', '', '', 'Continental slope', '', 'Continental shelf', '', '', 'Shallow sea'});
c.Label.String = 'Water depth(m)';
c.Label.FontSize = 12;
hold on;

%% 添加光照效果 (与动态绘图一致)
light('Position', [0 0 1000], 'Style', 'infinite');
lighting gouraud;
material([0.4 0.6 0.5 10 0.3]);

%% 绘制洋流场 (与动态绘图参数一致)
h_quiver = quiver3(data.current_x, data.current_y, zeros(size(data.current_u)) + 5, ...
    data.current_u, data.current_v, data.current_w, ...
    'Color', [0.1 0.5 0.8], 'LineWidth', 1.2, 'MaxHeadSize', 0.8, ...
    'AutoScaleFactor', 1.5);

%% 绘制起点和终点 (使用动态绘图的标记)
% 起点 - 绿色三角形
h_start = plot3(data.S(:,1), data.S(:,2), data.S(:,3), ...
    '^', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% 终点 - 红色五角星
h_goal = plot3(data.aimP(:,1), data.aimP(:,2), data.aimP(:,3), ...
    'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5);



%% 绘制障碍物 (使用动态绘图的颜色和透明度)
% 圆柱体障碍物
h_cyl = [];
for i_obs = 1:size(data.OB_C, 1)
    [xc, yc, zc] = cylinder(data.OB_C(i_obs,3), 50);
    h = surf(xc + data.OB_C(i_obs,1), yc + data.OB_C(i_obs,2), zc * data.OB_C(i_obs,4), ...
        'FaceColor', obstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    h_cyl = [h_cyl, h];
end

% 长方体障碍物
h_rect = [];
for i_obs = 1:size(data.OB_R, 1)
    [xr, yr] = meshgrid(data.OB_R(i_obs,1):(data.OB_R(i_obs,1)+data.OB_R(i_obs,3)), ...
                 data.OB_R(i_obs,2):(data.OB_R(i_obs,2)+data.OB_R(i_obs,4)));
    zr = ones(size(xr)) * data.OB_R(i_obs,5);
    h = surf(xr, yr, zr, ...
        'FaceColor', rectObstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    h_rect = [h_rect, h];
end

%% 设置视角 (与动态绘图一致)
view(-35, 50);
axis equal;
grid on;
box on;
title([str_legend{i} ' Algorithm Paths - 3D View'], 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
zlabel('Depth (m)', 'FontSize', 12);
zlim([0, max(data.map_z(:)) + 10]);

%% 添加指北针 (与动态绘图一致)
text(5, 95, max(data.map_z(:)) + 15, 'N', ...
    'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
plot3([5,5], [95,90], [max(data.map_z(:)) + 10, max(data.map_z(:)) + 10], ...
    'k-', 'LineWidth', 2);

%% 绘制路径 (保持样条插值，使用动态风格)
auv_colors = lines(data.numV);  % 为每个AUV分配独特颜色
path_handles = gobjects(1, data.numV);  % 预分配路径句柄

for noV = 1:data.numV
    path = result0(i).path{noV};
    
    % 路径平滑处理（三次样条插值）
    t_orig = 1:size(path,1);
    t_interp = linspace(1, size(path,1), 200);
    path_x = spline(t_orig, path(:,1), t_interp);
    path_y = spline(t_orig, path(:,2), t_interp);
    path_z = spline(t_orig, path(:,3), t_interp);
    
    % 绘制路径
    h_path = plot3(path_x, path_y, path_z, ...
        'LineWidth', 2.5, 'Color', auv_colors(noV,:));
    path_handles(noV) = h_path;
    
    % 添加路径点标记
    plot3(path(1:10:end,1), path(1:10:end,2), path(1:10:end,3), ...
        '.', 'MarkerSize', 12, 'Color', auv_colors(noV,:));
end

%% 创建图例 (使用动态绘图元素)
legend_entries = [h_start(1), h_goal(1), h_quiver, path_handles];
legend_labels = {'Start Points', 'Goal Points', 'Ocean Current'};

% 添加AUV路径标签
for noV = 1:data.numV
    legend_labels = [legend_labels, ['AUV ' num2str(noV) ' Path']];
end

legend(legend_entries, legend_labels, ...
    'Location', 'northeastoutside', ...
    'FontSize', 10);

%% 保存图像
print(gcf, '-dpng', '-r300', [str_legend{i} '_AUV_Paths_3D.png']);
savefig(gcf, [str_legend{i} '_AUV_Paths_3D.fig']);
    
%% ==== Top View (2D Projection) ====
figure('Position', [100, 100, 1200, 800], 'Color', 'white', 'Name', [str_legend{i} ' - Top View']);

%% 使用海洋色渐变 (与动态绘图一致)
ocean_colormap = [
    linspace(0.7, 0.3, 64)' ...   % R: 浅蓝到深蓝
    linspace(0.9, 0.5, 64)' ...   % G: 浅青到深青
    linspace(1.0, 0.8, 64)'       % B: 浅蓝到深蓝
];

%% 障碍物颜色 (与动态绘图一致)
obstacleColor = [0.8, 0.3, 0.2];     % 圆柱障碍物颜色 (橙色)
rectObstacleColor = [0.7, 0.4, 0.1]; % 长方体障碍物颜色 (棕色)

%% 创建海洋深度图 (使用海洋色渐变)
pcolor(data.map_x, data.map_y, data.map_z);
shading interp;
colormap(ocean_colormap);
caxis([global_minZ, global_maxZ]); % 设置颜色轴范围

% 设置colorbar (水深标签)
c = colorbar('eastoutside');
c.Label.String = 'Water depth(m)';
c.Label.FontSize = 12;
c.Ticks = linspace(global_minZ, global_maxZ, 5); % 设置刻度

hold on;

% 添加等高线
contour(data.map_x, data.map_y, data.map_z, 10, 'k', 'LineWidth', 0.5);

% 绘制洋流场 (2D版本)
if isfield(data, 'current_x') && ~isempty(data.current_x)
    h_quiver = quiver(data.current_x, data.current_y, data.current_u, data.current_v, ...
        'Color', [0.1 0.5 0.8], 'LineWidth', 1.2, 'MaxHeadSize', 0.8, ...
        'AutoScaleFactor', 1.5, 'DisplayName', 'Ocean Current');
else
    h_quiver = [];
end

% 绘制起点 (绿色三角形)
if size(data.S, 1) > 0
    h_start = plot(data.S(:,1), data.S(:,2), ...
        '^', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Start Points');
else
    h_start = [];
end

% 绘制终点 (红色五角星)
if size(data.aimP, 1) > 0
    h_goal = plot(data.aimP(:,1), data.aimP(:,2), ...
        'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Goal Points');
else
    h_goal = [];
end

% 绘制圆柱障碍物 (俯视图为圆形)
h_cyl = [];
if isfield(data, 'OB_C') && size(data.OB_C, 1) > 0
    for i_obs = 1:size(data.OB_C, 1)
        theta = linspace(0, 2*pi, 100);
        x_circle = data.OB_C(i_obs,3) * cos(theta) + data.OB_C(i_obs,1);
        y_circle = data.OB_C(i_obs,3) * sin(theta) + data.OB_C(i_obs,2);
        if i_obs == 1
            h_cyl = patch(x_circle, y_circle, obstacleColor, ...
                'FaceAlpha', 0.7, 'EdgeColor', 'none', 'DisplayName', 'Cylinder Obstacles');
        else
            patch(x_circle, y_circle, obstacleColor, ...
                'FaceAlpha', 0.7, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
end

% 绘制长方体障碍物 (俯视图为矩形)
h_rect = [];
if isfield(data, 'OB_R') && size(data.OB_R, 1) > 0
    for i_obs = 1:size(data.OB_R, 1)
        x_rect = [data.OB_R(i_obs,1), ...
                 data.OB_R(i_obs,1) + data.OB_R(i_obs,3), ...
                 data.OB_R(i_obs,1) + data.OB_R(i_obs,3), ...
                 data.OB_R(i_obs,1)];
        y_rect = [data.OB_R(i_obs,2), ...
                 data.OB_R(i_obs,2), ...
                 data.OB_R(i_obs,2) + data.OB_R(i_obs,4), ...
                 data.OB_R(i_obs,2) + data.OB_R(i_obs,4)];
        if i_obs == 1
            h_rect = patch(x_rect, y_rect, rectObstacleColor, ...
                'FaceAlpha', 0.7, 'EdgeColor', 'none', 'DisplayName', 'Rectangular Obstacles');
        else
            patch(x_rect, y_rect, rectObstacleColor, ...
                'FaceAlpha', 0.7, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
end

% 绘制路径 (2D投影)
auv_colors = lines(data.numV); % 为每个AUV分配独特颜色
path_handles = gobjects(1, data.numV); % 预分配路径句柄

for noV = 1:data.numV
    if length(result0(i).path) >= noV && ~isempty(result0(i).path{noV})
        path = result0(i).path{noV};
        
        % 绘制路径线
        path_handles(noV) = plot(path(:,1), path(:,2), ...
            'LineWidth', 2.5, 'Color', auv_colors(noV,:), ...
            'DisplayName', ['AUV ' num2str(noV) ' Path']);
        
        % 添加路径点标记
        plot(path(1:10:end,1), path(1:10:end,2), ...
            '.', 'MarkerSize', 12, 'Color', auv_colors(noV,:), ...
            'HandleVisibility', 'off');
    else
        path_handles(noV) = gobjects(1); % 空句柄
    end
end

% 设置视图和标签
axis equal;
axis([0 100 0 100]);
grid on;
box on;
title([str_legend{i} ' Algorithm Paths - Top View'], 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);

% 添加指北针
text(5, 95, 'N', ...
    'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
plot([5,5], [95,90], 'k-', 'LineWidth', 2);

% ===== 创建图例 =====
legend_objects = gobjects(0);
legend_labels = {};

% 添加起点
if ~isempty(h_start) && isvalid(h_start)
    legend_objects(end+1) = h_start;
    legend_labels{end+1} = get(h_start, 'DisplayName');
end

% 添加终点
if ~isempty(h_goal) && isvalid(h_goal)
    legend_objects(end+1) = h_goal;
    legend_labels{end+1} = get(h_goal, 'DisplayName');
end

% 添加AUV路径
for noV = 1:data.numV
    if ~isempty(path_handles(noV)) && isvalid(path_handles(noV))
        legend_objects(end+1) = path_handles(noV);
        legend_labels{end+1} = get(path_handles(noV), 'DisplayName');
    end
end

% 添加圆柱障碍物
if ~isempty(h_cyl) && isvalid(h_cyl)
    legend_objects(end+1) = h_cyl;
    legend_labels{end+1} = get(h_cyl, 'DisplayName');
end

% 添加矩形障碍物
if ~isempty(h_rect) && isvalid(h_rect)
    legend_objects(end+1) = h_rect;
    legend_labels{end+1} = get(h_rect, 'DisplayName');
end

% 添加洋流
if ~isempty(h_quiver) && isvalid(h_quiver)
    legend_objects(end+1) = h_quiver;
    legend_labels{end+1} = get(h_quiver, 'DisplayName');
end

% 创建图例
if ~isempty(legend_objects)
    legend(legend_objects, legend_labels, ...
        'Location', 'northeastoutside', ...
        'FontSize', 10);
end


% ===== 最后的安全检查：手动移除任何多余图例项 =====
h_legend = findobj(gcf, 'Type', 'legend');
if ~isempty(h_legend)
    % 获取图例项
    legend_items = get(h_legend, 'String');
    
    % 查找并删除任何包含"data"的项
    valid_indices = true(1, numel(legend_items));
    for idx = 1:numel(legend_items)
        if contains(legend_items{idx}, 'data') || contains(legend_items{idx}, 'Data')
            valid_indices(idx) = false;
        end
    end
    
    % 如果有无效项，重新创建图例
    if any(~valid_indices)
        % 提取有效项
        valid_items = legend_items(valid_indices);
        valid_handles = [];
        
        % 收集有效项对应的图形对象
        for idx = find(valid_indices)
            % 查找具有匹配DisplayName的对象
            for obj_idx = 1:numel(legend_objects)
                if strcmp(get(legend_objects(obj_idx), 'DisplayName'), legend_items{idx})
                    valid_handles = [valid_handles, legend_objects(obj_idx)];
                    break;
                end
            end
        end
        
        % 删除当前图例
        delete(h_legend);
        
        % 创建新图例
        if ~isempty(valid_handles)
            legend(valid_handles, valid_items,...
                'Location', 'northeastoutside',...
                'FontSize', 10);
        end
    end
end

% Add compass
text(5, 95, 'N',...
    'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
plot([5,5], [95,90], 'k-', 'LineWidth', 2);

% Save high-quality image
print(gcf, '-dpng', '-r300', [str_legend{i} '_AUV_Paths_TopView.png']);
savefig(gcf, [str_legend{i} '_AUV_Paths_TopView.fig']);
end

%% Save Results (Add Ocean Current Field Data)
data.Result = result0;
data.current_field = data.current_u; % Save ocean current data

%% Output Algorithm Comparison Table
fprintf('Algorithm\tPath Length\tEnergy\t\tSmoothness\tThreat Penalty\tTotal Fitness\n');
for i = 1:length(result0)
    fprintf('%-8s\t%8.2f\t%8.2f\t%8.4f\t%8.2f\t%8.2f\n', ...
            result0(i).name, ...
            result0(i).sumD, ...
            result0(i).sumE, ...
            result0(i).smooth, ...
            result0(i).threat, ...
            result0(i).fit);
end

% Add grid coordinates save
data.current_x = data.current_x;
data.current_y = data.current_y;
save('AUV_Path_Data.mat', 'data');

%% Radar Chart for Metric Comparison (Normalized)
metric_names = {'Path Length', 'Energy', 'Smoothness', 'Threat Penalty'};
metric_raw = [ [result0.sumD]', [result0.sumE]', [result0.smooth]', [result0.threat]' ];

% Normalize to 0~1
metric_norm = (metric_raw - min(metric_raw)) ./ (max(metric_raw) - min(metric_raw) + 1e-6);

% Generate radar chart data
metric_norm = [metric_norm, metric_norm(:,1)]'; 

% Create new figure and clear previous content
figure('Name','Algorithm Metric Radar Chart', 'NumberTitle', 'off');
clf;  % Clear existing plots

% Prepare circular axis
theta = linspace(0, 2*pi, size(metric_norm,1));

% Plot algorithms with explicit handles
h1 = polarplot(theta, metric_norm(:,1), '-o', 'LineWidth', 2, 'DisplayName', 'GA'); 
hold on;
h2 = polarplot(theta, metric_norm(:,2), '-s', 'LineWidth', 2, 'DisplayName', 'IMOPSO');
h3 = polarplot(theta, metric_norm(:,3), '-^', 'LineWidth', 2, 'DisplayName', 'PSO');

% Configure axis labels
thetaticks(rad2deg(theta(1:end-1)))
thetaticklabels([metric_names, metric_names(1)])

% 关键修复：使用显式句柄创建图例
legend([h1, h2, h3], 'Location', 'best'); 

title('Standardized Algorithm Performance Radar Chart (Lower Values Better)');