%% 水下AUV路径规划可视化与评估系统

%% 2024.2.16

close all;
clear;
clc;

%% 加载数据
load('AUV_Path_Data.mat', 'data');

%% 海洋色渐变 (与静态绘图一致)
ocean_colormap = [
    linspace(0.7, 0.3, 64)' ... % R: 浅蓝到深蓝
    linspace(0.9, 0.5, 64)' ... % G: 浅青到深青
    linspace(1.0, 0.8, 64)' % B: 浅蓝到深蓝
];

%% 障碍物颜色 (与静态绘图一致)
obstacleColor = [0.8, 0.3, 0.2]; % 圆柱障碍物颜色 (橙色)
rectObstacleColor = [0.7, 0.4, 0.1]; % 长方体障碍物颜色 (棕色)

%% 修复缺失字段
% 补充 M_Obstacle 定义
if ~isfield(data, 'M_Obstacle')
    data.M_Obstacle = [81,43,15,52,53,16,1;
                       21,33,45,32,23,16,1;
                       31,60,15,48,29,16,1;
                       52,89,10,44,32,20,1];
end

% 补充 numV 和 numMOB
if ~isfield(data, 'numV')
    data.numV = size(data.S, 1); % AUV数量 = 起始点数量
end

if ~isfield(data, 'numMOB')
    data.numMOB = size(data.M_Obstacle, 1); % 动态障碍物数量
end

%% ========== 关键修改1: 增大静态障碍物尺寸 ==========
% 创建障碍物副本并增大尺寸
OB_C_enlarged = data.OB_C;
OB_R_enlarged = data.OB_R;



%% 遍历所有算法结果
for noAA = 1:length(data.Result)
    %% 获取当前算法结果
    result = data.Result(noAA);
    algorithm_name = result.name;
    
    %% ========== 静态路径可视化 ==========
    figure('Position', [100, 100, 1200, 800], 'Color', 'white');
    
    % 地形曲面绘制 (与静态绘图一致)
    h_surf = surf(data.map_x, data.map_y, data.map_z, 'FaceAlpha', 0.85);
    colormap(ocean_colormap);
    shading interp;
    c = colorbar('southoutside', 'Ticks', 0:10:80, 'TickLabels', ...
        {'深海', '', '', '大陆坡', '', '大陆架', '', '', '浅海'});
    c.Label.String = '水深(m)';
    c.Label.FontSize = 12;
    hold on;
    
    % 添加光照效果增强立体感 (与静态绘图一致)
    light('Position', [0 0 1000], 'Style', 'infinite');
    lighting gouraud;
    material([0.4 0.6 0.5 10 0.3]);
    
    % 绘制洋流场 (与静态绘图一致)
    h_quiver = quiver3(data.current_x, data.current_y, zeros(size(data.current_u)) + 5,...
              data.current_u, data.current_v, data.current_w,...
              'Color', [0.1 0.5 0.8], 'LineWidth', 1.2, 'MaxHeadSize', 0.8,...
              'AutoScaleFactor', 1.5);
    
    % 绘制起点和终点 (与静态绘图一致)
    h_start = plot3(data.S(:,1), data.S(:,2), data.S(:,3),...
             '^', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0],...
             'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    h_goal = plot3(data.aimP(:,1), data.aimP(:,2), data.aimP(:,3),...
             'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0],...
             'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % 添加海面效果 (与静态绘图一致)
    [x_surf, y_surf] = meshgrid(0:5:100, 0:5:100);
    z_surf = zeros(size(x_surf)) + max(data.map_z(:)) + 5;
    h_surface = surf(x_surf, y_surf, z_surf,...
               'FaceColor', [0.3 0.6 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    
    % 绘制障碍物 (使用增大后的尺寸)
    % 圆柱体障碍物
    for i = 1:size(OB_C_enlarged, 1)
        [xc, yc, zc] = cylinder(OB_C_enlarged(i, 3), 50);
        surf(xc + OB_C_enlarged(i, 1), yc + OB_C_enlarged(i, 2), zc * OB_C_enlarged(i, 4),...
             'FaceColor', obstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
    
    % 长方体障碍物
    for i = 1:size(OB_R_enlarged, 1)
        [xr, yr] = meshgrid(OB_R_enlarged(i, 1):(OB_R_enlarged(i, 1)+OB_R_enlarged(i, 3)),...
                   OB_R_enlarged(i, 2):(OB_R_enlarged(i, 2)+OB_R_enlarged(i, 4)));
        zr = ones(size(xr)) * OB_R_enlarged(i, 5);
        surf(xr, yr, zr, 'FaceColor', rectObstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
    
    % 设置视角和标签 (与静态绘图一致)
    view(-35, 50);
    axis equal;
    grid on;
    box on;
    title([algorithm_name ' Algorithm path'], 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    zlabel('Depth (m)', 'FontSize', 12);
    
    % 设置z轴范围
    zlim([0, max(data.map_z(:)) + 10]);
    
    % 绘制路径 (与静态绘图一致)
    auv_colors = lines(data.numV); % 为每个AUV分配独特颜色
    for noV = 1:data.numV
        % 路径平滑处理（三次样条插值）
        raw_path = result.path{noV};
        t_orig = 1:size(raw_path, 1);
        t_interp = linspace(1, size(raw_path, 1), 200);
        path_x = spline(t_orig, raw_path(:,1), t_interp);
        path_y = spline(t_orig, raw_path(:,2), t_interp);
        path_z = spline(t_orig, raw_path(:,3), t_interp);
        
        % 绘制路径
        h_path = plot3(path_x, path_y, path_z,...
                 'LineWidth', 2.5, 'Color', auv_colors(noV,:));
        
        % 添加路径点标记
        plot3(raw_path(1:10:end,1), raw_path(1:10:end,2), raw_path(1:10:end,3),...
             '.', 'MarkerSize', 12, 'Color', auv_colors(noV,:));
    end
    
    % 创建图例 (与静态绘图一致)
    legend_entries = [h_start(1), h_goal(1), h_path, h_quiver];
    legend_labels = {'起点', '目标点', 'AUV路径', '洋流场'};
    legend(legend_entries, legend_labels,...
           'Location', 'northeastoutside', 'FontSize', 10);
    
    % 添加指北针 (与静态绘图一致)
    text(5, 95, max(data.map_z(:)) + 15, 'N',...
         'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    plot3([5,5], [95,90], [max(data.map_z(:)) + 10, max(data.map_z(:)) + 10],...
          'k-', 'LineWidth', 2);


   %% ========== 动态避障模拟 ==========
    % 创建独立图形窗口并保持打开
    fig = figure('Position', [100, 100, 1200, 800], 'Color', 'white', ...
                'Name', [algorithm_name ' Dynamic Obstacle Avoidance']);
    
    % 地形曲面绘制
    h_surf = surf(data.map_x, data.map_y, data.map_z, 'FaceAlpha', 0.85);
    colormap(ocean_colormap);
    shading interp;
    c = colorbar('southoutside', 'Ticks', 0:10:80, 'TickLabels', ...
        {'Deep sea', '', '', 'Continental slope', '', 'Continental shelf', '', '', 'Shallow sea'});
    c.Label.String = 'Water depth(m)';
    c.Label.FontSize = 12;
    hold on;
    
    % 添加光照效果
    light('Position', [0 0 1000], 'Style', 'infinite');
    lighting gouraud;
    material([0.4 0.6 0.5 10 0.3]);
    
    % 绘制洋流场 (关闭图例显示)
    h_quiver = quiver3(data.current_x, data.current_y, zeros(size(data.current_u)) + 5,...
        data.current_u, data.current_v, data.current_w,...
        'Color', [0.1 0.5 0.8], 'LineWidth', 1.2, 'MaxHeadSize', 0.8,...
        'AutoScaleFactor', 1.5, 'HandleVisibility', 'off');
    
    % 绘制起点 (只显示第一个起点在图例)
    h_start = plot3(data.S(1,1), data.S(1,2), data.S(1,3),...
        '^', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0],...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Start Points');
    plot3(data.S(2:end,1), data.S(2:end,2), data.S(2:end,3),...
        '^', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0],...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % 绘制终点 (只显示第一个终点在图例)
    h_goal = plot3(data.aimP(1,1), data.aimP(1,2), data.aimP(1,3),...
        'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0],...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Goal Points');
    plot3(data.aimP(2:end,1), data.aimP(2:end,2), data.aimP(2:end,3),...
        'pentagram', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.2 0],...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % 使用原始静态障碍物尺寸
    OB_C_original = data.OB_C;
    OB_R_original = data.OB_R;
    
    % 绘制圆柱体障碍物 (只显示第一个在图例)
    h_cyl = [];
    for i = 1:size(OB_C_original, 1)
        [xc, yc, zc] = cylinder(OB_C_original(i, 3), 50);
        if i == 1
            h = surf(xc + OB_C_original(i, 1), yc + OB_C_original(i, 2), zc * OB_C_original(i, 4),...
                'FaceColor', obstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7, ...
                'DisplayName', 'Cylinder Obstacles');
        else
            h = surf(xc + OB_C_original(i, 1), yc + OB_C_original(i, 2), zc * OB_C_original(i, 4),...
                'FaceColor', obstacleColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7, ...
                'HandleVisibility', 'off');
        end
        h_cyl = [h_cyl, h];
    end
    
    % 绘制长方体障碍物 (只显示第一个在图例)
    h_rect = [];
    for i = 1:size(OB_R_original, 1)
        [xr, yr] = meshgrid(OB_R_original(i, 1):(OB_R_original(i, 1)+OB_R_original(i, 3)),...
            OB_R_original(i, 2):(OB_R_original(i, 2)+OB_R_original(i, 4)));
        zr = ones(size(xr)) * OB_R_original(i, 5);
        if i == 1
            h = surf(xr, yr, zr, 'FaceColor', rectObstacleColor, 'EdgeColor', 'none', ...
                'FaceAlpha', 0.7, 'DisplayName', 'Rectangular Obstacles');
        else
            h = surf(xr, yr, zr, 'FaceColor', rectObstacleColor, 'EdgeColor', 'none', ...
                'FaceAlpha', 0.7, 'HandleVisibility', 'off');
        end
        h_rect = [h_rect, h];
    end
    
    % 设置视角和标签
    view(-35, 50);
    axis equal;
    grid on;
    box on;
    title([algorithm_name ' Dynamic Obstacle Avoidance'], 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    zlabel('Depth (m)', 'FontSize', 12);
    zlim([0, max(data.map_z(:)) + 10]);
    
    % 添加指北针
    text(5, 95, max(data.map_z(:)) + 15, 'N',...
        'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    plot3([5,5], [95,90], [max(data.map_z(:)) + 10, max(data.map_z(:)) + 10],...
        'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    
    % 初始化动态障碍物
    nowP_M = data.M_Obstacle(:, 1:3);
    path_M = cell(data.numMOB, 1);
    for noM = 1:data.numMOB
        path_M{noM} = nowP_M(noM, :);
    end
    
    % 初始化AUV路径
    newPath = cell(data.numV, 1);
    aimP_index = ones(data.numV, 1);
    for noV = 1:data.numV
        newPath{noV} = result.path{noV}(1, :);
    end
    
    % 初始化跳过点容器 (用于存储动态与静态路径不重合的点)
    skippedPoints = cell(data.numV, 1);  
    h_skipped = gobjects(0);            
    
    % 动态避障模拟参数
    simulationSteps = 200;
    auv_colors = lines(data.numV); % 为每个AUV分配独特颜色
    
    % ====== 创建图例占位符 ======
    % AUV计划路径占位符
    h_auv_planned = gobjects(1, data.numV);
    for noV = 1:data.numV
        h_auv_planned(noV) = plot3(NaN, NaN, NaN, '--', 'Color', auv_colors(noV, :), ...
            'LineWidth', 1.5, 'DisplayName', ['AUV' num2str(noV) ' Planned Path']);
    end
    
    % AUV实际路径占位符
    h_auv_actual = gobjects(1, data.numV);
    for noV = 1:data.numV
        h_auv_actual(noV) = plot3(NaN, NaN, NaN, '-', 'Color', auv_colors(noV, :), ...
            'LineWidth', 2.5, 'DisplayName', ['AUV' num2str(noV) ' Actual Path']);
    end
    
    % 跳过点占位符 (红色小球) - 表示不重合部分
    h_skipped_point = scatter3(NaN, NaN, NaN, 80, 'ro', 'filled', ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'DisplayName', 'Deviated Points');
    
    % ====== 创建图例 ======
    legend_handles = [h_start, h_goal, h_quiver, h_cyl(1), h_rect(1)];
    for noV = 1:data.numV
        legend_handles = [legend_handles, h_auv_planned(noV)];
    end
    for noV = 1:data.numV
        legend_handles = [legend_handles, h_auv_actual(noV)];
    end
    legend_handles = [legend_handles, h_skipped_point];
    
    % 创建图例（单列显示）
    legend(legend_handles, 'Location', 'northeastoutside', 'FontSize', 10);
    
    % 动态对象句柄容器
    dyn_handles = [];
    
    %% 动态避障主循环
    for t = 1:simulationSteps
        % 删除上一帧动态对象 (保留跳过点标记)
        if ~isempty(dyn_handles)
            delete(dyn_handles(ishandle(dyn_handles)));
        end
        dyn_handles = [];
        
        %% 障碍物移动更新
        v_M = data.M_Obstacle(:, 4:6) - data.M_Obstacle(:, 1:3);
        for i = 1:data.numMOB
            % 计算到目标点的距离
            current_pos = nowP_M(i, :);
            target_pos = data.M_Obstacle(i, 4:6);
            dist_to_target = norm(target_pos - current_pos);
            if dist_to_target > 3
                step = 0.5 * v_M(i, :) / norm(v_M(i, :));
                nowP_M(i, :) = current_pos + step;
            end
            
            % 添加边界约束
            nowP_M(i, 1) = max(1, min(100, nowP_M(i, 1)));
            nowP_M(i, 2) = max(1, min(100, nowP_M(i, 2)));
            
            % 地形高度约束
            x = round(nowP_M(i, 1));
            y = round(nowP_M(i, 2));
            terrainH = data.map_z(y, x);
            if nowP_M(i, 3) < terrainH + 5
                nowP_M(i, 3) = terrainH + 5;
            end
            
            % 更新轨迹
            path_M{i} = [path_M{i}; nowP_M(i, :)];
        end
        
        %% 更新AUV路径点
        for noV = 1:data.numV
            path_completed = false;
            while ~path_completed && aimP_index(noV) <= size(result.path{noV}, 1)
                current_target = result.path{noV}(aimP_index(noV), :);
                
                % 动态避障检测
                collision = false;
                for noM = 1:data.numMOB
                    if norm(current_target(1:3) - nowP_M(noM, :)) < 10
                        collision = true;
                        break;
                    end
                end
                
                if ~collision
                    % 安全则添加当前点（重合部分）
                    newPath{noV} = [newPath{noV}; current_target];
                    aimP_index(noV) = aimP_index(noV) + 1;
                    break;
                else
                    % 不安全则跳过当前点（不重合部分）
                    % 记录被跳过的点
                    skippedPoints{noV} = [skippedPoints{noV}; current_target];
                    
                    % 绘制红色小球标记差异点
                    h = scatter3(current_target(1), current_target(2), current_target(3),...
                                80, 'ro', 'filled', 'MarkerFaceColor', 'r', ...
                                'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
                    h_skipped = [h_skipped, h];  % 保存句柄避免删除
                    
                    aimP_index(noV) = aimP_index(noV) + 1;
                    if aimP_index(noV) > size(result.path{noV}, 1)
                        path_completed = true;
                    end
                end
            end
        end
        
        %% 绘制动态障碍物轨迹
        for noM = 1:data.numMOB
            if ~isempty(path_M{noM})
                % 轨迹线
                h_traj = plot3(path_M{noM}(:, 1), path_M{noM}(:, 2), path_M{noM}(:, 3),...
                    '-', 'LineWidth', 2, 'Color', [1 0.6 0], 'HandleVisibility', 'off');
                dyn_handles = [dyn_handles, h_traj];
                
                % 当前位置标记
                h_pos = scatter3(nowP_M(noM, 1), nowP_M(noM, 2), nowP_M(noM, 3),...
                    100, 'filled', 'MarkerFaceColor', [1 0.4 0], ...
                    'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
                dyn_handles = [dyn_handles, h_pos];
            end
        end
        
        %% 绘制AUV路径
        for noV = 1:data.numV
            % 原计划路径 (使用不同颜色)
            if ~isempty(result.path{noV})
                h_plan = plot3(result.path{noV}(:, 1), result.path{noV}(:, 2), result.path{noV}(:, 3),...
                    '--', 'LineWidth', 1.5, 'Color', auv_colors(noV, :), 'HandleVisibility', 'off');
                dyn_handles = [dyn_handles, h_plan];
            end
            
            % 实时路径 (使用不同颜色)
            if size(newPath{noV}, 1) > 1
                % 实际路径线
                h_actual = plot3(newPath{noV}(:, 1), newPath{noV}(:, 2), newPath{noV}(:, 3),...
                    '-', 'LineWidth', 2.5, 'Color', auv_colors(noV, :), 'HandleVisibility', 'off');
                dyn_handles = [dyn_handles, h_actual];
                
                % 当前位置 (使用AUV特定颜色)
                h_auvpos = scatter3(newPath{noV}(end, 1), newPath{noV}(end, 2), newPath{noV}(end, 3),...
                    100, 'filled', 'MarkerFaceColor', auv_colors(noV, :), ...
                    'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
                dyn_handles = [dyn_handles, h_auvpos];
            end
        end
        
        % 绘制所有跳过的点 (确保它们不被删除)
        for noV = 1:data.numV
            if ~isempty(skippedPoints{noV})
                for i = 1:size(skippedPoints{noV}, 1)
                    point = skippedPoints{noV}(i, :);
                    h = scatter3(point(1), point(2), point(3),...
                                80, 'ro', 'filled', 'MarkerFaceColor', 'r', ...
                                'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
                    h_skipped = [h_skipped, h];
                end
            end
        end
        
        
        %% 视角控制
        drawnow;
        pause(0.05);
        
        % 确保图形窗口保持打开状态
        set(fig, 'Visible', 'on');
    end  % 结束动态避障主循环
    
    % 显示完成信息
    disp(['Completed dynamic simulation for algorithm: ' algorithm_name]);
    
    % 确保所有图形窗口保持打开
    drawnow;
end  % 结束算法循环

% 完成所有算法处理
disp('All algorithm simulations completed.');