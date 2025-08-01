function [fit, result] = aimFcn_2test(x, option, data, iteration)

%% 移除全局变量声明 - 使用传入的data参数
% global data; % 删除此行

%% 初始化路径存储
path = cell(data.numV, 1);
flagE = zeros(size(data.aimP, 1), 1);
recording = [];
direction = getDirectiontest(1); % 获取运动方向
numR = size(direction, 1);
xx = reshape(x, data.mapSize(1), data.mapSize(2));

%% ===== 修复1: 添加洋流场存在性检查 =====
if ~isfield(data, 'current_field')
    % 创建默认洋流场（零流速）
    data.current_field = zeros(data.mapsize(1), data.mapsize(2), 3);
end

%% 遍历所有AUV
for noV = 1:data.numV
    %% 初始化当前AUV参数
    nowP = data.S(noV, :); % 当前位置
    prevP = nowP; % 前一位置
    prevV = [0, 0, 0]; % 初始速度
    prevA = [0, 0, 0]; % 初始加速度
    aimIdx = data.SE(noV); % 目标索引
    aimP = data.aimP(aimIdx, :); % 目标位置
    flag = zeros(data.mapSize); % 访问标志矩阵
    flag(nowP(1), nowP(2)) = 1; % 标记起点
    
    %% 路径初始化
    path{noV} = [nowP, 0, 0, 0, 0, 0, 0]; % [x,y,z,cost,dist,dx,dy,dz,v,a]
    count = 0; % 步数计数器
    
    %% 路径搜索循环
    while true
        count = count + 1;
        if count > 5000 % 防止无限循环
            break;
        end
        
        %% === 添加当前速度计算 ===
        current_speed = norm(prevV(1:2));  % 仅考虑水平面速度
        
        %% 生成候选点
        nextP = repmat(nowP(1:2), numR, 1) + direction;
        
        %% 移除越界点
        invalid_idx = (nextP(:, 1) <= 0 | nextP(:, 2) <= 0 | ...
                     nextP(:, 1) > data.mapSize(2) | nextP(:, 2) > data.mapSize(1));
        nextP(invalid_idx, :) = [];
        
        %% 计算各候选点代价
        costs = inf(size(nextP, 1), 1);
        for i = 1:size(nextP, 1)
            x_pos = nextP(i, 1);
            y_pos = nextP(i, 2);
            
            %% 高度约束计算 (文档高度模型)
            terrainH = data.map(y_pos, x_pos);
            horizDist = norm([x_pos, y_pos] - nowP(1:2));
            maxPitch = tand(data.maxBeta) * horizDist;
            maxH = nowP(3) + maxPitch;
            
            %% === 修改1: 增加2米安全高度 ===
            minH = max(terrainH + data.minH + 2, nowP(3) - maxPitch); % 增加2米安全高度
            %% ===========================
            
            %% 决策变量控制高度
            H0 = (data.maxH - data.minH) * xx(y_pos, x_pos) + data.minH;
            newH = min(max(terrainH + H0, minH), maxH);
            
            %% ===== 修复2: 安全的洋流场访问 =====
            % 确保坐标在洋流场范围内
            x_idx = min(max(1, round(x_pos)), size(data.current_field, 1));
            y_idx = min(max(1, round(y_pos)), size(data.current_field, 2));
            
            % 获取洋流向量
            current_vec = squeeze(data.current_field(x_idx, y_idx, :))';
            current_effect = norm(current_vec) * 0.1; % 洋流影响因子
            
            %% 动态障碍物检测 (文档式3.9)
            obs_cost = 0;
            if isfield(data, 'Ob_M') % 检查动态障碍物是否存在
                for obs_idx = 1:size(data.Ob_M, 1)
                    obs_pos = data.Ob_M(obs_idx, 1:3);
                    obs_radius = data.Ob_M(obs_idx, 6);
                    dist_to_obs = norm([x_pos, y_pos, newH] - obs_pos);
                    if dist_to_obs < obs_radius + 5 % 安全距离
                        obs_cost = obs_cost + 1000 / max(0.1, dist_to_obs - obs_radius);
                    end
                end
            end
            
            %% === 修改2: 使用动态安全距离 ===
            safe_dist = 3 + 2*(1 - exp(-current_speed/10)); % 动态安全距离
            
            %% === 静态障碍物安全检测 ===
            static_penalty = 0;
            
            % 圆柱障碍物检测
            for j = 1:size(data.OB_C, 1)
                dist_to_cyl = norm([x_pos, y_pos] - data.OB_C(j, 1:2));
                if dist_to_cyl < (data.OB_C(j, 3) + safe_dist) % 使用动态安全距离
                    penalty = 500 * exp(-0.5*(dist_to_cyl)/(data.OB_C(j,3)+safe_dist)); 
                    static_penalty = static_penalty + penalty;
                end
            end
            
            % 长方体障碍物检测
            for j = 1:size(data.OB_R, 1)
                dx = max([data.OB_R(j,1)-x_pos, 0, x_pos-(data.OB_R(j,1)+data.OB_R(j,3))]);
                dy = max([data.OB_R(j,2)-y_pos, 0, y_pos-(data.OB_R(j,2)+data.OB_R(j,4))]);
                dist_to_rect = norm([dx, dy]);
                if dist_to_rect < safe_dist % 使用动态安全距离
                    penalty = 500 * exp(-0.5*(dist_to_rect/safe_dist)); 
                    static_penalty = static_penalty + penalty;
                end
            end
            %% =============================
            
            %% 代价计算 (文档多目标代价)
            dist_to_aim = norm([x_pos, y_pos, newH] - aimP);
            dist_from_now = norm([x_pos, y_pos, newH] - nowP);
            
            %% === 修改3: 增加静态障碍物惩罚 ===
            costs(i) = 0.5 * dist_to_aim + 0.3 * dist_from_now + ...
                       0.2 * current_effect + obs_cost + static_penalty;
            %% =============================
            
            %% 约束检查
            if newH < terrainH + data.minH || newH > terrainH + data.maxH
                costs(i) = inf;
            end
            if flag(x_pos, y_pos) == 1 % 已访问点
                costs(i) = inf;
            end
        end
        
        %% 选择最佳候选点
        [min_cost, best_idx] = min(costs);
        if isinf(min_cost) % 无可行路径
            if size(path{noV}, 1) > 1
                nowP = path{noV}(end-1, 1:3); % 回溯
                path{noV}(end, :) = []; % 移除当前点
                continue;
            else
                break; % 无法回溯
            end
        end
        
        %% 更新位置
        x_pos = nextP(best_idx, 1);
        y_pos = nextP(best_idx, 2);
        terrainH = data.map(y_pos, x_pos);
        horizDist = norm([x_pos, y_pos] - nowP(1:2));
        maxPitch = tand(data.maxBeta) * horizDist;
        maxH = nowP(3) + maxPitch;
        
        %% === 修改4: 保持高度安全余量 ===
        minH = max(terrainH + data.minH + 2, nowP(3) - maxPitch); % 增加2米安全高度
        %% ===========================
        
        H0 = (data.maxH - data.minH) * xx(y_pos, x_pos) + data.minH;
        newH = min(max(terrainH + H0, minH), maxH);
        newPos = [x_pos, y_pos, newH];
        
        %% 计算运动学参数 (文档式3.9)
        dt = 1; % 时间步长
        V = (newPos - nowP) / dt;
        A = (V - prevV) / dt;
        
        %% 能耗计算 (文档式5.4)
        horizDist = norm(newPos(1:2) - nowP(1:2));
        vertDist = abs(newPos(3) - nowP(3));
        
        %% 更新路径
        path{noV} = [path{noV};
                     newPos, min_cost, norm(newPos - nowP), ...
                     horizDist, vertDist, norm(V), norm(A)];
        
        %% 更新状态
        prevP = nowP;
        prevV = V;
        nowP = newPos;
        flag(x_pos, y_pos) = 1; % 标记已访问
        
        %% 检查是否到达目标
        if norm(nowP - aimP) <= 3
            path{noV} = [path{noV};
                         aimP, norm(nowP - aimP), norm(nowP - aimP), ...
                         norm(aimP(1:2) - nowP(1:2)), abs(aimP(3) - nowP(3)), ...
                         norm(V), norm(A)];
            break;
        end
    end
    
    %% 计算路径长度
    pathLen = 0;
    for i = 1:size(path{noV}, 1)-1
        pathLen = pathLen + norm(path{noV}(i+1, 1:3) - path{noV}(i, 1:3));
    end
    sumD(noV) = pathLen;
    
    %% ======= 平滑度惩罚计算 =======
    smoothPenalty = 0;
    for noV = 1:data.numV
        path_now = path{noV};
        if size(path_now,1) >= 3
            angle_changes = [];
            for j = 2:size(path_now,1)-1
                v1 = path_now(j,:) - path_now(j-1,:);
                v2 = path_now(j+1,:) - path_now(j,:);
                angle = acos(dot(v1(1:3), v2(1:3)) / (norm(v1(1:3)) * norm(v2(1:3)) + 1e-6));
                angle_changes(end+1) = angle;
            end
            smoothPenalty = smoothPenalty + var(angle_changes); % 角度变化的方差
        end
    end
    
    %% ======= 威胁惩罚项计算 =======
    threatPenalty = 0;
    if isfield(data, 'Ob')
        for noV = 1:data.numV
            for j = 1:size(data.Ob,1)
                cx = data.Ob(j,1); cy = data.Ob(j,2); h = data.Ob(j,3);
                r = mean([data.Ob(j,4), data.Ob(j,5)]);
                for i = 1:size(path{noV},1)
                    d = norm(path{noV}(i,1:2) - [cx,cy]);
                    if d < r + 5
                        threatPenalty = threatPenalty + 100 / (d + 1);
                    end
                end
            end
        end
    end
    
    %% 计算路径能耗 (文档式5.4)
    energy = 0;
    alpha = 0.5; beta = 0.2; gamma = 0.3;
    for i = 1:size(path{noV}, 1)-1
        vel = path{noV}(i, 8);
        acc = path{noV}(i, 9);
        
        %% ===== 修复3: 安全的洋流场访问 =====
        % 确保坐标在洋流场范围内
        x_idx = min(max(1, round(path{noV}(i, 1))), size(data.current_field, 1));
        y_idx = min(max(1, round(path{noV}(i, 2))), size(data.current_field, 2));
        current_vec = squeeze(data.current_field(x_idx, y_idx, :))';
        % 修复：vel 是标量，current_vec 是向量，需处理维度
if length(current_vec) >= 2
    current_speed = norm(current_vec(1:2));
else
    current_speed = 0; % 若 current_vec 不足两维，设为 0
end
current_effect = abs(vel - current_speed); % 两个标量相减

        
        energy = energy + alpha * vel^2 + beta * acc^2 + gamma * current_effect;
    end
    sumE(noV) = energy;
end

%% ======= 总适应度函数加罚项 =======
lambda1 = 0.1; lambda2 = 100; lambda3 = 50; % 可调节权重
penalty = lambda1 * smoothPenalty + lambda2 * threatPenalty;
fit = sum(sumD) + sum(sumE) + penalty;

%% 返回更多指标
if nargout > 1
    result.fit = fit;
    result.sumD = sum(sumD);
    result.sumE = sum(sumE);
    result.smooth = smoothPenalty;
    result.threat = threatPenalty;
    result.path = path;
end
end
