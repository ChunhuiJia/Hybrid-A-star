function costmap = GridAStar(obstlist,goal,gres)
    [minx,miny,obmap] = CalcObstMap(obstlist,gres);   % gres，单元格分辨率，是用来搜索路径的单元格长度吗？、/
    col = goal(1);  %目标点的x坐标
    row = goal(2);  %目标点的y坐标
    col = ceil((col-minx)/gres);
    row = ceil((row-miny)/gres);
%     goal = [gx,gy];
%     goal.parent = [gx,gy];
%     goal.precost = 0;
%     goal.postcost = inf;
    goal = [row,col];
    costmap = 0*obmap; %这个costmap怎么理解呢？？？
    dim = size(obmap);     
    for i = 1:dim(1)
        for j = 1:dim(2)
            if obmap(i,j) == 1
                costmap(i,j) = inf;  %障碍物边缘处的cost置为inf
                continue
            elseif i == col && j == row  %若查找到目标点时，怎么样？？？
                continue
            end            
            start = [i,j]; %start点为什么是这样的？遍历所有的可查找点到目标点的AStar搜索吗？为什么这么做？
            cost = AStarSearch(start,goal,obmap);
            costmap(i,j) = cost;
        end
    end    
end

function cost = AStarSearch(start,goal,obmap)
    dim = size(obmap);
    % Grids(i,j,1) - x of parent pos; 2 - y of parent pos; 3 - precost; 4 -
    % postcost
    Grids = zeros(dim(1),dim(2),4);
    for i = 1:dim(1)
        for j = 1:dim(2)
            Grids(i,j,1) = i; % 父节点的所在行
            Grids(i,j,2) = j; % 父节点的所在列
            Grids(i,j,3) = norm(([i,j]-goal)); % 启发值h，直线距离
            Grids(i,j,4) = inf; % g值
        end
    end
    Open = [start];
    Grids(start(1),start(2),4) = 0;
    Close = [];
    while ~isempty(Open)
        [wknode,Open] = PopOpen(Open,Grids);
        [Grids,Open,Close] = Update(wknode,goal,obmap,Grids,Open,Close);
        Close(end+1,:) = wknode;
    end
    cost = Grids(goal(1),goal(2),3)+Grids(goal(1),goal(2),4);
end

function [Grids,Open,Close] = Update(wknode,goal,obmap,Grids,Open,Close)
    dim = size(obmap);
    for i = -1:1
        for j = -1:1
            adjnode = wknode+[i,j];
            row = adjnode(1);
            col = adjnode(2);
            if i == 0 && j == 0
                continue
            elseif row < 1 || row > dim(1)
                continue
            elseif col < 1 || col > dim(2)
                continue
            elseif obmap(row,col) == 1
                continue
            end
            tcost = Grids(wknode(1),wknode(2),4)+norm([i,j]);
            if Grids(row,col,4) > tcost
                Grids(row,col,1) = wknode(1);
                Grids(row,col,2) = wknode(2);
                Grids(row,col,4) = tcost;
                % add adjnode to Open except wknode is goal
                if ~ismember(adjnode,Open,'rows') && ~isequal(adjnode,goal)
                    Open(end+1,:) = adjnode;
                end
                % if adjnode is in Close remove it
                if isempty(Close)
                    % do nothing
                elseif ismember(adjnode,Close,'rows')
                    [~,rIdx] = ismember(adjnode,Close,'rows');  %若可达临近节点在CloseList中，把该节点从CloseList中删除，这是为什么呢？
                    Close(rIdx,:) = [];
                end
            end
        end
    end
end

function [wknode,Open] = PopOpen(Open,Grids)
    mincost = inf;
    minidx = 1;
    for i = 1:size(Open,1)
        node = Open(i,:);
        tcost = Grids(node(1),node(2),3)+Grids(node(1),node(2),4);
        if tcost < mincost
            minidx = i;
            mincost = tcost;
        end
    end
    wknode = Open(minidx,:);
    Open(minidx,:) = [];  %把第minidx个元素置空，其实也就是把第minidx个元素删除
end

function [minx,miny,obmap] = CalcObstMap(obstlist,gres)
    minx = min(obstlist(:,1));
    maxx = max(obstlist(:,1));
    miny = min(obstlist(:,2));
    maxy = max(obstlist(:,2));
    xwidth = maxx - minx;
    xwidth = ceil(xwidth/gres);  %cei,Y = ceil(X) 将 X 的每个元素四舍五入到大于或等于该元素的最接近整数。
    ywidth = maxy - miny;
    ywidth = ceil(ywidth/gres);  
    obmap = zeros(ywidth,xwidth);  %obmap怎么理解
    for i = 1:ywidth
        for j = 1:xwidth
            ix = minx+(j-1/2)*gres;
            iy = miny+(i-1/2)*gres;
            [~,D] = knnsearch(obstlist,[ix,iy]);  %求最近的障碍物点的距离，也就是距离障碍边沿的距离
            if D < 0.5  %啥意思，当搜索的栅格点离障碍物非常仅时，把该点置1，咋的？碰撞检测吗？
                obmap(i,j) = 1;
            end
        end
    end
end

% function [xidx,yidx] = CalcIdx(x,y,minx,miny,gres)
%     xidx = ceil((x-minx)/gres);
%     yidx = ceil((y-miny)/gres);
% end