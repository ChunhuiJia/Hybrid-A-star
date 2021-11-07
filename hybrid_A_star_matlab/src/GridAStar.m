function costmap = GridAStar(obstlist,goal,gres)
    [minx,miny,obmap] = CalcObstMap(obstlist,gres);   % gres����Ԫ��ֱ��ʣ�����������·���ĵ�Ԫ�񳤶��𣿡�/
    col = goal(1);  %Ŀ����x����
    row = goal(2);  %Ŀ����y����
    col = ceil((col-minx)/gres);
    row = ceil((row-miny)/gres);
%     goal = [gx,gy];
%     goal.parent = [gx,gy];
%     goal.precost = 0;
%     goal.postcost = inf;
    goal = [row,col];
    costmap = 0*obmap; %���costmap��ô����أ�����
    dim = size(obmap);     
    for i = 1:dim(1)
        for j = 1:dim(2)
            if obmap(i,j) == 1
                costmap(i,j) = inf;  %�ϰ����Ե����cost��Ϊinf
                continue
            elseif i == col && j == row  %�����ҵ�Ŀ���ʱ����ô��������
                continue
            end            
            start = [i,j]; %start��Ϊʲô�������ģ��������еĿɲ��ҵ㵽Ŀ����AStar������Ϊʲô��ô����
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
            Grids(i,j,1) = i; % ���ڵ��������
            Grids(i,j,2) = j; % ���ڵ��������
            Grids(i,j,3) = norm(([i,j]-goal)); % ����ֵh��ֱ�߾���
            Grids(i,j,4) = inf; % gֵ
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
                    [~,rIdx] = ismember(adjnode,Close,'rows');  %���ɴ��ٽ��ڵ���CloseList�У��Ѹýڵ��CloseList��ɾ��������Ϊʲô�أ�
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
    Open(minidx,:) = [];  %�ѵ�minidx��Ԫ���ÿգ���ʵҲ���ǰѵ�minidx��Ԫ��ɾ��
end

function [minx,miny,obmap] = CalcObstMap(obstlist,gres)
    minx = min(obstlist(:,1));
    maxx = max(obstlist(:,1));
    miny = min(obstlist(:,2));
    maxy = max(obstlist(:,2));
    xwidth = maxx - minx;
    xwidth = ceil(xwidth/gres);  %cei,Y = ceil(X) �� X ��ÿ��Ԫ���������뵽���ڻ���ڸ�Ԫ�ص���ӽ�������
    ywidth = maxy - miny;
    ywidth = ceil(ywidth/gres);  
    obmap = zeros(ywidth,xwidth);  %obmap��ô���
    for i = 1:ywidth
        for j = 1:xwidth
            ix = minx+(j-1/2)*gres;
            iy = miny+(i-1/2)*gres;
            [~,D] = knnsearch(obstlist,[ix,iy]);  %��������ϰ����ľ��룬Ҳ���Ǿ����ϰ����صľ���
            if D < 0.5  %ɶ��˼����������դ������ϰ���ǳ���ʱ���Ѹõ���1��զ�ģ���ײ�����
                obmap(i,j) = 1;
            end
        end
    end
end

% function [xidx,yidx] = CalcIdx(x,y,minx,miny,gres)
%     xidx = ceil((x-minx)/gres);
%     yidx = ceil((y-miny)/gres);
% end