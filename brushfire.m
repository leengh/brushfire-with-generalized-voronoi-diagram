%% Define Variables
global num_of_rows;
global num_of_cols;
global obstacles;
global obstacles_rows;

num_of_rows = 20;
num_of_cols = 20;

brushfire_grid = zeros(num_of_rows,num_of_cols);

%% Define Obstacles
obstacle1 = [5,6,7,8,9,10,11,12,13,14,15,16;3,3,3,3,3,3,3,3,3,3,3,3];
obstacle2 = [11,12,11,11,11,11,12,12,12,12;7,7,8,9,10,11,8,9,10,11];
obstacle3 = [5,5,5,7,7,7,6,6,6,8;12,13,14,12,13,14,14,12,13,14];
obstacle4 = [12,13,14,12,13,14,11,11;17,17,17,18,18,18,17,18];
obstacle5 = [5,5,6,6,7;7,8,7,8,7];
obstacle6 = [16,16,16,16,16,16,16,16,16,16;7,8,9,10,11,12,13,14,15,16];
% mark edges as obstacles
border1 = [ones(1,num_of_cols); 1:num_of_rows];
border2 = [2:num_of_cols-1;ones(1,num_of_rows-2)];
border3 = [2:num_of_cols-1;ones(1,num_of_rows-2)*num_of_rows];
border4 = [ones(1,num_of_cols)*num_of_cols;1:num_of_rows];

obstacles = [obstacle1 obstacle2 obstacle3 obstacle4 obstacle5 obstacle6 border1 border2 border3 border4];
obstacles_rows = [struct('obstacle',obstacle1);struct('obstacle',obstacle2);struct('obstacle',obstacle3);struct('obstacle',obstacle4);struct('obstacle',obstacle5);struct('obstacle',border1);struct('obstacle',border2);struct('obstacle',border3);struct('obstacle',border4);struct('obstacle',obstacle6)];

% fill all obstacles with 1
for i = 1:size(obstacles,2)
    obstacle = obstacles(:,i);
    brushfire_grid(obstacle(1),obstacle(2)) = 1;
end

close all;

%% Run code
[distanceMap, GVD] = getDistanceMap(brushfire_grid);
brushfire_grid = convertMapToGrid(distanceMap);
% Brushfire with GVD
figure 
plotGrid(brushfire_grid);
title("Brushfire Grid")

% Brushfire with GVD
figure
plotGrid(brushfire_grid);
hold on
plotGVD(GVD)
hold off
title("Brushfire with GVD")

% Path 
figure
plotGrid(brushfire_grid);
hold on
plotGVD(GVD)
hold on
% Path 1
plot(4-0.5,15-0.5,'go');
plot(18-0.5,17-0.5,'g*');
path = calculatePath(brushfire_grid, GVD, [15;4], [17;19]);
plot(path(2,:) - 0.5,path(1,:)-0.5,'g-','LineWidth',2);
hold on
% Path 2
plot(8-0.5,3-0.5,'mo');
plot(10-0.5,13-0.5,'m*');
path = calculatePath(brushfire_grid, GVD, [3;8], [13;10]);
plot(path(2,:) - 0.5,path(1,:)-0.5,'m-','LineWidth',2);
% Path 3
plot(19-0.5,3-0.5,'bo');
plot(13-0.5,8-0.5,'b*');
path = calculatePath(brushfire_grid, GVD, [3;19],[8;13]);
plot(path(2,:) - 0.5,path(1,:)-0.5,'b-','LineWidth',2)
% Path 4
plot(4-0.5,3-0.5,'co');
plot(9-0.5,7-0.5,'c*');
path = calculatePath(brushfire_grid, GVD, [3;4],[7;9]);
plot(path(2,:) - 0.5,path(1,:)-0.5,'c-','LineWidth',2)
hold off
title("Path")

% Plot voronoi without Brushfire
figure
voronoi(obstacles(2,:) - 0.5,obstacles(1,:) - 0.5,'r.-')
title("Voronoi Diagram without Brushfire")

%% Plot Grid
function plotGrid(brushfire_grid)
global num_of_cols;
global num_of_rows;
% Generate grid
plot([0:num_of_rows; 0:num_of_rows], [0:num_of_cols; 0:num_of_cols], 'k')
hax = gca;
hax.XTick = 0:num_of_rows;
hax.YTick = 0:num_of_cols;
hax.XTickLabel = [];
hax.YTickLabel = [];
grid
axis square
% fill grid with values
for iRow = 1:num_of_rows
    for iCol = 1:num_of_cols
        value  = brushfire_grid(iRow ,iCol);
        value = value + 1;
        hold on
        text((iCol - 1)+0.5,(iRow -1)+0.5,num2str(value),'FontWeight','bold');
        if value == 1
            fill([0 1 1 0]+(iCol - 1), [0 0 1 1]+(iRow-1), 'k','facealpha',0.6)
        end
        hold off
    end
end
end

%% Plot GVD from brushfire
function plotGVD(GVD)
for i = 1:size(GVD,2)
point = GVD(:,i);
[manhattan_neighbors, diagonal_neighbors] = get_neighbors(point(1),point(2));
allneighbors = horzcat(manhattan_neighbors, diagonal_neighbors);
for j=1:size(allneighbors,2)
    neighbor = allneighbors(:,j);
    flag = isCellGVD(GVD, neighbor);
    if flag
        plot([point(2) neighbor(2)] - 0.5, [point(1) neighbor(1)] - 0.5,'r.-','linewidth',2);
        hold on
    end
end
end
end

%% Calculations
function [D, GVD] = getDistanceMap(brushfire_grid)
global num_of_rows
global num_of_cols
M = convertGridToMap(brushfire_grid);
% Distance to obstacles - 0 if obstacle otherwise infinity
D = containers.Map(); 
obst = containers.Map();
open = priority_prepare();
GVD = [];
% Initialize data structures
for iCol = 1:num_of_cols
    for iRow = 1:num_of_rows
        key = iRow + "-" + iCol;
        if M(key) == 1
            % obstacle
            D(key) = 0;
            obst(key) = key;
            [open] = priority_insert(open, key, 0);
        else
            % free space
            D(key) = 1/0;
        end
    end
end
% Update Grid
while size(open,1) > 0
    [open,key,~] = priority_minExtract(open);
    coordinates = split(key,'-');
    s_x = str2double(coordinates(1));
    s_y = str2double(coordinates(2));
    obst_s_value = obst(key);
    obst_s = split(obst_s_value,'-');
    obst_s = str2double(obst_s);
    % lower
    [manhattan_neighbors, diagonal_neighbors] = get_neighbors(s_x,s_y);
    allneighbors = horzcat(manhattan_neighbors, diagonal_neighbors);
    for i = 1:size(allneighbors,2)
        neighbor = allneighbors(:,i);
        neighbor_key = neighbor(1) + "-" + neighbor(2);
        d =  int8(norm(obst_s - neighbor));
        if d < D(neighbor_key)
            D(neighbor_key) = d;
            obst(neighbor_key) = obst(key);
            [open] = priority_insert(open, neighbor_key,d);
        elseif d == D(neighbor_key)
            obst_n_value = obst(neighbor_key);
            obst_n = split(obst_n_value,'-');
            obst_n = str2double(obst_n);
            if ~belongs_toSameObstacle(obst_n, obst_s)
                GVD = addGVDCell(GVD, neighbor);
            end
        end
    end
    %     end lower
end



end

function [gridMap] = convertGridToMap(grid)
global num_of_rows;
global num_of_cols;
gridMap = containers.Map();
for iCol = 1:num_of_rows
    for iRow = 1:num_of_cols
        key = iCol + "-" + iRow;
        gridMap(key) = grid(iCol,iRow);
    end
end
end

function [grid] = convertMapToGrid(map)
global num_of_rows;
global num_of_cols;
grid = zeros(num_of_rows,num_of_cols);
for iCol = 1:num_of_rows
    for iRow = 1:num_of_cols
        key = iCol + "-" + iRow;
        grid(iCol,iRow) = map(key);
    end
end
end

function [GVD] = addGVDCell(GVD, point)
for i = 1:size(GVD,2)
    if isequal(point, GVD(:,i))
        return;
    end
end
GVD = [GVD point];
end

function  [flag] = isCellGVD(GVD, point)
for i = 1:size(GVD,2)
    if isequal(point, GVD(:,i))
        flag = true;
        return;
    end
end
flag = false;
end

%% Get manhattan and diagonal neighbors of a grid cell
function [manhattan_neighbors, diagonal_neighbors] = get_neighbors(x,y)
manhattan_neighbors = [];
diagonal_neighbors = [];
global num_of_rows;
global num_of_cols;
max_cols = num_of_cols;
max_rows = num_of_rows;
% Manhattan Neighbors
if (x + 1 <= max_cols)
    manhattan_neighbors = horzcat(manhattan_neighbors,[x + 1; y]);
end
if (x - 1 >= 1)
    manhattan_neighbors = horzcat(manhattan_neighbors,[x - 1; y]);
end
if (y + 1 <= max_rows)
    manhattan_neighbors = horzcat(manhattan_neighbors,[x; y + 1]);
end
if (y - 1 >= 1)
    manhattan_neighbors = horzcat(manhattan_neighbors,[x; y - 1]);
end
% Diagonal Neighbors
if (x + 1 <= max_cols && y + 1 <= max_rows)
    diagonal_neighbors = horzcat(diagonal_neighbors,[x + 1; y + 1]);
end
if (x + 1 <= max_cols && y - 1 >= 1)
    diagonal_neighbors = horzcat(diagonal_neighbors,[x + 1; y - 1]);
end
if (x - 1 >= 1 && y + 1 <= max_rows)
    diagonal_neighbors = horzcat(diagonal_neighbors,[x - 1; y + 1]);
end
if (x - 1 >= 1 && y - 1 >= 1)
    diagonal_neighbors = horzcat(diagonal_neighbors,[x - 1; y - 1]);
end
end

%% Check if two points belong to the same obstacle
function [flag] = belongs_toSameObstacle(point1, point2)
global obstacles_rows;

if isequal(point1, point2)
    flag = true;
    return;
end
flag = false;
for i=1:size(obstacles_rows,1)
  
    obstacle = obstacles_rows(i,1).obstacle;
    flag1 = false;
    flag2 = false;
    for j = 1:size(obstacle,2)
        if isequal(obstacle(:,j), point1)
            flag1 = true;
        end
        if isequal(obstacle(:,j), point2)
            flag2 = true;
        end
    end
    if flag1 && flag2
        flag = true;
        return
    end
end
end

%% Path Planning
function [path] = calculatePath(grid,GVD,start,goal)
up_path = getPathToGVD(grid,GVD,start);
down_path = flip(getPathToGVD(grid,GVD,goal),2);
gvd_path = getGVDPath(grid,up_path(:,end), down_path(:,1));

up_path(:,end) = [];
down_path(:,1) = [];
path = [up_path gvd_path down_path];
end

function [path] = getGVDPath(grid,start, goal)
path = [];
queue = priority_prepare();
queue =  priority_insert(queue,start,0);

previous = containers.Map();
cost = containers.Map();

start_key = start(1)+"-"+start(2);

previous(start_key) = NaN;
cost(start_key) = 0;

while size(queue,1) > 0
[queue,current,~] = priority_minExtract(queue);
current_key = current(1)+"-"+current(2);

if isequal(current,goal)
    path = getFinalPath(previous, goal);
    return
end

[manhattan_neighbors, diagonal_neighbors] = get_neighbors(current(1), current(2));
allneighbors = [manhattan_neighbors diagonal_neighbors];

for i = 1:size(allneighbors,2)
    neighbor = allneighbors(:,i);
    v = grid(neighbor(1),neighbor(2));
    if v == 1
        continue
    end
    neighbor_key = neighbor(1)+"-"+neighbor(2);
    current_cost = cost(current_key) + 1;
    
    flag = false;
    if isKey(cost,neighbor_key)
        if current_cost < cost(neighbor_key)
            flag = true;
        end
    else
    flag = true;
    end

if flag
    previous(neighbor_key) = current;
    cost(neighbor_key) = current_cost;
    [queue] = priority_insert(queue,neighbor,current_cost);
end
end
end
end

function [path] = getFinalPath(previous, goal)
path = [];
path(:,1) = goal;
current = goal;
current_key = current(1)+"-"+current(2);
while isKey(previous,current_key)
    if isnan(previous(current_key))
        break;
    end
    current = previous(current_key);
    current_key = current(1)+"-"+current(2);
    path = [path current];
end
path = [path current];
path = flip(path,2);
end

function [path] = getPathToGVD(grid,GVD,start)
path = [];
path(:,1) = start;

while true
    point = path(:,end);
    if isCellGVD(GVD, point)
         break;
    end
    
    max_neighbor = getMaximumNeighbor(grid, point(1), point(2));
    if isnan(max_neighbor)
        break;
    else
        path =[path max_neighbor];
    end
end
end

function [max_neighbor] = getMaximumNeighbor(grid,x,y)
[manhattan_neighbors, diagonal_neighbors] = get_neighbors(x,y);
max_value = 0;
max_neighbor = NaN;
allneighbors = [manhattan_neighbors diagonal_neighbors];

for i = 1:size(allneighbors,2)
    neighbor = allneighbors(:,i);
    value = grid(neighbor(1), neighbor(2));
    if value == 1
        continue
    end
    if value > max_value
        max_value = value;
        max_neighbor = neighbor;
    end
end
end

%% Queue
function [pQueue]=priority_insert(pQueue,key,distance)
for i=1:size(pQueue,1)
    if pQueue(i).key == key
        cellUpdated = true;
        pQueue(i).distance = distance;
        return;
    end
end
cellUpdated = false;
pQueue=[pQueue;struct('key',key,'distance',distance)];
end

function [pQueue,key,dist]=priority_minExtract(pQueue)
elemNum=numel(pQueue);%get number of records
if elemNum==0%if the given queue is empty, return an empty queue.
    key=[];%set key as empty.
    dist=[];%set cost as empty.
    return;
end
[~,idxMin]=min([pQueue.distance]);
idxMin=idxMin(1);
key=pQueue(idxMin).key;
dist=pQueue(idxMin).distance;
pQueue(idxMin)=[];
if isempty(pQueue)
    pQueue=priority_prepare();
end
end

function [pQueue]=priority_prepare()
pQueue=repmat(struct('key',[],'distance',[]),0,1);
end