clc;
clear all;
close all;

%% generate path
%{.
dis = 0.7068582;
angle = 6.75;
deltaAngle = angle / 1;
scale = 0.65;

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');

for shift1 = -angle : deltaAngle : angle
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];
    
    pathStartR = 0 : 0.01 : dis;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
    
    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));
    
    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];
    plot3(pathStartX, pathStartY, pathStartZ);
    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;
                          3 * dis, shift3, 0];

                pathR = 0 : 0.01 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];
                
                pathID = pathID + 1;

                %plot3(pathX, pathY, pathZ);
        end
    end
    
    groupID = groupID + 1
end

