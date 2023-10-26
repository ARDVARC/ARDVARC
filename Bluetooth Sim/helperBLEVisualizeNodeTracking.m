function helperBLEVisualizeNodeTracking(posLocators,posNode,posActiveLocators,posNodeTrackEst)
%helperBLEVisualizeNodeTracking Generates Bluetooth LE node tracking
%visualization
%   helperBLEVisualizeNodeTracking(POSLOCATORS,POSNODE,POSACTIVELOCATORS,
%   POSNODETRACKEST) plots the positions of locators, actual node, active
%   locators, estimated node and history of node positions.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2D or 3D position of each locator.
%
%   POSNODE is a vector of size 2-by-M or 3-by-M, represents the M node
%   positions in a network. Each column of POSNODE denotes the 2D or 3D
%   position of node.
%
%   POSACTIVELOCATORS is a matrix of size 2-by-P or 3-by-P, represents the
%   position of the P number of active locators in a network. Each column
%   of POSACTIVELOCATORS denotes the 2D or 3D position of each active
%   locator.
%
%   POSNODETRACKEST is a vector of size 2-by-M or 3-by-M, represents the M
%   estimated node positions using linear Kalman filter. Each column of
%   POSNODEEST denotes the 2D or 3D position of estimated node.

%   Copyright 2021 The MathWorks, Inc.

numDimensions = size(posNode,1);
numNodePositions = size(posNode,2);
numLocators = size(posLocators,2);
figure
p2 = uipanel('BackgroundColor',[0.1570 0.1570 0.1570],'ForegroundColor',[1 1 1]);
ax  = axes('Parent',p2,'XLim',[-20 20],'YLim',[-20 20],'ZLim',[-20 20],...
        'NextPlot','add','Color',[0 0 0],'GridColor',0.68*[1 1 1],'GridAlpha',0.4);
grid(ax,'on');
axis(ax,'equal');
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';
ax.XColor = [1 1 1]*0.68;
ax.YColor = [1 1 1]*0.68;
ax.ZColor = [1 1 1]*0.68;
view(ax,3);
ax.Position = [0.11 0.11 0.8 0.8];
view(ax,-65,25);
if numDimensions == 2
    posLocators(3,:) = zeros(1,numLocators);
    posNode(3,:) = zeros(1,numNodePositions);
    posNodeTrackEst(3,:) = zeros(1,numNodePositions);
end
plot3(posLocators(1,:),posLocators(2,:),posLocators(3,:),'^','MarkerSize',8,...
        'Color',[19, 159, 255]/255,'MarkerFaceColor',[19, 159, 255]/255);
locatorPlotter = plot3(ax,nan,nan,nan,'^','MarkerSize',8,'Color',...
                      [255, 255, 17]/255,'MarkerFaceColor',[255, 255, 17]/255);

pLine = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 19, 166]/255,'Marker','o',...
                    'MarkerFaceColor',[255, 19, 166]/255,'MarkerFaceAlpha',1);
hLine = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 19, 166]/255,...
                    'MarkerFaceColor',[255, 19, 166]/255,'MarkerFaceAlpha',...
                    0.08,'MarkerEdgeAlpha',0.08,'Marker','o');
pLineEst = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 105, 41]/255,...
                    'MarkerFaceColor',[255, 105, 41]/255,'MarkerFaceAlpha',...
                    1,'Marker','+','LineWidth',2);
hLineEst = scatter3(ax,nan,nan,nan,'MarkerEdgeColor',[255, 105, 41]/255,...
                    'MarkerFaceColor',[255, 105, 41]/255,'MarkerFaceAlpha',...
                    0.1,'MarkerEdgeAlpha',0.1,'Marker','+','LineWidth',2);

legend('Locator Positions','Active Locator Positions','Current Node Position',...
    'Node Positions History','Current Estimated Node Position','Estimated Node Positions History',...
    'TextColor','white','Location',[0.58 0.64 0.41 0.25]);
title('Bluetooth LE Node Tracking','Color','white');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');

HistoryPositions = {};
HistoryPositionsEst = {};
for j = 1:numNodePositions
    HistoryPositions = plotTrack(posNode(:,j).',pLine,hLine,HistoryPositions);
    HistoryPositionsEst = plotTrack(posNodeTrackEst(:,j).',pLineEst,hLineEst,HistoryPositionsEst);
    pause(0.25)
    if numDimensions == 2
        posActiveLocators{j}(3,:) = zeros(1,length(posActiveLocators{j}(1,:)));
    end
    set(locatorPlotter,'XData',posActiveLocators{j}(1,:),'YData',...
              posActiveLocators{j}(2,:),'ZData',posActiveLocators{j}(3,:));
end

function HistoryPositions = plotTrack(positions,pLine,hLine,HistoryPositions)
    set(pLine,'XData',positions(:,1),'YData',positions(:,2),'ZData',positions(:,3));
    lineData = vertcat(zeros(0,3),HistoryPositions{:});
    set(hLine,'XData',lineData(:,1),'YData',lineData(:,2),'ZData',lineData(:,3));
    HistoryPositions{end+1} = positions;
end

end