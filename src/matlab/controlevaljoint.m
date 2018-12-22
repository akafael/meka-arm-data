function data = controlevaljoint(state,command)
% CONTROLEVALJOINT Evaluate control Response from each joint

% Get Peak Time for each joint
    peakPositionValue = max(state.position_j);
    peakTimeStamp = zeros(1,7);
    for i = 1:7
         % Select Only First Ocurrence
         timePeak = state.time(peakPositionValue(1,i) == state.position_j(:,i));
         peakTimeStamp(1,i) = timePeak(1);
    end

    %% SET POINT vs EFFORT
    figure;
    h(1) = subplot(3,1,1);
    plot(state.time,state.position_j(:,1),state.time,state.position_j(:,2),...
         state.time,state.position_j(:,3),state.time,state.position_j(:,4),...
         state.time,state.position_j(:,5),state.time,state.position_j(:,6),...
         state.time,state.position_j(:,7));
    legend('J_0','J_1','J_2','J_3','J_4','J_5','J_6');
    ylabel('Position');

    h(2) = subplot(3,1,2);
    plot(state.time,state.velocity_j(:,1),state.time,state.velocity_j(:,2),...
         state.time,state.velocity_j(:,3),state.time,state.velocity_j(:,4),...
         state.time,state.velocity_j(:,5),state.time,state.velocity_j(:,6),...
         state.time,state.velocity_j(:,7));
    ylabel('Velocity');

    h(3) = subplot(3,1,3);
    plot(state.time,(state.effort_j(:,1)-mean(state.effort_j(:,1)))/var(state.effort_j(:,1)),...
         state.time,(state.effort_j(:,2)-mean(state.effort_j(:,2)))/var(state.effort_j(:,2)),...
         state.time,(state.effort_j(:,3)-mean(state.effort_j(:,3)))/var(state.effort_j(:,3)),...
         state.time,(state.effort_j(:,4)-mean(state.effort_j(:,4)))/var(state.effort_j(:,4)),...
         state.time,(state.effort_j(:,5)-mean(state.effort_j(:,5)))/var(state.effort_j(:,5)),...
         state.time,(state.effort_j(:,6)-mean(state.effort_j(:,6)))/var(state.effort_j(:,6)),...
         state.time,(state.effort_j(:,7)-mean(state.effort_j(:,7)))/var(state.effort_j(:,7)));
    ylabel('Normalized Effort');

    % Connect Plots
    hpos = get(h,'position'); % Get plot Position
    plotspace=(hpos{2}(2)+hpos{2}(4))-hpos{3}(2); % Find max size
    hpos{3}(4)=plotspace/2;
    hpos{2}(4)=plotspace/2;
    hpos{1}(4)=plotspace/2;
    % Find New Position
    hpos{2}(2) = hpos{3}(2)+plotspace/2;
    hpos{1}(2) = hpos{2}(2)+plotspace/2;
    % Set New position
    set(h(1),'position',hpos{1});
    set(h(2),'position',hpos{2});
    set(h(3),'position',hpos{3});
    % Fix ylabel by removing first and last tick
    set(h(1),'ytick',h(1).YTick(2:end-1));
    set(h(2),'ytick',h(2).YTick(2:end-1));
    set(h(3),'ytick',h(3).YTick(2:end-1));
    % Remove xlabel
    set(h(1),'xticklabel',[]);
    set(h(2),'xticklabel',[]);

    data = [];
end
