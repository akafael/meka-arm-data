function data = stepjointposition(state,command)
    % Get Peak Time for each joint
    peakPositionValue = max(state.position_j);
    peakTimeStamp = zeros(1,7);
    for i = 1:7
         % Select Only First Ocurrence
         timePeak = state.time(peakPositionValue(1,i) == state.position_j(:,i));
         peakTimeStamp(1,i) = timePeak(1);
    end

    % Get Step Start Time for each joint
    stepPositionValue = max(command.positionControl_j);
    stepTimeStamp = zeros(1,7);
    for i = 1:7
         % Select Only First Ocurrence
         timePeak = command.time(stepPositionValue(1,i) == command.positionControl_j(:,i));
         stepTimeStamp(1,i) = timePeak(1);
    end

    % Get stead state value
    steadValue = zeros(1,7);
    steadTimeStamp = zeros(1,7);
    dtcommand = mean(diff(command.time));
    dtstead = (3/4)*dtcommand;
    for i = 1:7
         % Select Only First Ocurrence
         timePeak = state.time(state.time >= (stepTimeStamp(i) + dtstead));
         steadValue(i) = state.position_j(state.time==timePeak(1),i);
         steadTimeStamp(1,i) = timePeak(1);
    end

    overshotpercent = (peakPositionValue./steadValue).*(peakTimeStamp < steadTimeStamp);
    steaderrorpercent = ((stepPositionValue - steadValue)./stepPositionValue);

    % Group Data for export
    data.peak.time = peakTimeStamp;
    data.peak.value = peakPositionValue;
    data.step.time = stepTimeStamp;
    data.step.value = stepPositionValue;
    data.steadstate.time = steadTimeStamp;
    data.steadstate.value = steadValue;
    data.overshotP = overshotpercent;
    data.errorP = steaderrorpercent;
    data.deadtime = 0.02;

    figure;
    hold on;
    stairs([0 dtcommand 2*dtcommand],[1 0 0]);
    stairs([0 data.deadtime dtcommand+data.deadtime 2*dtcommand] + data.deadtime,[0 1 0 0],'--');
    for  i = 2:7
        timestart = data.step.time(i);
        timeend = timestart + 2*dtcommand;
        plot(state.time((state.time<timeend) & (state.time>=timestart))-timestart, ...
            state.position_j((state.time<timeend) & (state.time>=timestart),i))
    end
    hold off;
    title(['Resposta ao degrau ( Velocity = ',num2str(command.velocityControl*100),'% , Stifness = ',num2str(command.stiffnessControl*100),'% )'])
    legend('Ref','Delay','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6')
end
