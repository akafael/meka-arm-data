function [state,command] = extracthumanoiddata( dataFile, controlFile )
% EXTRACTHUMANOIDDATA Pre process hummanoid_state and hummanoid_command data

    % Read files
    if(nargin > 1)
        dataControl = genericExtractor(controlFile);
        dataSensor = genericExtractor(dataFile);
    elseif(nargin > 0)
        dataSensor = genericExtractor(dataFile);
    else
        error('Invalid Input Argument');
    end

    % Group Data Status
    state.effort_j = [dataSensor.effort_j0 dataSensor.effort_j1 ...
                dataSensor.effort_j2 dataSensor.effort_j3 ...
                dataSensor.effort_j4 dataSensor.effort_j5 ...
                dataSensor.effort_j6];
    state.position_j = [dataSensor.position_j0 dataSensor.position_j1 ...
                    dataSensor.position_j2 dataSensor.position_j3 ...
                    dataSensor.position_j4 dataSensor.position_j5 ...
                    dataSensor.position_j6];
    state.velocity_j = [dataSensor.velocity_j0 dataSensor.velocity_j1 ...
                    dataSensor.velocity_j2 dataSensor.velocity_j3 ...
                    dataSensor.velocity_j4 dataSensor.velocity_j5 ...
                    dataSensor.velocity_j6];
    % Data time
    timeStatus = 10e-10*dataSensor.header_stamp_nsecs+dataSensor.header_stamp_secs;

    % Group Data Control
    if(nargin > 1)
        command.positionControl_j = [dataControl.position_j0 dataControl.position_j1 ...
                        dataControl.position_j2 dataControl.position_j3 ...
                        dataControl.position_j4 dataControl.position_j5 ...
                        dataControl.position_j6];
        command.stiffnessControl = dataControl.stiffness_j1(end);
        command.velocityControl = dataControl.velocity_j1(end);
        % Command time
        timeControl = 10e-10*dataControl.header_stamp_nsecs+dataControl.header_stamp_secs;
        % Find Start Time and change to absolute
        timeStart = min([timeStatus;timeControl]);
        command.time = timeControl - timeStart;
    else
        timeStart = min(timeStatus);
    end

    % Change to Absolute time
    state.time = timeStatus - timeStart;
end
