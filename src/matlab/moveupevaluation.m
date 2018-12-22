%% First Analisis without command ref
clc
clear
close all

% Get all jointIdentificationData
location = '../logs/noControlRef/';
bagFileNames = strsplit( ls([location,'moveUp*.bag']) ,'\n');

% Initialize Storage
numexp = (size(bagFileNames,2)-1);
state = cell(1,numexp);

% Latex Table Files
explabel = 'moveUp';

for fileIndex = 1:numexp
    % Generate filenames
    dataFile = [bagFileNames{fileIndex}(1:end-3) 'csv']
    controlFile = [bagFileNames{fileIndex}(1:size(location,2)) 'control_' bagFileNames{fileIndex}(size(location,2)+1:end-3) 'csv']
    [status,command] = extracthumanoiddata(dataFile,controlFile);

    % Extract Data
    state{fileIndex} = extracthumanoiddata( dataFile );

    % Plot Step evaluation and Export data
    data = controlevaljoint(state{fileIndex} );
    %stepdata{fileIndex} = data;

    % Save plot
    % Save Plot
    plotname = ['./',... % Path
                'moveUp',num2str(fileIndex),...        % ID Exp
                'stateEval',...        % ID Plot
                'v',num2str(command.velocityControl*100),... % velocity
                's',num2str(command.stiffnessControl*100)];  % stifness
    print(plotname,'-dpng');
end
