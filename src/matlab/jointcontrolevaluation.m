%% First Analisis without command ref
clc
clear
close all

% Get all jointIdentificationData
location = '../logs/noControlRef/';
bagFileNames = strsplit( ls([location,'jointIdentification*.bag']) ,'\n');

% Initialize Storage
numexp = (size(bagFileNames,2)-1);
state = cell(1,numexp);
stepdata = cell(1,numexp);
errortable = cell(1,numexp);
overshottable = cell(1,numexp);
steadstatetable = cell(1,numexp);
peaktable = cell(1,numexp);

% Latex Table Files
explabel = 'jointEvaluation';
fileerrortable = fopen([explabel,'_errortable.tex'],'w');
fileovershottable = fopen([explabel,'_overshottable.tex'],'w');
filesteadstatetable = fopen([explabel,'_steadstatetable.tex'],'w');
filepeaktable = fopen([explabel,'_peaktable.tex'],'w');

for fileIndex = 1:numexp
    % Generate filenames
    dataFile = [bagFileNames{fileIndex}(1:end-3) 'csv']

    % Extract Data
    state{fileIndex} = extracthumanoiddata( dataFile );

    % Plot Step evaluation and Export data
    data = controlevaljoint(state{fileIndex} );
    %stepdata{fileIndex} = data;

    % Save plot
    expname = ['./',... % Path
               dataFile,'_exp',num2str(fileIndex),...        % ID Exp
               ];
    print(expname,'-dpng');
end
