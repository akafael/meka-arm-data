%% Joint Identification Exp
clc
clear
close all

% Get all jointIdentificationData
location = '../logs/'
bagFileNames = strsplit( ls([location,'jointIdentification_s*.bag']) ,'\n');

% Initialize Storage
numexp = (size(bagFileNames,2)-1);
state = cell(numexp);
command = cell(numexp);
stepdata = cell(numexp);
errortable = cell(numexp);
overshottable = cell(numexp);
steadstatetable = cell(numexp);
peaktable = cell(numexp);

% Latex Table Files
fileerrortable = fopen('jointIdentification_errortable.tex','w');
fileovershottable = fopen('jointIdentification_overshottable.tex','w');
filesteadstatetable = fopen('jointIdentification_steadstatetable.tex','w');
filepeaktable = fopen('jointIdentification_peaktable.tex','w');

for fileIndex = 1:numexp
    % Generate filenames
    dataFile = [bagFileNames{fileIndex}(1:end-3) 'csv'];
    controlFile = [bagFileNames{fileIndex}(1:size(location,2)) 'control_' bagFileNames{fileIndex}((size(location,2)+1):end-3) 'csv'];

    % Extract Data
    [state{fileIndex},command{fileIndex}] = extracthumanoiddata( dataFile, controlFile );

    % Plot Step evaluation and Export data
    data = stepjointposition(state{fileIndex},command{fileIndex});
    axis([0 8 -0.2 1.0]);
    stepdata{fileIndex} = data;

    % Save plot
    expname = ['./',... % Path
               'jointIdentification_exp',num2str(fileIndex),...        % ID Exp
               'v',num2str(command{fileIndex}.velocityControl*100),... % velocity
               'v',num2str(command{fileIndex}.stiffnessControl*100)];  % stifness
    print(expname,'-dpng');

    % Generate latex table
    errorline = [strjoin(arrayfun(@num2str,[data.errorP], 'Uniform', false),' & ') '\\'];
    overshotline = [strjoin(arrayfun(@num2str, data.overshotP, 'Uniform', false),' & ') '\\'];
    steadstateline = [strjoin(arrayfun(@num2str, data.steadstate.value, 'Uniform', false),' & ') '\\'];
    peakline = [strjoin(arrayfun(@num2str, data.peak.value, 'Uniform', false),' & ') '\\'];

    % Print it to a file
    fprintf(fileerrortable,'%.2f & %.2f & %s\n',...
        command{fileIndex}.velocityControl,...
        command{fileIndex}.stiffnessControl,errorline);
    fprintf(fileovershottable,'%.2f & %.2f & %s\n',...
        command{fileIndex}.velocityControl,...
        command{fileIndex}.stiffnessControl,overshotline);
    fprintf(filesteadstatetable,'%.2f & %.2f & %s\n',...
        command{fileIndex}.velocityControl,...
        command{fileIndex}.stiffnessControl,steadstateline);
    fprintf(filepeaktable,'%.2f & %.2f & %s\n',...
        command{fileIndex}.velocityControl,...
        command{fileIndex}.stiffnessControl,peakline);
end
