tmp = matlab.desktop.editor.getActive;
cd(fileparts(tmp.Filename));
data = csvread('x64\Debug\data.csv');
plot(data, '.');