function out = readXYZpts(filename, dataLines)
%readXYZpts Import data from a text file
%  out = readXYZpts(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the data as a table.
%
%  out = readXYZpts(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  out = readXYZpts("C:\Users\Francesco\Desktop\belvStereo\GCPs_daModelloDrone.txt", [1, Inf]);

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 4);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["name", "E", "N", "h"];
opts.VariableTypes = ["string", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "name", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "name", "EmptyFieldRule", "auto");

% Import the data
out = readtable(filename, opts);

end