    %% train_ann_full_replace_v7.m
% Full ANN replacement of Iq PI controller
% Input : I_D_MEAS, I_Q_MEAS, I_Q_REF, ENCODER_SPEED
% Output: V_Q
% Architecture: 4 -> [10 tansig] -> [8 tansig] -> 1 purelin
clear; clc;

%% ---- STEP 1: Load log data (matches MotorPilot export format) ----
LOG_FILE = 'LOG0-100%_4.csv';   % <-- update to your file

% MotorPilot CSV has a variable-length metadata block (device info,
% FOC rate, PWM frequency, etc.) before the actual header row, uses
% CRLF line endings, and is ISO-8859 encoded (not plain UTF-8) -- so
% readtable() with default options will fail or mis-parse. Find the
% real header row automatically instead of hardcoding a line number.
fid = fopen(LOG_FILE, 'r', 'n', 'ISO-8859-1');
assert(fid ~= -1, 'Could not open %s', LOG_FILE);

headerLineNum = 0;
headerLine = '';
lineNum = 0;
while ~feof(fid)
    lineNum = lineNum + 1;
    tline = fgetl(fid);
    if ischar(tline) && startsWith(strtrim(tline), 'TIMESTAMPS')
        headerLineNum = lineNum;
        headerLine = tline;
        break;
    end
end
fclose(fid);
assert(headerLineNum > 0, 'Could not find TIMESTAMPS header row in %s', LOG_FILE);

fprintf('Found header on line %d: %s\n', headerLineNum, strtrim(headerLine));

opts = detectImportOptions(LOG_FILE, 'NumHeaderLines', headerLineNum - 1, ...
    'Encoding', 'ISO-8859-1');
T = readtable(LOG_FILE, opts);
T.Properties.VariableNames = strtrim(T.Properties.VariableNames);

fprintf('Columns found: %s\n', strjoin(T.Properties.VariableNames, ', '));

%% ---- Column names as actually logged (NO TIMESTAMPS as input) ----
INPUT_NAMES = {'I_D_MEAS','I_Q_MEAS','I_Q_REF','ENCODER_SPEED'};
TARGET_NAME = 'V_Q';

for i = 1:numel(INPUT_NAMES)
    if ~any(strcmp(T.Properties.VariableNames, INPUT_NAMES{i}))
        error(['Missing input column "' INPUT_NAMES{i} '". ' ...
               'Columns present: ' strjoin(T.Properties.VariableNames, ', ')]);
    end
end

if ~any(strcmp(T.Properties.VariableNames, TARGET_NAME))
    error(['Log file has no "%s" column. Columns present: %s\n' ...
           'MotorPilot must have V_Q (Vqd.q) enabled in the ' ...
           'High-Frequency signal list before logging -- re-log ' ...
           'with V_Q added, then re-run this script.'], ...
           TARGET_NAME, strjoin(T.Properties.VariableNames, ', '));
end

X_raw = T{:, INPUT_NAMES}';   % 4 x N
Y_raw = T{:, TARGET_NAME}';   % 1 x N

%% ---- STEP 2: Clean data ----
valid = all(isfinite(X_raw),1) & isfinite(Y_raw);
X_raw = X_raw(:, valid);
Y_raw = Y_raw(:, valid);

% remove extreme outliers (>5 std from mean) per column
for i = 1:size(X_raw,1)
    mu = mean(X_raw(i,:)); sd = std(X_raw(i,:));
    keep = abs(X_raw(i,:) - mu) < 5*sd;
    X_raw = X_raw(:, keep);
    Y_raw = Y_raw(:, keep);
end

fprintf('Samples after cleaning: %d\n', size(X_raw,2));

%% ---- STEP 3: Train/Val/Test split & network ----
net = fitnet([10 8], 'trainlm');
net.divideParam.trainRatio = 0.70;
net.divideParam.valRatio   = 0.15;
net.divideParam.testRatio  = 0.15;

net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'tansig';
net.layers{3}.transferFcn = 'purelin';

net.trainParam.epochs = 500;
net.trainParam.max_fail = 20;

[net, tr] = train(net, X_raw, Y_raw);

%% ---- STEP 4: Evaluate ----
Y_pred = net(X_raw);

testX = X_raw(:, tr.testInd);
testY = Y_raw(:, tr.testInd);
testPred = net(testX);

R = corrcoef(testPred, testY);
fprintf('Test R = %.5f\n', R(1,2));

rmse_overall = calcRMSE(Y_pred, Y_raw);
rmse_test    = calcRMSE(testPred, testY);
fprintf('Overall RMSE = %.5f V\n', rmse_overall);
fprintf('Test RMSE    = %.5f V\n', rmse_test);

%% ---- STEP 5: Graphs ----
% Graph 1: MATLAB regression plot (Output vs Target, R-value)
%          -- ตรงกับ ANN-FF regression figure
plotregression(testY, testPred, 'ANN-FF (Motor Pilot log, steady-state)');

% Graph 2: MATLAB training performance curve (Train/Val/Test MSE vs epoch)
%          -- ตรงกับ Best Validation Performance figure
plotperform(tr);

% Graph 3: Predicted vs Actual (test set) -- scatter with y=x reference
figure('Name', 'ANN Prediction vs Actual');
scatter(testY, testPred, 8, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
lims = [min([testY testPred]), max([testY testPred])];
plot(lims, lims, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Actual V_Q (V)');
ylabel('Predicted V_Q (V)');
title(sprintf('Test Set: R=%.4f, RMSE=%.4f V', R(1,2), rmse_test));
grid on;
axis equal;

% Graph 4: Residual (error) distribution
residuals = testPred - testY;
figure('Name', 'ANN Prediction Residuals');
histogram(residuals, 50);
xlabel('Prediction error, V_{Q,pred} - V_{Q,actual} (V)');
ylabel('Count');
title(sprintf('Residual Distribution (mean=%.4f, std=%.4f)', ...
    mean(residuals), std(residuals)));
grid on;

% Graph 2 (new): Residual (error) distribution -- shows where the ANN
% is biased or has high-variance error, useful for spotting
% extrapolation problems in specific operating regions
residuals = testPred - testY;
figure('Name', 'ANN Prediction Residuals');
histogram(residuals, 50);
xlabel('Prediction error, V_{Q,pred} - V_{Q,actual} (V)');
ylabel('Count');
title(sprintf('Residual Distribution (mean=%.4f, std=%.4f)', ...
    mean(residuals), std(residuals)));
grid on;

%% ---- STEP 6: Extract normalization + weights ----
xmin = net.inputs{1}.processSettings{1}.xmin;
xmax = net.inputs{1}.processSettings{1}.xmax;
xgain = 2 ./ (xmax - xmin);
xoffset = xmin;

ymin = net.outputs{3}.processSettings{1}.xmin;
ymax = net.outputs{3}.processSettings{1}.xmax;
ygain = 2 ./ (ymax - ymin);
yoffset = ymin;

IW1 = net.IW{1};   % 10 x 4
b1  = net.b{1};    % 10 x 1
LW2 = net.LW{2,1}; % 8 x 10
b2  = net.b{2};    % 8 x 1
LW3 = net.LW{3,2}; % 1 x 8
b3  = net.b{3};    % 1 x 1

%% ---- STEP 7: Export to C header ----
fid = fopen('ann_foc_full_weights.h', 'w');
fprintf(fid, '#ifndef ANN_FOC_FULL_WEIGHTS_H\n#define ANN_FOC_FULL_WEIGHTS_H\n\n');
fprintf(fid, '#define ANNFULL_N_INPUTS   4\n');
fprintf(fid, '#define ANNFULL_N_HIDDEN1  10\n');
fprintf(fid, '#define ANNFULL_N_HIDDEN2  8\n');
fprintf(fid, '#define ANNFULL_N_OUTPUTS  1\n\n');

writeArr = @(name, M) fprintf(fid, 'static const float %s[%d] = {%s};\n\n', ...
    name, numel(M), strjoin(compose('%.8ff', M(:)'), ', '));

writeArr('ANNFULL_X_GAIN',   xgain);
writeArr('ANNFULL_X_OFFSET', xoffset);
fprintf(fid, 'static const float ANNFULL_Y_GAIN = %.8ff;\n', ygain);
fprintf(fid, 'static const float ANNFULL_Y_OFFSET = %.8ff;\n\n', yoffset);

writeArr('ANNFULL_IW1', IW1');   % row-major flatten: [hidden][input]
writeArr('ANNFULL_B1',  b1);
writeArr('ANNFULL_LW2', LW2');
writeArr('ANNFULL_B2',  b2);
writeArr('ANNFULL_LW3', LW3');
writeArr('ANNFULL_B3',  b3);

fprintf(fid, '#endif // ANN_FOC_FULL_WEIGHTS_H\n');
fclose(fid);

fprintf('Exported ann_foc_full_weights.h\n');

%% ---- Local function: RMSE ----
function r = calcRMSE(predicted, actual)
    % Root Mean Square Error between predicted and actual vectors.
    % Both inputs must be the same size (row or column vectors).
    assert(isequal(size(predicted), size(actual)), ...
        'calcRMSE: predicted and actual must be the same size');
    r = sqrt(mean((predicted - actual).^2));
end
