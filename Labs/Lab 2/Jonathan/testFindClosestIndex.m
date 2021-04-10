currentOdomIndex = 1;
odomIndexList = [];
trueDepthTimes = dataStore.rsdepth(:,1) - dataStore.rsdepth(:,2); % Recorded time - delay

for i = 1:length(trueDepthTimes) % For the entire length of depth measurement
    while dataStore.odometry(currentOdomIndex,1) < trueDepthTimes(i,1) % Iterate until odom timestamp is bigger than depth timestamp
        currentOdomIndex = currentOdomIndex + 1; % and update the index
    end
    if currentOdomIndex == 1 % If currentOdomIndex-1 == 0, we change it to 1, as ther eis no index 0
        currentOdomIndex = 2;
    end
    odomIndexList = [odomIndexList; currentOdomIndex-1]; % Store the index
end
odomIndexList