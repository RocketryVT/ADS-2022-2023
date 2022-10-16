clc; clear; close all

load("state.mat")
dataSize = 304;

apo = nan(dataSize,1);

deltav = [diff(v); v(dataSize)-v(dataSize-1)];
for i = 1:dataSize
    apo(i) = apoest(h(i),v(i),cd(i));
end

figure; hold on
plot(t,apo);
plot(t,h);

%% apogee determination dynamics
function apogee = apoest(h0,v0,cd)
    v = v0;
    h = h0;
    hprev = h0;
    dt = 1e-2;
    
    g = -9.8;
    rhoG = 1.225;
    rhoH0 = -1.1500e-04*h0 + rhoG;    
    
    while v > 1e-1 && hprev <= h
        hprev = h;
        rhoCur = -1.1500e-04*h + rhoG;
        h = h + v*dt;
        v = v + (g - cd*(rhoCur/rhoH0)*v^2)*dt;
    end
    apogee = h;
    if apogee < 0
        apogee = nan;
    end
end

% function filtered = butterworth2nd(x)
% 	fdata[0] = fdata[1]
% 	fdata[1] = fdata[2]
% 	fdata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * fdata[0]) + (1.68227318476524834168 * fdata[1])
% 	return (fdata[0] + fdata[2]) + 2 * fdata[1]
% end
