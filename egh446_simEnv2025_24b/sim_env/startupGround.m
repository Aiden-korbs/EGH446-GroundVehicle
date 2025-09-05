%% startupGround.m — run model, read Kalman xhat -> X/Y, compute & plot RMSE

close all; clear; clc;
rng(3);

%% Waypoints
unordered_waypoints.x = randi([-500 500],1,10);
unordered_waypoints.y = randi([-500 500],1,10);
[waypoints.x, waypoints.y] = order_waypoints(unordered_waypoints.x, unordered_waypoints.y);
assignin('base','waypoints',waypoints);     % if the model reads from base

%% (optional) toolboxes
homedir = pwd;
addpath(genpath(fullfile(homedir,'toolboxes')));
tbxStarter = fullfile(homedir,'toolboxes','MRTB','startMobileRoboticsSimulationToolbox.m');
if exist(tbxStarter,'file'), try, run(tbxStarter); end, end

%% Find & load the model
cands = {'sl_groundvehicleDynamics23.slx','sl_groundvehicleDynamics23b.slx','sl_groundvehicleDynamics23.slx'};
mdlFile = '';
for k = 1:numel(cands)
    f = which(cands{k});
    if ~isempty(f), mdlFile = f; break; end
end
assert(~isempty(mdlFile),'Could not find sl_groundvehicleDynamics(.slx) on the MATLAB path.');
[~, mdlName] = fileparts(mdlFile);
if bdIsLoaded(mdlName), close_system(mdlName,0); end
load_system(mdlFile);

%% Run the model
simOut = sim(mdlName,'ReturnWorkspaceOutputs','on'); %#ok<NASGU>

%% Read xhat (Timeseries) written by the To Workspace block
xh = [];
% prefer Single Simulation Output if present
if exist('simOut','var') && ismethod(simOut,'who') && ismember('xhat',simOut.who)
    xh = simOut.get('xhat');
elseif evalin('base','exist(''xhat'',''var'')==1')
    xh = evalin('base','xhat');
end
assert(~isempty(xh), ['No variable ''xhat'' found. Put a To Workspace on the Kalman output:', ...
                      ' name=xhat, format=Timeseries.']);

if isa(xh,'timeseries')
    t  = xh.Time(:);
    D  = xh.Data;
else
    D  = xh;
    t  = (0:size(D,1)-1)';   % fallback time
end

% >>> change these indices if your X/Y aren’t columns 1 and 2 of xhat <<<
ix = 1;  iy = 2;
x_pos = D(:,ix);
y_pos = D(:,iy);

%% Compute RMSE
wx = waypoints.x(:).'; wy = waypoints.y(:).';
[rmse_cte, cte] = calculateCrossTrackRMSE(x_pos, y_pos, wx, wy);
fprintf('Cross-track RMSE: %.3f m (N=%d samples)\n', rmse_cte, numel(cte));

% Align lengths
n = min(numel(t), numel(cte)); t = t(1:n); cte = cte(1:n);

%% Plots
figure('Name','Path vs Waypoints');
plot(wx, wy, 'o-','DisplayName','Waypoints'); hold on;
plot(x_pos, y_pos, '.-','DisplayName','Vehicle path');
axis equal; grid on; legend('Location','best');
title(sprintf('Path vs Waypoints (RMSE = %.3f m)', rmse_cte));
xlabel('X (m)'); ylabel('Y (m)');

figure('Name','Cross-track error vs time');
plot(t, cte, 'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Cross-track error (m)');
title(sprintf('CTE over time (RMSE = %.3f m)', rmse_cte));

figure('Name','Cumulative RMSE vs time');
cumRMSE = sqrt(cumsum(cte.^2) ./ (1:n)');
plot(t, cumRMSE, 'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Cumulative RMSE (m)');
title('Cumulative RMSE vs time');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------------------- Local functions --------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ordered_wp_x, ordered_wp_y] = order_waypoints(wp_x, wp_y)
    % Nearest-neighbour + 2-opt ordering
    pts = [wp_x(:), wp_y(:)];
    n = size(pts,1);
    if n<=1, ordered_wp_x=pts(:,1); ordered_wp_y=pts(:,2); return; end
    remaining = 1:n; route = zeros(1,n);
    route(1) = remaining(1); remaining(1) = [];
    for i = 1:n-1
        curr = pts(route(i),:);
        [~,kmin] = min(sum((pts(remaining,:)-curr).^2,2));
        route(i+1) = remaining(kmin);
        remaining(kmin) = [];
    end
    improved=true; tol=1e-12; dist=@(a,b)pdist2(pts(a,:),pts(b,:));
    while improved
        improved=false;
        for i=1:n-3
            a=route(i); b=route(i+1);
            for j=i+2:n-1
                c=route(j); d=route(j+1);
                if (dist(a,c)+dist(b,d)) < (dist(a,b)+dist(c,d)) - tol
                    route(i+1:j) = route(j:-1:i+1); improved=true;
                end
            end
        end
    end
    ordered_wp_x = pts(route,1); ordered_wp_y = pts(route,2);
end

function [rmse, cte] = calculateCrossTrackRMSE(vehicle_x, vehicle_y, waypoints_x, waypoints_y)
    vx=vehicle_x(:); vy=vehicle_y(:);
    wx=waypoints_x(:); wy=waypoints_y(:);
    assert(numel(wx)>=2,'Waypoints must contain at least two points.');
    x1=wx(1:end-1); y1=wy(1:end-1); x2=wx(2:end); y2=wy(2:end);
    dx=x2-x1; dy=y2-y1; L2=dx.^2+dy.^2;
    n=numel(vx); cte=zeros(n,1);
    for k=1:n
        px=vx(k); py=vy(k);
        t=((px-x1).*dx+(py-y1).*dy)./max(L2,eps);
        t=max(0,min(1,t));
        cx=x1+t.*dx; cy=y1+t.*dy;
        cte(k)=min(hypot(px-cx,py-cy));
    end
    rmse=sqrt(mean(cte.^2));
end
