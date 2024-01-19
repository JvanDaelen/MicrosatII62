function [] = PlotResults(relative_state_chaser_history)
%PLOTRESULTS Function to plot simulation output
%   Plots are made for 3D-trajectory, deltaV consumption, control force,
%   position, velocity

%% 3D-trajectory plot
figure_name = 'Trajectory';
figure('Name', figure_name)

% Postion variable to plot
X = relative_state_chaser_history(1,:);
Y = relative_state_chaser_history(2,:);
Z = relative_state_chaser_history(3,:);

h = stem3(X, Y, Z);
h.Color = [.7 .7 .7];
h.Marker = 'none';
h.LineStyle = "--";
hold on
grid on
l = plot3(X, Y, Z);
l.Color = 'b';
xlabel("position [m]")
ylabel("position [m]")
zlabel("position [m]")
% legend({ ...
%     'x', ...
%     'y', ...
%     'z', ...
%     'x runs', ...
%     'y runs', ...
%     'z runs' ...
%     },'Location','northeast')
disp([pwd '/Plots/' figure_name '.png'])
saveas(gcf, [pwd '/Plots/' + figure_name + '.png'])












end

% figure_name = "Trajectory"
% figure('Name', figure_name)
% plot(state_timeseries_reference.Time, state_timeseries_reference.Data(:, 1:3))
% grid
% hold on
% plot(t_reconstruct, state_memory(:,1:3), '--')
% xlabel("time [s]")
% ylabel("position [m]")
% legend({ ...
%     'x', ...
%     'y', ...
%     'z', ...
%     'x runs', ...
%     'y runs', ...
%     'z runs' ...
%     },'Location','northeast')
% saveas(gcf, figure_name)