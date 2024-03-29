function [] = PlotResults(relative_state_chaser_history, control_force_history, run_ID, time, desired_state_history, m)
%PLOTRESULTS Function to plot simulation output
%   Plots are made for 3D-trajectory, deltaV consumption, control force,
%   position, velocity

mkdir("Plots\" + run_ID)
folder_name = pwd + "\Plots\" + run_ID + "\";

%% 3D-trajectory plot
figure_name = "Trajectory";
figure('Name', figure_name)

% Postion variable to plot
X = relative_state_chaser_history(1,:);
Y = relative_state_chaser_history(2,:);
Z = relative_state_chaser_history(3,:);

h = stem3(X, Y, Z);
h.Color = [.7 .7 .7];
h.Marker = 'none';
h.LineStyle = "-";
hold on
grid on
l = plot3(X, Y, Z);
l.Color = 'b';
xlabel("X pos [m]")
ylabel("Y pos [m]")
zlabel("Z pos [m]")
scatter3([0],[0],[0],'filled', 'r')
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")


%% 2D-trajectory plot
figure_name = "Trajectory2D";
figure('Name', figure_name)

% Postion variable to plot
X = relative_state_chaser_history(1,:);
Y = relative_state_chaser_history(2,:);
Z = relative_state_chaser_history(3,:);

plot(X, Y)
hold on
grid on
xlabel("X pos [m]")
ylabel("Y pos [m]")
scatter([0],[0],'filled', 'r')
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")

%% Plot positions
figure_name = "Positions";
f1 = figure('Name', figure_name);

suplotTitle = {"X", "Y", "Z"}; 
for ii=1:3
    subplot(3,1,ii);
    plot(time, relative_state_chaser_history(ii,:))
    ylabel(suplotTitle{ii} + " pos [m]")
    grid on
    title(suplotTitle{ii})
end
xlabel("time [s]")
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")


%% Plot velocities
figure_name = "Velocities";
f1 = figure('Name', figure_name);

suplotTitle = {'u', 'v', 'w'}; 
for ii=1:3
    subplot(3,1,ii);
    plot(time, relative_state_chaser_history(ii+3,:))
    hold on
    grid on
    plot(time, desired_state_history(ii+3,:))
    ylabel(suplotTitle{ii} + " vel [m/s]")
    title(suplotTitle{ii})
    legend({ ...
        'simulation', ...
        'desired', ...
            },'Location','northeast')
end
xlabel("time [s]")
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")


%% 3D-trajectory plot
figure_name = "Control Forces";
f2 = figure('Name', figure_name);

% Postion variable to plot

suplotTitle = {'Fx', 'Fy', 'Fz'}; 
for ii=1:3
    subplot(3,1,ii);
    plot(time, control_force_history(ii, :))
    ylabel(suplotTitle{ii} + " [N]")
    grid on
    title(suplotTitle{ii})
end
xlabel("time [s]")
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")

%% Plot velocities
figure_name = "deltaV";
f1 = figure('Name', figure_name);

dVx = cumtrapz(abs(control_force_history(1, :))) / m;
dVy = cumtrapz(abs(control_force_history(2, :))) / m;
dVz = cumtrapz(abs(control_force_history(3, :))) / m;
dV = [dVx;dVy;dVz];

suplotTitle = {'dVx', 'dVy', 'dVz'}; 
for ii=1:3
    subplot(3,1,ii);
    plot(time, dV(ii, :))
    hold on
    grid on
    ylabel(suplotTitle{ii} + " [m/s]")
    title(suplotTitle{ii})
end
xlabel("time [s]")
saveas(gcf, folder_name + figure_name + "_" + run_ID + ".png")

end