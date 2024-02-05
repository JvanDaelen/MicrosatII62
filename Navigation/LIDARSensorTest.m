
ind = 1;
for j = -150:10:0
    true_rel_state = [0.5*j;j;.001;1;1;-1];
    true_state_history(:, j/10+16) = true_rel_state;
    for i = 1:10
        noisy_state = LIDARSensor(true_rel_state);
        relative_state_chaser_history(:, ind) = noisy_state;
        ind = ind + 1;
    end
end

%% 3D-trajectory plot
figure_name = "Trajectory";
figure('Name', figure_name)

% Postion variable to plot
X = relative_state_chaser_history(1,:);
Y = relative_state_chaser_history(2,:);
Z = relative_state_chaser_history(3,:);

scatter3(X,Y,Z)
hold on
grid on
xlabel("X pos [m]")
xlim([-160/2;2])
ylabel("Y pos [m]")
ylim([-160;2])
zlabel("Z pos [m]")
zlim([-2;2])
scatter3(true_state_history(1,:),true_state_history(2,:),true_state_history(3,:),'filled', 'r')
scatter3(0,0,0,'filled', 'k')


% %% 3D-trajectory plot
% figure_name = "Velocity";
% figure('Name', figure_name)
% 
% % Postion variable to plot
% X = relative_state_chaser_history(4,:);
% Y = relative_state_chaser_history(5,:);
% Z = relative_state_chaser_history(6,:);
% 
% scatter3(X,Y,Z)
% hold on
% grid on
% xlabel("X vel [m/s]")
% ylabel("Y vel [m/s]")
% zlabel("Z vel [m/s]")
% scatter3(true_rel_state(4),true_rel_state(5),true_rel_state(6),'filled', 'r')
% scatter3(0,0,0,'filled', 'k')


true_rel_state = [0.001;-15;.001;1;1;-1];
clear relative_state_chaser_history
for i = 1:100
        noisy_state = LIDARSensor(true_rel_state);
        relative_state_chaser_history(:, i) = noisy_state;
end

%% 3D-trajectory plot
figure_name = "StatioKeeping";
figure('Name', figure_name)

% Postion variable to plot
X = relative_state_chaser_history(1,:);
Y = relative_state_chaser_history(2,:);
Z = relative_state_chaser_history(3,:);

scatter(X,Y)
hold on
grid on
xlabel("X pos [m]")
ylabel("Y pos [m]")
zlabel("Z pos [m]")
scatter(true_rel_state(1,:),true_rel_state(2,:),'filled', 'r')