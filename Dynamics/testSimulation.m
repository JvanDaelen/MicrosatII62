initialState = [6871000; 0; 0; 0; 7617; 0]; % m and m/s
initialEarthRotation = 0; % rad
dt = 100; % s
iterations = 1000;
t = zeros(1, iterations);
mass = 100; % kg
force = 0; % N

positionHistory = zeros(3, iterations);
positionHistory(:,1) = initialState(1:3);
state = initialState;

earthRotationHistory = zeros(1,iterations);
earthRotationHistory(1,1) = initialEarthRotation;
earthRotation = initialEarthRotation;

accelerationHistory = zeros(3,iterations-1);

ECEFpositionHistory = zeros(3,iterations);
ECEFpositionHistory(:,1) = ECI2ECEF(initialState(1:3), earthRotation);

tic
for i = 2:iterations
    disp(i)
    t(i) = t(i-1) + dt;
    controlForce = force * state(4:6) / norm(state(4:6));
    % [state, acceleration] = propagateSpacecraft_FE_Kepler(state, controlForce, mass, dt);
    % [state, acceleration] = propagateSpacecraft_RK4_Kepler(state, controlForce, mass, dt);
    % [state, acceleration] = propagateSpacecraft_FE_SH(state, controlForce, mass, dt, earthRotation);
    [state, acceleration] = propagateSpacecraft_RK4_SH(state, controlForce, mass, dt, earthRotation);
    positionHistory(:,i) = state(1:3);
    ECEFpositionHistory(:,i) = ECI2ECEF(state(1:3), earthRotation);
    accelerationHistory(:,i-1) = acceleration(1:3);
    earthRotation = propagateEarth(earthRotation, dt);
    earthRotationHistory(1,i) = earthRotation;
end
toc

figure()
plot(t, positionHistory(1,:),'DisplayName','x')
hold on
plot(t, positionHistory(2,:),'DisplayName','y')
plot(t, positionHistory(3,:),'DisplayName','z')
ax = gca;
ax.FontSize = 11;
xlabel('t [s]')
ylabel('Coordinates [m]')
legend
% saveas(gcf,'RK4KepCoords.jpg')
hold off

figure()
plot(t,earthRotationHistory)

figure()
plot(t(1:end-1),accelerationHistory(1,:), 'DisplayName','a_{x}')
hold on
plot(t(1:end-1), accelerationHistory(2,:),'DisplayName','a_{y}')
plot(t(1:end-1), accelerationHistory(3,:),'DisplayName','a_{z}')
plot(t(1:end-1),vecnorm(accelerationHistory),'DisplayName','|a|')
ax = gca;
ax.FontSize = 11;
xlabel('t [s]')
ylabel('Acceleration [m/s^{2}]')
legend
saveas(gcf,'RK4SHAcc.jpg')
hold off

figure()
plot(t, ECEFpositionHistory(1,:),'DisplayName','x_{ECEF}')
hold on
plot(t, ECEFpositionHistory(2,:),'DisplayName','y_{ECEF}')
plot(t, ECEFpositionHistory(3,:),'DisplayName','z_{ECEF}')
legend
hold off
