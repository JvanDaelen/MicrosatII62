initialState = [6871000; 0; 0; 0; 7617; 0];
dt = 100;
iterations = 1000;
t = zeros(1, iterations);

positionHistory = zeros(3, iterations);
positionHistory(:,1) = initialState(1:3);
state = initialState;

for i = 2:iterations
    t(i) = t(i-1) + dt;
    state = propagateSpacecraftRK4(state, dt);
    positionHistory(:,i) = state(1:3);
end

plot(t, positionHistory(1,:),'DisplayName','x')
hold on
plot(t, positionHistory(2,:),'DisplayName','y')
plot(t, positionHistory(3,:),'DisplayName','z')
legend
hold off