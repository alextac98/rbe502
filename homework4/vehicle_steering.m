%Steering control
T=50;
x0 =[0;0;0.1]; % Feel free to change the initial state and sampling horizon.
 


% design steering control to follow a straight lane while maintaining a given velocity
% the lateral position yr= ??;



%TODO: param is the additional parameter to pass to the ode function.
[T,X] = ode45(@(t,x) ode_dubins(t,x, param), [0:T], x0, param);

% plot your state trajectories for both 1 and 2, using the following code or else.
figure
plot(T,X(:,1),'LineWidth',4);
xlabel('t');
ylabel('x');

figure
plot(T,X(:,2),'LineWidth',4);
xlabel('t');
ylabel('y');

figure
plot(T,X(:,3),'LineWidth',4);
xlabel('t');
ylabel('theta');

figure
plot(X(:,1), X(:,2))


