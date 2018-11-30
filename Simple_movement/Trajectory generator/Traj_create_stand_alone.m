function [Theta, X, Y] = Traj_create_stand_alone(Theta,Axis_input,N_input, Name)
%% Trj_create_stand_alone(Theta,Axis_input,N_input, Name)
% Theta = desired phase variable range
% Axis_input = [-x_limit x_limit -y_limit y_limit], It provides the box in which
% the user draws the trajectory. This is independent of the center of set
% of the actual trajectory.
% N_input = Numper of points to be provide by the user
% Name = 'Trot_1', Name of the trajectory
% Example:
% [Theta, X, Y] = Traj_create_stand_alone(linspace(-pi,3*pi,301)',[-0.015 0.015 -0.025 0.025], 25, 'Trot_1');
% _____________________________________________________________________________________________________________
clc
%% Trajectory modification

x= zeros(N_input,1); y=x;

opengl hardware
figure
axis(Axis_input)

for i =1:N_input
    if i>1
        hold on
        plot(linspace(-10,10,21), mean(Axis_input(3:4))*ones(1,21),'r', 'linewidth', 3)
        plot(x(1:i-1),y(1:i-1),'b-s','linewidth',3,'MarkerSize',15);
        for j= 1:i-1
            text(x(j)+0.0025,y(j)+0.0025,[num2str(j) '/' num2str(N_input)])
        end
        xlabel('X')
        ylabel('Y(m)')
        hold off
        axis(Axis_input)
    end
    grid on
    [x(i), y(i)] = ginput(1);
    
end
hold on
plot(x(1:i),y(1:i),'b-s','linewidth',3,'MarkerSize',15);
for j= 1:i
    text(x(j)+0.0025,y(j)+0.0025,[num2str(j) '/' num2str(N_input)])
end
x_center = 0; y_center = 0;
plot(x_center, y_center,'k-x','linewidth', 3, 'MarkerSize',10);

% phase calculation
theta_temp = zeros(length(y),1);
for i = 1:length(y)
    if y< 0; phase_off = 0;
    else; phase_off = 0;
    end
    theta_temp(i) = atan2(y(i) - y_center, x(i)- x_center) + phase_off;
end
theta = [theta_temp - pi; theta_temp + pi; theta_temp + 3*pi];

X = interp1(theta, [x;x;x], Theta);
Y = interp1(theta, [y;y;y], Theta);

hold on
plot(X,Y,'k','linewidth',2);
hold off

x_off = 0; y_off = -0.175; 
% Off sets are provided to the center of rotation.
X = X + x_off;
Y = Y + y_off;

Xm = circshift(X, floor(length(X)/2));
Ym = circshift(X, floor(length(Y)/2));
data_set = [Theta, X, Y, Xm, Ym, X, Y, Xm, Ym];
% The "data_set" contains the phase(Theta), X(x- axis motion), Y(Y-axis
% motion), Xm (Shifted x-axis), Ym (shifted Y-axis motion)

figure
plot(Theta*180/pi, data_set(:,2:3),'o')
grid on

figure
plot(theta_temp*180/pi, [x, y],'o')
grid on

%% Saving data
save(['end_effector_data_' Name '.txt'],'data_set','-ascii')