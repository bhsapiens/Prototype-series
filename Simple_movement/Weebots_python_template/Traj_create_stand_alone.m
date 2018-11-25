function [theta_temp, X, Y] = Traj_create_stand_alone(Theta,Axis_input,N_input, Name)

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
    if y< 0; phase_off = 2*pi;
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

%% Saving data
fid=fopen(['end_effector_data_' Name '.txt'],'w');
fprintf(fid, '%f \n', [Theta, X, Y]);
fclose(fid);
