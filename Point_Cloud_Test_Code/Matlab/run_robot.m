function run_robot(R)
iterations = 20;
fai1 = acos(75/R);
fai2 = pi/2 - fai1;
circle_center1 = [-225, -R * sin(fai1)];
circle_center2 = [-75, R * sin(fai1)];
circle_center3 = [75, -R * sin(fai1)];
circle_center4 = [225, R * sin(fai1)];
X = zeros(1,2*iterations);
Y = X;

theta = fai2;
for i=1:iterations*2
    Y(i) = R * cos(theta) + circle_center1(2);
    X(i) = -225 - R * sin(theta);
    theta =theta - (fai2/iterations);
end

theta = fai2;
for i=1+iterations*2:iterations*2+iterations*2
    Y(i) = -R * cos(theta) + circle_center2(2);
    X(i) = -75 - R * sin(theta);
    theta =theta - (fai2/iterations);
end

theta = fai2;
for i=1+iterations*4:iterations*2+iterations*4
    Y(i) = R * cos(theta) + circle_center3(2);
    X(i) = 75 - R * sin(theta);
    theta =theta - (fai2/iterations);
end

theta = fai2;
for i=1+iterations*6:iterations*2+iterations*6
    Y(i) = -R * cos(theta) + circle_center4(2);
    X(i) = 225 - R * sin(theta);
    theta =theta - (fai2/iterations);
end





Theta = 0;
X_Rec=[];
Y_Rec=[];
for i=1:length(X)

    X_Rec=[X_Rec X(i)];
    Y_Rec=[Y_Rec Y(i)];
    %Theta = Theta + Theta_local;
    clf;
    
    
    hold on;
    axis([-400 400 -400 400]);

    plot(75,0,'r*');
    circle(75,0,10,2,[1 0 0]); 
    plot(225,0,'r*');
    circle(225,0,10,2,[1 0 0]);
    plot(-225,0,'r*');
    plot(-75,0,'r*');
    circle(-75,0,10,2,[1 0 0]);
    circle(-225,0,10,2,[1 0 0]);

    plot(X_Rec,Y_Rec,'color',[149/255 225/255 235/255],'linewidth',15);
    drawrobot( X(i),Y(i),Theta );
    title('Car trajectory Simulation in Slalom');
    drawnow;
end

end