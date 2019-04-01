a = csvread('/home/customrubber/ros_ws/xyzData.csv');
X= a(:,1);
Y = a(:,2);
Z = a(:,3);
stem3(X,Y,Z,'filled')
xlabel('x')
ylabel('y')
zlabel('z')