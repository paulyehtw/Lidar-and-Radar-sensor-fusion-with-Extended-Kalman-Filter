clear;
clf;
dt = 0.1;
Data = csvread('Radar_Lidar_Data1.csv',1,1);
% Data = csvread('Radar_Lidar_Data2.csv',1,1);
Radar_Measurement = [];
Lidar_Measurement = [];
EKF_Path = [];

F = [[1, 0, dt, 0];
     [0, 1, 0, dt];
     [0, 0, 1, 0];
     [0, 0, 0, 1]];
 
u = 0;
B = [(dt^2)/2 (dt^2)/2 dt dt]';

P = [[1, 0, 0, 0];
     [0, 1, 0, 0];
     [0, 0, 1000, 0];
     [0, 0, 0, 1000]];


R_l = [[0.0025, 0];
       [0, 0.0025]];
  
R_r = [[0.09, 0, 0];
      [0, 0.005, 0];
      [0, 0, 0.09]];
 

Q = [(dt^2)/4 0 (dt^3)/2 0;
     0 (dt^2)/4 0 (dt^3)/2;
     (dt^3/2) 0 (dt^2) 0;
     0 (dt^3)/2 0 (dt^2)];


H = [[1, 0, 0, 0];
     [0, 1, 0, 0]];

I = eye(4);

if (Data(1,1) == 1)
    x = [Data(1,2); Data(1,3); 0; 0];
else
    x = [Data(1,2); Data(1,3); Data(1,4); 0];
end

for n = 1:length(Data)
    
    if (Data(n,1) == 2)
        
        %prediction
        x = F * x + B*u;
        P = F * P * transpose(F) + Q;

        %measurement update
        Z = Data(n,2:4);
        X = Z(1)*cos(Z(2));
        Y = Z(1)*sin(Z(2));
        VX = Z(3)*cos(Z(2));
        VY = Z(3)*sin(Z(2));

        c1 = X^2 + Y^2;
        c2 = sqrt(c1);
        c3 = c1 * c2;
        if (c1==0 || c2==0 || c3==0)
            H_Jac = [[0, 0, 0, 0];
                     [0, 0, 0, 0];
                     [0, 0, 0, 0]];
        else
            H_Jac = [[X/c2, Y/c2, 0, 0];
                    [-Y/c1, X/c1, 0, 0];
                    [(Y*(VX*Y-VY*X))/c3, (X*(X*VY-Y*VX))/c3, X/c2, Y/c2]];
        end
        Z_Car = [X; Y; VX; VY];
        y = transpose(Z) - (H_Jac * Z_Car);
        S = H_Jac * P * transpose(H_Jac) + R_r;
        K = P * transpose(H_Jac) * inv(S);
        x = Z_Car + (K * y);
        P = (I - (K * H_Jac)) * P;
        EKF_Path = [EKF_Path;[x(1),x(2)]];
        Radar_Measurement = [Radar_Measurement; Data(n,2:4)];
    
    else
        
        %prediction
        x = (F * x) + B*u;
        P = F * P * transpose(F) + Q;

        %measurement update
        Z = Data(n,2:3);
        y = transpose(Z) - (H * x);
        S = H * P * transpose(H) + R_l;
        K = P * transpose(H) * inv(S);
        x = x + (K * y);
        P = (I - (K * H)) * P;
        EKF_Path = [EKF_Path;[x(1),x(2)]];
        Lidar_Measurement = [Lidar_Measurement; Data(n,2:3)];
    end
    
end

for i = 1:length(Radar_Measurement)
    Radar_Measurement_Cart(i,:) = [[Radar_Measurement(i,1),0];[0, Radar_Measurement(i,1)]]*[cos(Radar_Measurement(i,2));sin(Radar_Measurement(i,2))];
end

hold on;

plot(Data(:,6),Data(:,7),'linewidth', 2);
scatter(EKF_Path(:,1),EKF_Path(:,2),25,'filled','r');
scatter(Lidar_Measurement(:,1),Lidar_Measurement(:,2),5,'filled','blue');
scatter(Radar_Measurement_Cart(:,1),Radar_Measurement_Cart(:,2),5,'filled','g');

legend('Grundtruth','EKF Path result','Lidar Measurement','Radar Measurement','Location','northwest');
axis square;
hold off;
