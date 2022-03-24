% Brendan Luke
% May 18, 2021
% Launch Sims
clear, clc, format compact
close all
tic
%% Inputs
% constants
G = 6.6743*10^-11; % big G gravitational constant
M = 5.972*10^24; % mass of body (kg)
R0 = 6378137; % radius of body (m)
SH = 8500; % scale height of atmosphere (m)
T0 = 230; % temperature of atmosphere (K)

% LV design
%{
m0S1 = 100000; % first stage total mass (kg)
mpS1 = 80000; % first stage propellant mass (kg)
IspS1 = 300; % specific imuplse of first stage engine (s)
FtS1 = 1600000; % engine thrust of first stage (N)
m0S2 = 14780; % second stage total mass (kg)
mpS2 = 12375; % second stage propellant mass (kg)
IspS2 = 370; % specific imuplse of second stage engine (s)
FtS2 = 155700; % engine thrust of second stage (N)
S = pi*(3.66/2)^2; % vehicle cross-sectional area
Cd = 0.3; % drag coefficient
P_TV = 0.075; % pitch thrust vector, 'Angle of Attack' (deg)
%}

% New Shepard Mod
%
mPay = 25; % payload mass (kg)
mFair = 7.5*(27.7+21.7); % fairing mass (kg)
m0S1 = 59*300+10000+4772/2.205+mPay+mFair; % first stage total mass (kg)
mpS1 = 59*300; % first stage propellant mass (kg)
IspS1 = 370; % specific imuplse of first stage engine (s)
FtS1 = 490000; % engine thrust of first stage (N)
m0S2 = 4772/2.205+mPay; % second stage total mass (kg)
mpS2 = 4431.2/2.205; % second stage propellant mass (kg)
IspS2 = 292.1; % specific imuplse of second stage engine (s)
FtS2 = 15430/2.205*9.81; % engine thrust of second stage (N)
S = pi*(3.5160/2)^2; % vehicle cross-sectional area
Cd = 0.4; % drag coefficient
% P_TV = 0.045; % pitch thrust vector, 'Angle of Attack' (rad)
% t_coast = 120; % coast period (s)
P_TV = 0.030; % pitch thrust vector, 'Angle of Attack' (rad)
t_coast = 100; % coast period (s)
%}

% RocketLab Electron
%{
mPay = 0; % payload mass (kg)
mFair = 50; % fairing mass (kg)
% m0S1 = 59*300+10000+4772/2.205+mPay+mFair; % first stage total mass (kg)
m0S1 = 13000; % first stage total mass (kg)
mpS1 = 10000; % first stage propellant mass (kg)
IspS1 = 311; % specific imuplse of first stage engine (s)
FtS1 = 9*26000; % engine thrust of first stage (N)
m0S2 = 4772/2.205+mPay; % second stage total mass (kg)
mpS2 = 2000; % second stage propellant mass (kg)
IspS2 = 343; % specific imuplse of second stage engine (s)
FtS2 = 26000; % engine thrust of second stage (N)
S = pi*(1.2/2)^2; % vehicle cross-sectional area
Cd = 0.4; % drag coefficient
P_TV = 0.075; % pitch thrust vector, 'Angle of Attack' (deg)
t_coast = 7; % coast period (s)
%}


% sim parameters
dt = 0.1; % time step (s)
n = 3601; % number of points for body fill
h0 = 000; % initial launch site elevation (m)
h_orb = 250; % orbital altitude (km)
maxQ = 20000; % maximum aerodynamic pressure (Pa)

%% Initial Conditions
s(1) = 0; r(1) = R0+h0; FPA(1) = pi/2;
V(1) = 0.01; i = 1; a(1) = 1; TV(1) = pi/2;
m(1) = m0S1; mode(1) = 1; t(1) = 0; q(1) = 0;
DragLoss(1) = 0; GravityLoss(1) = 0; Orbit = [(r(1)-R0)/1000, -R0/1000];
z(1,:) = [s(1), r(1), V(1), FPA(1), m(1)]; SteeringLoss(1) = 0; dV(1) = 0;

%% Run Sim
% Stage 1
while 1
    % Integrate
    z(i+1,:) = z(i,:) + dt*dz(z(i,:),FtS1,a(i),TV(i),IspS1,G,M,S,Cd,R0,SH,T0);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    q(i+1) = z(i+1,3)^2/2*EarthAtmosRho(z(i+1,2)-R0,R0);
    DragLoss(i+1) = DragLoss(i) + dt*(z(i+1,3)^2/2/z(i+1,5)*S*Cd*EarthAtmosRho(z(i+1,2)-R0,R0));
    GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
    SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = IspS1*9.81*log(m0S1/z(i+1,5)); 
    h = cos(z(i+1,4))*z(i+1,2)*z(i+1,3); % orbital angular momentum
    E = z(i+1,3)^2/2 - G*M/(z(i+1,2)); % sepcific orbital energy
    SMA = -G*M/2/E;
    ecc = sqrt(1+2*E*h^2/(G*M)^2); % eccentricity
    Orbit(i+1,:) = [(SMA*(1+ecc)-R0)/1000, (SMA*(1-ecc)-R0)/1000]; % apoapsis height (km), periapsis height (km)   
    
    % control parameters (based on mode)    
    if z(i+1,5) < m0S1-mpS1
        fprintf('stage\n');
        TV(i+1) = 0;
        a(i+1) = 0;
        mode(i+1) = NaN;
%         i = i+1;
        break
    elseif z(i+1,2)-R0 < 2000+h0
        fprintf('vertical ascent, mode 1\n');
        TV(i+1) = pi/2;
        a(i+1) = 1;
        mode(i+1) = 1;
    elseif (z(i+1,4) > pi/6) && (Orbit(i+1,1) < h_orb)
        fprintf('pitch program, mode 3\n');
        TV(i+1) = z(i+1,4) - P_TV;
        a(i+1) = 1;
        mode(i+1) = 3;
    elseif Orbit(i+1,1) < h_orb
        fprintf('prograde, mode 4\n');
        TV(i+1) = z(i+1,4);
        a(i+1) = 1;
        mode(i+1) = 4;
    elseif Orbit(i+1,1) > h_orb
        fprintf('horizontal, mode 5\n');
        TV(i+1) = 0;
        a(i+1) = 1;
        mode(i+1) = 5;
    end

    i = i+1;
end

% Staging changes:
z(i,5) = m0S2;
dV_carry = dV(end);
i_flag = i;
t_flag = t(i);

% Coast
while 1
    % Integrate
    z(i+1,:) = z(i,:) + dt*dz(z(i,:),0,a(i),TV(i),IspS2,G,M,S,Cd,R0,SH,T0);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    q(i+1) = z(i+1,3)^2/2*EarthAtmosRho(z(i+1,2)-R0,R0);
    DragLoss(i+1) = DragLoss(i) + dt*(z(i+1,3)^2/2/z(i+1,5)*S*Cd*EarthAtmosRho(z(i+1,2)-R0,R0));
    GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
    SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS2/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,5));
    h = cos(z(i+1,4))*z(i+1,2)*z(i+1,3); % orbital angular momentum
    E = z(i+1,3)^2/2 - G*M/(z(i+1,2)); % sepcific orbital energy
    SMA = -G*M/2/E;
    ecc = sqrt(1+2*E*h^2/(G*M)^2); % eccentricity
    Orbit(i+1,:) = [(SMA*(1+ecc)-R0)/1000, (SMA*(1-ecc)-R0)/1000]; % apoapsis height (km), periapsis height (km)
    
    % control parameters (based on mode)
    fprintf('coasting\n');
    TV(i+1) = z(i+1,4);
    a(i+1) = 0;
    mode(i+1) = 0;
    
    if i > i_flag + t_coast/dt
        break
    end
    
    i = i+1;
end


% Stage 2
while 2
    % Integrate
    z(i+1,:) = z(i,:) + dt*dz(z(i,:),FtS2,a(i),TV(i),IspS2,G,M,S,Cd,R0,SH,T0);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    q(i+1) = z(i+1,3)^2/2*EarthAtmosRho(z(i+1,2)-R0,R0);
    DragLoss(i+1) = DragLoss(i) + dt*(z(i+1,3)^2/2/z(i+1,5)*S*Cd*EarthAtmosRho(z(i+1,2)-R0,R0));
    GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
    SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS2/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,5));
    h = cos(z(i+1,4))*z(i+1,2)*z(i+1,3); % orbital angular momentum
    E = z(i+1,3)^2/2 - G*M/(z(i+1,2)); % sepcific orbital energy
    SMA = -G*M/2/E;
    ecc = sqrt(1+2*E*h^2/(G*M)^2); % eccentricity
    Orbit(i+1,:) = [(SMA*(1+ecc)-R0)/1000, (SMA*(1-ecc)-R0)/1000]; % apoapsis height (km), periapsis height (km)   
    
    % control parameters (based on mode)    
    if z(i+1,5) < m0S2-mpS2
        fprintf('out of gas\n');
        TV(i+1) = 0;
        a(i+1) = 0;
        mode(i+1) = NaN;
        break
    elseif (abs(Orbit(i+1,1) - h_orb) < 50 || Orbit(i+1,1) > h_orb) && (Orbit(i+1,2) > 185)
        fprintf('orbit achieved\n\n');
        TV(i+1) = TV(i);
        a(i+1) = a(i);
        mode(i+1) = mode(i);
        break
    elseif z(i+1,3)*sin(z(i+1,4)) < 0
        fprintf('maintaining altitude, mode 6\n');
        TV(i+1) = 0.2;
        a(i+1) = 1;
        mode(i+1) = 6;
    else
        fprintf('horizontal, mode 5\n');
        TV(i+1) = 0;
        a(i+1) = 1;
        mode(i+1) = 5;
    end

    i = i+1;
end

%% Plots
% figure('name','Trajectory');
% set(gcf,'WindowState','Maximized');
% plot(t,(r-R0)/1000);
% grid on
% xlabel('Time (s)');
% ylabel('Altitude (km)');

fprintf('Mass remaining: %.0f kg\n',z(end,5)-(m0S2-mpS2));

figure('name','States');
set(gcf,'WindowState','Maximized');
subplot(2,2,1);
plot(t,(z(:,2)-R0)/1000,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Altitude (km)');
hold on
xline(t_flag,'k-','LineWidth',2);
xline(t_flag+t_coast,'k-','LineWidth',2);
subplot(2,2,2);
plot(t,z(:,1)/1000,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Downrange Distance (km)');
hold on
xline(t_flag,'k-','LineWidth',2);
xline(t_flag+t_coast,'k-','LineWidth',2);
subplot(2,2,3);
plot(t,z(:,3),'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Velocity (m/s)');
hold on
xline(t_flag,'k-','LineWidth',2);
xline(t_flag+t_coast,'k-','LineWidth',2);
subplot(2,2,4);
plot(t,z(:,4)*180/pi,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Flight Path Angle (°)');
ylim([0 90]);
hold on
xline(t_flag,'k-','LineWidth',2);
xline(t_flag+t_coast,'k-','LineWidth',2);

figure('name','Trajectory');
set(gcf,'WindowState','Maximized');
plot(t,(z(:,2)-R0)/1000,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Altitude (km)');

figure('name','Energy State');
set(gcf,'WindowState','Maximized');
plot(z(:,3),(z(:,2)-R0)/1000,'LineWidth',2);
grid on
xlabel('Velocity (m/s)');
ylabel('Altitude (km)');

figure('name','Aerodynamic Pressure');
set(gcf,'WindowState','Maximized');
plot(t,q/1000,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Aerodynamic Pressure (kPa)');

figure('name','Orbit Progression');
set(gcf,'WindowState','Maximized');
plot(t,Orbit(:,1),t,Orbit(:,2),'LineWidth',2);
grid on
ylabel('Apsis Height (km)');
xlabel('Time (s)');
legend('Apoapsis','Periapsis','Location','NorthOutside','NumColumns',2);

figure('name','Losses');
set(gcf,'WindowState','Maximized');
plot(t,DragLoss,t,GravityLoss,t,SteeringLoss,t,dV,'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Drag Losses','Gravity Losses','Steering Losses','Delta-V Expended','Location','NorthOutside','NumColumns',4);

figure('name','Control Mode');
set(gcf,'WindowState','Maximized');
plot(t,mode,'LineWidth',2);
xlabel('Time (s)');
ylabel('Control Mode');
yticks([0 1 2 3 4 5 6]);
yticklabels({'Coast','Vertical Ascent','N/A','Pitch Program','Prograde','Horizontal','Maintain Altitude'});
grid on

fprintf('Orbit is: %.0f x %.0f km \n',Orbit(end,1),Orbit(end,2));
fprintf('Losses are: %.1f m/s Drag, %.1f m/s Gravity, %.1f m/s Steering\n',DragLoss(end),GravityLoss(end),SteeringLoss(end));
fprintf('Delta-V Expended: %.0f m/s \n',dV(end));
P_TV*180/pi

%% Stop Clock
toc

function dzdt = dz(z,F,a,TV,Isp,G,M,S,Cd,R0,SH,T0)
% 1 = s, 2 = r, 3 = V, 4 = FPA, 5 = m
aG = G*M/z(2)^2;
aD = EarthAtmosRho(z(2)-R0,R0)*z(3)^2/2*S*Cd/z(5);

dzdt(1) = z(3)*cos(z(4));
dzdt(2) = z(3)*sin(z(4));
if abs(TV-z(4)) < 10^-3
    dzdt(3) = -aG*sin(z(4)) + a*F/z(5)*cos(0) - aD;
    dzdt(4) = -aG/z(3)*cos(z(4)) + z(3)/z(2)*cos(z(4)) + 0*a*F/z(3)/z(5)*sin(TV-z(4));
else
    dzdt(3) = -aG*sin(z(4)) + a*F/z(5)*cos(TV-z(4)) - aD;
    dzdt(4) = -aG/z(3)*cos(z(4)) + z(3)/z(2)*cos(z(4)) + a*F/z(3)/z(5)*sin(TV-z(4));
end
dzdt(5) = -a*F/9.81/Isp;
end
