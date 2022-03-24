% Brendan Luke
% November 12, 2021
% Launch Sims 3D
clear, clc, format compact
close all
tic
%% Inputs
% input values: CONSTANT
R0 = 6378137; % radius of Earth (m)
G = 6.6743*10^-11; % gravitational constant (N*m^2/kg^2)
M = 5.97237*10^24; % mass of Earth (kg)
wEarth = 72.92115*10^-6; % angular velocity of Earth rotation (rad/s)
f = 1/298.257223563; % WGS-84 flattenig ellipsoid
EarthImg = imread('eo_base_2020_clean_3600x1800.png');
% EarthImg = imread('eo_base_2020_clean_720x360.jpg');
EarthImg = flip(EarthImg,1); % rectify image

% sim parameters
dt = 0.1; % time step (s)
LaunchSite = [28.6, -80.7]; % lat, lon coords of launch site (deg)
h_orb = 225; % orbital altitude target (km)
smaTol = 10; % +/- tolerance on semi-major axis (km)
i_orb = 50;%51.6;%LaunchSite(1); % target orbital inclination (°) 82 from wallops is good initial, 70 from Cape
% 96.2 for direct Cape, 96.0 for direct Wallops
launchSouth = false;%true; % direction of launching azimuth, 'North' (false) or 'South' (true)
inertialBlend = 4; % degree to raise pressure to for transitioning guidance from surface reltive to inertial
postInsertion_Coast = 100; % length of time to continue sim post insertion (s)

% make setup plot
figure('name','Launch Site');
set(gcf,'WindowState','maximized');
[Xs,Ys,Zs] = ellipsoid(0,0,0,R0,R0,R0*(1-f),360);
surf(Xs,Ys,Zs);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',EarthImg,'EdgeColor',[0 0 0],'EdgeAlpha',0)
hold on
plot3(R0*cosd(LaunchSite(1))*cosd(LaunchSite(2)),R0*cosd(LaunchSite(1))*sind(LaunchSite(2)),R0*sind(LaunchSite(1)),...
    'r.','MarkerSize',20);
text(R0*cosd(LaunchSite(1))*cosd(LaunchSite(2)),R0*cosd(LaunchSite(1))*sind(LaunchSite(2)),R0*sind(LaunchSite(1)),...
    ['Launch Site ',num2str(LaunchSite(1),3),'°N, ',num2str(LaunchSite(2),3),'°E']);
grid on
axis equal
axis off
set(gca,'clipping','off');

%% Launch Vehicle Design
% New Shepard Mod
%{
mPay = 50; % payload mass (kg)
mFair = 7.5*(27.7+21.7); % fairing mass (kg)
m0S1 = 59*300+10000+4772/2.205+mPay+mFair; % first stage total mass (kg)
mpS1 = 59*300; % first stage propellant mass (kg)
IspS1 = 350; % specific imuplse of first stage engine (s)
FtS1 = 490000; % engine thrust of first stage (N)
m0S2 = 4772/2.205+mPay; % second stage total mass (kg)
mpS2 = 4431.2/2.205; % second stage propellant mass (kg)
IspS2 = 292.1; % specific imuplse of second stage engine (s)
FtS2 = 15430/2.205*9.81; % engine thrust of second stage (N)
S = pi*(3.5160/2)^2; % vehicle cross-sectional area
P_TV = 0.030; % pitch thrust vector, 'Angle of Attack' (rad)
t_coast = 115; % coast period (s)
%}

% RocketLab Electron
%{
mPay = 200; % payload mass (kg)
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
P_TV = 0.095; % pitch thrust vector, 'Angle of Attack' (deg)
t_coast = 7; % coast period (s)
h_vert = 2000; % initial vertical trajectory height (m)
%}

% RocketLab Neutron
%{
mPay = 8000; % payload mass (kg)
mFair = 7.5*(38.2+38.1); % fairing mass (kg), cyl + cone
mpS2 = 800*4.5^2/4*pi*(5.153); % second stage propellant mass (kg)
m0S2 = 1800+mpS2+mPay; % second stage total mass (kg)
mpS1 = 800*4.5^2/4*pi*(22.9); % first stage propellant mass (kg)
m0S1 = 6000+mpS1+m0S2+mFair; % first stage total mass (kg)
IspS1 = 311; % specific imuplse of first stage engine (s)
FtS1 = 1.2*m0S1*9.81; % engine thrust (total) of first stage (N)
IspS2 = 343; % specific imuplse of second stage engine (s)
FtS2 = FtS1/5;%0.5*m0S2*9.81; % engine thrust of second stage (N)
S = pi*(4.5/2)^2; % vehicle cross-sectional area
P_TV = 0.0107; % pitch thrust vector, 'Angle of Attack' (rad); 500km 97.4 deg go South from 38.5 N
% P_TV = 0.0152; % pitch thrust vector, 'Angle of Attack' (rad); 500km 97.4 deg go South from 28.5 N
t_coast = 7; % coast period (s)
h_vert = 2000; % initial vertical trajectory height
%}

% Starship - Old
%{
mPay = 0; % payload mass (kg)
mpS1 = 1000*1200; % first stage propellant mass (kg)
m0S1 = 1000*100+mpS1+mPay; % first stage total mass (kg)
mpS2 = 1e-5; % second stage propellant mass (kg)
m0S2 = (m0S1-mpS1)+mpS2+mPay; % second stage total mass (kg)
IspS1 = 380; % specific impulse of first stage engine (s)
FtS1 = 6*2.23e6; % engine thrust (total) of first stage (N)
IspS2 = 1e-5; % specific impulse of second stage engine (s)
FtS2 = 1e-5; % engine thrust of second stage (N)
S = pi*(9/2)^2; % vehicle cross-sectional area
P_TV = 0.0069; % pitch thrust vector, 'Angle of Attack' (rad)
t_coast = 7; % coast period (s)
h_vert = 2000; % initial vertical trajectory height (m)
%}

% KSP LV-1
%{
mPay = 200; % payload mass (kg)
mFair = 6753-6467; % fairing mass (kg)
m0S1 = 36831; % total liftoff mass (kg)
mpS1 = m0S1-8019; % first stage propellant mass (kg)
IspS1 = 300; % specific imuplse of first stage engine (s)
FtS1 = 2*530000*0.535; % engine thrust of first stage (N)
mpS2 = 6753-1177; % second stage propellant mass (kg)
m0S2 = 891+mpS2+mPay; % second stage total mass (kg)
IspS2 = 343.9; % specific imuplse of second stage engine (s)
FtS2 = 63500; % engine thrust of second stage (N)
S = pi*(1.875/2)^2; % vehicle cross-sectional area
P_TV = 0.040; % pitch thrust vector, 'Angle of Attack' (deg)
t_coast = 5; % coast period (s)
h_vert = 1000; % initial vertical trajectory height (m)
%}

% SpaceX Falcon 9 Crew Dragon
%
mPay = 12700; % payload mass (kg)
mFair = 0; % fairing mass (kg)
mpS2 = 1000*(75.2+2.3); % second stage propellant mass (kg)
m0S2 = 4000+mpS2+mPay; % second stage total mass (kg)
IspS2 = 348; % specific imuplse of second stage engine (s)
FtS2 = 981000; % engine thrust of second stage (N)
mpS1 = 1000*(287.4+123.5); % first stage propellant mass (kg)
m0S1 = mpS1+m0S2+22200; % total liftoff mass (kg)
IspS1 = [282 311]; % specific imuplse of first stage engine (s)
FtS1 = 9*1000*[854 981]; % engine thrust of first stage (N)
S = pi*(4/2)^2; % vehicle cross-sectional area
P_TV = 0.060; % pitch thrust vector, 'Angle of Attack' (deg)
t_coast = 10; % coast period (s)
h_vert = 1000; % initial vertical trajectory height (m)
%}

% troubleshooting
%{
TWR = FtS1/(9.81*m0S1)
t_S2 = mpS2/(FtS2/(9.81*IspS2)) % time of pseudo 2nd stage burn
if t_S2 < dt
    dV_add = dt*(FtS2/m0S2)
else
    dV_add = t_S2*(FtS2/m0S2)
end
%}

% plot(180/pi*diff(alt)/dt)
% grid on
% ylim([-5 5]);
% xlim([0 1800]);

%% Initial conditions
dumbStool = 0;
i = 1; a(1) = 1; dV(1) = 0; altitude(1) = dumbStool;
Mode(1) = 1.01; t(1) = 0; q(1) = 0; Mach(1) = 0;
DragLoss(1) = 0; GravityLoss(1) = 0; SteeringLoss(1) = 0;
r = R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cosd(LaunchSite(2))^2+sind(LaunchSite(2))^2)*cosd(LaunchSite(1))^2+R0^4*sind(LaunchSite(1))^2);
[xs,ys,zs] = sph2cart(pi/180*LaunchSite(2),pi/180*LaunchSite(1),r+dumbStool);
v = wEarth*r*[-sind(LaunchSite(2))*cosd(LaunchSite(1)), cosd(LaunchSite(2))*cosd(LaunchSite(1)), 0];
v_Air = [0 0 0];
if launchSouth
    VectLaunch = [8000*cosd(i_orb)-wEarth*r*cosd(LaunchSite(1)), -8000*sind(i_orb)];
else
    VectLaunch = [8000*cosd(i_orb)-wEarth*r*cosd(LaunchSite(1)), 8000*sind(i_orb)];
end
LAz = atan2d(VectLaunch(2),VectLaunch(1)); % LAz is launching azimuth (°, CCW from East)
z(i,:) = [xs, ys, zs, v(1), v(2), v(3), m0S1];
Orbit = SV2Ele([xs ys zs],[v(1) v(2) v(3)],G,M); % orbital elements: a, e, i, RAAN, w
crashFlag = false;

%% Sim Loops
% Stage 1
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV(i) + a(i)*Ft(FtS1,altitude(i),R0)/z(i+1,7)*dt;%IspS1*9.81*log(m0S1/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    
    % control parameters (based on mode)
    if z(i+1,7) < m0S1-mpS1
        % stage
        fprintf('Stage! (T+ %.0f)\n',t(i));
        Mode(i+1) = 3;
        a(i+1) = 0;
        i = i+1;
        break
    elseif altitude(i+1) < h_vert
        % vertical ascent
        if length(Mode) < 5
            Mode(i+1) = 1.01; % special first few instances of true 'up'
        else
            Mode(i+1) = 1;
        end
        a(i+1) = 1;
    else
        % pitch program
        if Mode(i) == 1
            Mode(i+1) = 2.01; % special first instance to kick over (opposite of LAz)
        else
            Mode(i+1) = 2;
        end
        a(i+1) = 1;
    end
    
    % Sanity Checks
    if altitude(i+1) < -100
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % iterate
    i = i+1;    
end

if ~(crashFlag)
    idxS1 = i;
    t_trigger = t(i) + t_coast;
end

% Coast
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,Ft(FtS1,altitude(i),R0),a(i),Mode(i),Isp(IspS1,altitude(i),R0),P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV(i) + a(i)*Ft(FtS1,altitude(i),R0)/z(i+1,7)*dt;%IspS1*9.81*log(m0S1/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    % control parameters (based on mode)
    if t(i+1) > t_trigger
        % stage
        fprintf('Second Stage Fire! (T+ %.0f)\n',t(i));
        Mode(i+1) = 4;
        a(i+1) = 1;
        i = i+1;
        break
    else
        % coast
        Mode(i+1) = 3;
        a(i+1) = 0;
    end
    
    % Sanity Checks
    if altitude(i+1) < 0
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % iterate
    i = i+1; 
end

if ~(crashFlag)
    idxS2 = i;
    z(i,7) = m0S2; % fix mass (staged)
    dV_carry = dV(end);
end

% Stage 2
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    % control parameters (based on mode)
    if z(i+1,7) < m0S2-mpS2
        % out of fuel
        fprintf('Out of Fuel! (T+ %.0f)\n',t(i));
        Mode(i+1) = Mode(i);
        a(i+1) = a(i);
        break
    elseif (Orbit(i+1,1)*(1+Orbit(i+1,2))-R0 > h_orb*1000) && (dot([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)]) > 0.01)
        % overshot target apoapsis && ascending: coast until apoapsis
        fprintf('SECO; wait until apoapsis! (T+ %.0f)\n',t(i));
        Mode(i+1) = 3;
        a(i+1) = 0;
        i = i+1;
        break
    % enhance remaining controls
    elseif altitude(i+1) > altitude(i)
        % prograde
        Mode(i+1) = 4;
        a(i+1) = 1;
    else
        % horizontal
        Mode(i+1) = 5;
        a(i+1) = 1;
    end
    
    % Sanity Checks
    if altitude(i+1) < 0
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % End checks
    if (Orbit(i+1,2) < 0.1) && (Orbit(i+1,1)-R0 > 1000*h_orb) && (Orbit(i+1,1)*(1-Orbit(i+1,2))-R0 > 120000)
        fprintf('Orbit reached! (T+ %.0f)\n',t(i));
        break
    elseif Orbit(i+1,1) < 0
        fprintf('Escaped Earth! (T+ %.0f)\n',t(i));
        break
    end
    
    % iterate
    i = i+1; 
end

if ~(crashFlag)
    idxC2 = i;
    vertSpeed = (z(i,1)*z(i,4)+z(i,2)*z(i,5)+z(i,3)*z(i,6))/sqrt(z(i,1)^2+z(i,2)^2+z(i,3)^2);
    aCent = ((z(i,4)^2+z(i,5)^2+z(i,6)^2)-vertSpeed^2)/sqrt(z(i,1)^2+z(i,2)^2+z(i,3)^2);
    t2Apo = vertSpeed/(G*M/(z(i,1)^2+z(i,2)^2+z(i,3)^2) - aCent);
    vApo = sqrt(G*M*(2/(Orbit(i,1)*(1+Orbit(i,2))) - 1/Orbit(i,1)));
    tBurn = (sqrt(G*M/(R0+1000*h_orb)) - vApo)/(FtS2/z(i,7));
    t_trigger2 = t(i) + t2Apo - tBurn/2;
end

% Second Stage Coast
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),FtS1,a(i),Mode(i),IspS1,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,FtS1,a(i),Mode(i),IspS1,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,FtS1,a(i),Mode(i),IspS1,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,FtS1,a(i),Mode(i),IspS1,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    % control parameters (based on mode)
    if (t(i+1) > t_trigger2) || (altitude(i+1) < altitude(i))
        % altitude decreasing, re-light second stage to circularize
        fprintf('SES-2! (T+ %.0f)\n',t(i));
        Mode(i+1) = 5;
        a(i+1) = 1;
        i = i+1;
        break
    else
        % coast
        Mode(i+1) = 3;
        a(i+1) = 0;
    end
    
    % Sanity Checks
    if altitude(i+1) < 0
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % iterate
    i = i+1; 
end

if ~(crashFlag)
    idxSES2 = i;
end

% Stage 2: Circularize
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    % control parameters (based on mode)
    if z(i+1,7) < m0S2-mpS2
        % out of fuel
        fprintf('Out of Fuel! (T+ %.0f)\n',t(i));
        Mode(i+1) = Mode(i);
        a(i+1) = a(i);
        break
    % enhance remaining controls
    else
        % horizontal
        Mode(i+1) = 5;
        a(i+1) = 1;
    end
    
    % Sanity Checks
    if altitude(i+1) < 0
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % End checks
    if (Orbit(i+1,2) < Orbit(i,2)) && (abs(Orbit(i+1,1)-R0-1000*h_orb) < 1000*smaTol)% (Orbit(i+1,1)-R0 > 1000*h_orb)% && (Orbit(i+1,1)*(1-Orbit(i+1,2))-R0 > 120000)
        % eccentricity increasing, SMA >= h_orb
        fprintf('Orbit reached! (T+ %.0f)\n',t(i));
        break
    elseif Orbit(i+1,1) < 0
        fprintf('Escaped Earth! (T+ %.0f)\n',t(i));
        break
    end
    
    % iterate
    i = i+1; 
end

if ~(crashFlag)
    idxC3 = i;
    t_trigger3 = t(i) + postInsertion_Coast;
end

% Finished Orbit Coast
while ~(crashFlag)
    % RK4 Slopes
    k1 = dz(z(i,:),FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k2 = dz(z(i,:)+dt/2*k1,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k3 = dz(z(i,:)+dt/2*k2,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    k4 = dz(z(i,:)+dt*k3,FtS2,a(i),Mode(i),IspS2,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz);
    
    % Integrate
    z(i+1,:) = z(i,:) + dt/6*(k1+2*k2+2*k3+k4);
    t(i+1) = t(i) + dt;
    
    % ancillary metrics
    [long,lat,r] = cart2sph(z(i,1), z(i,2), z(i,3));
    v_Air(i+1,:) = [z(i,4), z(i,5), z(i,6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
    altitude(i+1) = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
    Mach(i+1) = norm(v_Air(i+1,:))/EarthAtmos(altitude(i),R0,'a'); % get Mach #
    q(i+1) = norm(v_Air(i,:))^2/2*EarthAtmosRho(altitude(i),R0);
    DragLoss(i+1) = DragLoss(i) + dt*q(i+1)*S*dragCoeff(Mach(i+1))/z(i+1,7);
%     GravityLoss(i+1) = GravityLoss(i) + dt*G*M/z(i+1,2)^2*sin(z(i+1,4));
%     SteeringLoss(i+1) = SteeringLoss(i) + dt*a(i)*FtS1/z(i+1,5)*(1-cos(TV(i)-z(i+1,4)));
    dV(i+1) = dV_carry + IspS2*9.81*log(m0S2/z(i+1,7)); 
    Orbit(i+1,:) = SV2Ele([z(i+1,1) z(i+1,2) z(i+1,3)],[z(i+1,4) z(i+1,5) z(i+1,6)],G,M); % orbital elements (a,e,i,RAAN,w)
    
    % control parameters (based on mode)
    if (t(i+1) > t_trigger3)
        % post insertion coast period done, end sim
        fprintf('Sim Complete! (T+ %.0f)\n',t(i));
        Mode(i+1) = Mode(i);
        a(i+1) = 0;
        i = i+1;
        break
    else
        % coast
        Mode(i+1) = 3;
        a(i+1) = 0;
    end
    
    % Sanity Checks
    if altitude(i+1) < 0
        fprintf('Crashed at T+ %.0f seconds.\n',t(i+1));
        crashFlag = true;
        break
    end
    
    % iterate
    i = i+1; 
end

if (crashFlag)
    if ~exist('idxS1','var')
        idxS1 = i+1;
    end
    if ~exist('idxS2','var')
        idxS2 = i+1;
    end
    if ~exist('idxC2','var')
        idxC2 = i+1;
    end
    if ~exist('idxSES2','var')
        idxSES2 = i+1;
    end
    if ~exist('idxC3','var')
        idxC3 = i+1;
    end
end

%% Reporting
fprintf('Mass remaining: %.0f kg\n',z(end,7)-(m0S2-mpS2));
fprintf('Orbit is: %.0f x %.0f km x %.1f °\n',(Orbit(end,1)*(1+Orbit(end,2))-R0)/1000,(Orbit(end,1)*(1-Orbit(end,2))-R0)/1000,Orbit(end,3));
% fprintf('Losses are: %.1f m/s Drag, %.1f m/s Gravity, %.1f m/s Steering\n',DragLoss(end),GravityLoss(end),SteeringLoss(end));
fprintf('Delta-V Expended: %.0f m/s \n',dV(end));

%{
for i = 1:length(t)
    P(i) = EarthAtmos(altitude(i),R0,'P');
    plotVal(i) = P(i) + EarthAtmosRho(altitude(i),R0)/2*(v_Air(i,1)^2+v_Air(i,2)^2+v_Air(i,3)^2);
end

plot(t,plotVal/101325,'LineWidth',2);
hold on
plot(t,P/101325,'LineWidth',2);
grid on
%}

%% Trajectory Plots
figure('name','Trajectory');
set(gcf,'WindowState','maximized');
[Xs,Ys,Zs] = ellipsoid(0,0,0,R0,R0,R0*(1-f),360);
surf(Xs,Ys,Zs);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',EarthImg,'EdgeColor',[0 0 0],'EdgeAlpha',0)
hold on
% correct longitude & make ground track
for i = 1:length(t)
    [longitude(i),latitude_geo(i),radius(i)] = cart2sph(z(i,1), z(i,2), z(i,3));
    %{
    R = [-sin(latitude_geo(i))*cos(longitude(i)), -sin(latitude_geo(i))*sin(longitude(i)), cos(latitude_geo(i));
        cos(latitude_geo(i))*(-sin(longitude(i))), cos(latitude_geo(i))*cos(longitude(i)), 0;
        cos(latitude_geo(i))*cos(longitude(i)), sin(longitude(i)), sin(latitude_geo(i))];
    tempDumb = (R*[v_Air(i,1); v_Air(i,2); v_Air(i,3)])';
    North(i) = tempDumb(1); East(i) = tempDumb(2); Up(i) = tempDumb(3);
    [az(i), alt(i), ~] = cart2sph(North(i),East(i),Up(i));
    %}
    longitude_Plot(i) = longitude(i) - t(i)*wEarth; % correct Longitude for rotating Earth
    [x_traj(i),y_traj(i),z_traj(i)] = sph2cart(longitude_Plot(i),latitude_geo(i),radius(i));
    rsurf = R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(longitude_Plot(i))^2+sin(longitude_Plot(i))^2)*cos(latitude_geo(i))^2+R0^4*sin(latitude_geo(i))^2);
    [x_g(i),y_g(i),z_g(i)] = sph2cart(longitude_Plot(i),latitude_geo(i),rsurf);
end
plot3(x_traj,y_traj,z_traj,'LineWidth',2); % trajectory
plot3(x_g,y_g,z_g,'r','LineWidth',2); % ground track
grid on
axis equal
axis off
set(gca,'clipping','off');
xlim([-3*R0 3*R0]);
ylim([-3*R0 3*R0]);
zlim([-3*R0 3*R0]);

figure('name','Air Speed');
set(gcf,'WindowState','maximized');
% plot(t,sqrt(z(:,4).^2+z(:,5).^2+z(:,6).^2));
plot(t,sqrt(v_Air(:,1).^2+v_Air(:,2).^2+v_Air(:,3).^2),'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
title('Velocity (Air, m/s)');
grid on

figure('name','Altitude');
set(gcf,'WindowState','maximized');
plot(t,altitude/1000,'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
title('Altitude (km)');
grid on

figure('name','Apsis Heights');
set(gcf,'WindowState','maximized');
plot(t,(Orbit(:,1).*(1+Orbit(:,2))-R0)/1000,'LineWidth',2);
hold on
plot(t,(Orbit(:,1).*(1-Orbit(:,2))-R0)/1000,'LineWidth',2);
yline(h_orb,'k--');
grid on
legend('Apoapsis','Periapsis','Location','SouthOutside','NumColumns',2);
title('Apsis Heights (km)');
xlabel('Time (s)');
ylabel('Apsis Height (km)');

figure('name','Elements');
set(gcf,'WindowState','maximized');
subplot(2,2,1);
plot(t,Orbit(:,2),'LineWidth',2);
hold on
yline(1,'k--','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Eccentricity');
ylim([0 1.2]);

subplot(2,2,2);
plot(t,Orbit(:,3),'LineWidth',2);
hold on
yline(i_orb,'k--','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Inclination (°)');
ylim([0 180]);

subplot(2,2,3);
plot(t,Orbit(:,4),'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('R.A. of Ascending Node (°)');
ylim([0 360]);

subplot(2,2,4);
plot(t,Orbit(:,5),'LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Argument of Periapsis (°)');
ylim([0 360]);


figure('name','Key Performance Indicators');
set(gcf,'WindowState','maximized');
subplot(2,2,1);
plot(t,Orbit(:,2),'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Eccentricity');
ylim([0 1]);
set(gca,'FontSize',12);

subplot(2,2,2);
% plot(t,sqrt(z(:,4).^2+z(:,5).^2+z(:,6).^2),'LineWidth',2);
plot(t,sqrt(v_Air(:,1).^2+v_Air(:,2).^2+v_Air(:,3).^2),'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Air Speed (m/s)');
set(gca,'FontSize',12);

subplot(2,2,3);
plot(t,altitude/1000,'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('Altitude (km)');
set(gca,'FontSize',12);

subplot(2,2,4);
plot(t,dV,'LineWidth',2);
hold on
xline(t(idxS1),'k-','LineWidth',2);
xline(t(idxS2),'k-','LineWidth',2);
xline(t(idxC2),'k-','LineWidth',2);
xline(t(idxSES2),'k-','LineWidth',2);
xline(t(idxC3),'k-','LineWidth',2);
grid on
xlabel('Time (s)');
ylabel('\Delta V Used (m/s)');
set(gca,'FontSize',12);


%% Stop Clock
toc

function dzdt = dz(z,F,a,Mode,Isp,P_TV,G,M,S,R0,wEarth,f,inertialBlend,LAz)
% z1 = x, z2 = y, z3 = z,
% z4 = vx, z5 = vy, z6 = vz
% z7 = mass

% Modes:
% 1: Vertical ascent
% 2: Pitch Program
% 3: Coast
% 4: Prograde
% 5: Horizontal
% 6: Maintain Altitude

% Simple Slopes
dzdt(1) = z(4);
dzdt(2) = z(5);
dzdt(3) = z(6);
dzdt(7) = -a*F/9.81/Isp; % mass rate of change (kg/s)

% Supporting Metrics (always used)
[long,lat,r] = cart2sph(z(1), z(2), z(3));
v_Air = [z(4), z(5), z(6)] - wEarth*norm(r)*[-sin(long)*cos(lat), cos(long)*cos(lat), 0]; % air speed
altitude = r - R0^3*(1-f)/sqrt(R0^4*(1-f)^2*(cos(long)^2+sin(long)^2)*cos(lat)^2+R0^4*sin(lat)^2);
Mach = norm(v_Air)/EarthAtmos(altitude,R0,'a'); % get Mach #
R = [-sin(lat)*cos(long), -sin(lat)*sin(long), cos(lat); % rotation matrix from ECI cartesian to NEU frame
    cos(lat)*(-sin(long)), cos(lat)*cos(long), 0;
    cos(lat)*cos(long), sin(long), sin(lat)];

% Accelerations (m/s^2)
aG = G*M/(norm(r)^2);
aD = EarthAtmosRho(altitude,R0)/2*S*dragCoeff(Mach)*norm(v_Air)^2/z(7);
aT = a*F/z(7);

% Inertial Vector Accelerations (Slopes, Mode dependent)
    if Mode == 1 || Mode == 1.01
    % Vertical Ascent: target to 'pitch' of 90 degrees
        [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],v_Air,Mode,P_TV,LAz,R);
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*Tx;
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*Ty;
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*Tz;
    elseif Mode == 2 || Mode == 2.01
    % Pitch Program: thrust vector offset from prograde
        [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],v_Air,Mode,P_TV,LAz,R);
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*Tx;
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*Ty;
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*Tz;
    elseif Mode == 3
    % Coast: thrust is zero
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air);
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air);
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air);
    elseif Mode == 4
    % Prograde: thrust is inline with air speed%inertial velocity
%         dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*z(4)/sqrt(z(4)^2+z(5)^2+z(6)^2);
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*v_Air(1)/norm(v_Air);
%         dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*z(5)/sqrt(z(4)^2+z(5)^2+z(6)^2);
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*v_Air(2)/norm(v_Air);
%         dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*z(6)/sqrt(z(4)^2+z(5)^2+z(6)^2);
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*v_Air(3)/norm(v_Air);
    elseif Mode == 5
    % Horizontal: thrust is horizontal in direction of air speed%inertial velocity
%         [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],[z(4) z(5) z(6)],Mode,P_TV,LAz,R);
        [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],v_Air,Mode,P_TV,LAz,R);
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*Tx;
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*Ty;
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*Tz;
    elseif Mode == 6
    % Maintain Altitude: vertical component of thrust balances vertical component of other accelerations
%         [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],[z(4) z(5) z(6)],Mode,P_TV);
        [Tx,Ty,Tz] = thrustVect([z(1) z(2) z(3)],v_Air,Mode,P_TV,LAz,R);
        dzdt(4) = -aG*z(1)/norm(r) - aD*v_Air(1)/norm(v_Air) + aT*Tx;
        dzdt(5) = -aG*z(2)/norm(r) - aD*v_Air(2)/norm(v_Air) + aT*Ty;
        dzdt(6) = -aG*z(3)/norm(r) - aD*v_Air(3)/norm(v_Air) + aT*Tz;
    end
end

function Isp = Isp(I,alt,R0)
    Pi = EarthAtmos(alt,R0,'P');
    P0 = EarthAtmos(0,R0,'P');
    rat = 1 - Pi/P0; % progression from sea-level to vacuum
    Isp = I(1) + rat*(I(2)-I(1)); % Isp (s)
end

function F_t = Ft(F,alt,R0)
    Pi = EarthAtmos(alt,R0,'P');
    P0 = EarthAtmos(0,R0,'P');
    rat = 1 - Pi/P0; % progression from sea-level to vacuum
    F_t = F(1) + rat*(F(2)-F(1)); % thrust (N)
end

function Cd = dragCoeff(M)
% curve fit of data from https://space.stackexchange.com/a/12650/40257
    if M > 8.5
        Cd = 0.26;
    elseif M > 3 && M <= 8.5
        Cd = 0.000152278*M^5 + -0.004619289*M^4 + 0.051607772*M^3 + -0.253769873*M^2 + 0.500892646*M;
    elseif M > 0.5 && M <=1.4
        Cd = 0.554890681*M^5 + -3.390155651*M^4 + 6.85436193*M^3 + -5.629190532*M^2 + 2.010093572*M;
    elseif M > 1.4 && M <= 3
        Cd = 0.01785091*M^5 + -0.131572264*M^4 + 0.405280079*M^3 + -0.837683138*M^2 + 1.063722819*M;
    else % M <= 0.5
        Cd = -0.364189143*M^3 + 0.524189143*M^2 + -0.251047286*M + 0.3;
    end
end

function elements = SV2Ele(r,v,G,M)
% turn state vector to elements
elements(1) = 1/(2/norm(r)-norm(v)^2/G/M);
h = cross(r,v);
e_Vect = cross(v,h)/G/M-r/norm(r);
elements(2) = norm(e_Vect);
elements(3) = acosd(h(3)/norm(h));
n = cross([0,0,1],h);
    if n(2) > 0
        elements(4) = acosd(n(1)/norm(n));
    else
        elements(4) = 360 - acosd(n(1)/norm(n));
    end
    if e_Vect(1) > 0
        elements(5) = acosd(dot(n,e_Vect)/norm(n)/elements(2));
    else
        elements(5) = 360 - acosd(dot(n,e_Vect)/norm(n)/elements(2));
    end
end

function [Tx,Ty,Tz] = thrustVect(r,v,Mode,P_TV,LAz,R)
% return inertial thrust vector (P_TV is in radians)
if Mode == 1
    % Vertical Ascent: target to 'pitch' of 90 degrees
    tempDumb = (R*[v(1); v(2); v(3)])';
    North = tempDumb(1); East = tempDumb(2); Up = tempDumb(3);
    [~, alt_cur, ~] = cart2sph(North,East,Up);
    if alt_cur < 88/180*pi
        alt_cur = pi/2;
        [Te,Tn,Tu] = sph2cart(0,alt_cur,1); % get cartesian T vect in NEU frame
    else
            differ = pi/2 - alt_cur; % difference
            [Te,Tn,Tu] = sph2cart((LAz+180)/180*pi,alt_cur+differ/16,1); % get cartesian T vect in NEU frame
    end
    dumbTemp = (R\[Tn;Te;Tu])';
    Tx = dumbTemp(1); Ty = dumbTemp(2); Tz = dumbTemp(3);
elseif Mode == 1.01
    % Vertical Ascent: first few instances, true 'up'
    Tx = r(1)/norm(r); Ty = r(2)/norm(r); Tz = r(3)/norm(r);
elseif Mode == 2
    % Pitch Program
    alt_cur = asin((v(1)*r(1)+v(2)*r(2)+v(3)*r(3))/sqrt(v(1)^2+v(2)^2+v(3)^2)/sqrt(r(1)^2+r(2)^2+r(3)^2)); % find current Alt (radians)
    [Te,Tn,Tu] = sph2cart((LAz)/180*pi,alt_cur-P_TV,1); % get cartesian T vect in NEU frame
    dumbTemp = (R\[Tn;Te;Tu])';
    Tx = dumbTemp(1); Ty = dumbTemp(2); Tz = dumbTemp(3);  
elseif Mode == 2.01
    % kick into launch (opposite of) azimuth; Pitch Program
    [Te,Tn,Tu] = sph2cart((LAz+180)/180*pi,(90)/180*pi,1); % get cartesian T vect in NEU frame
    dumbTemp = (R\[Tn;Te;Tu])';
    Tx = dumbTemp(1); Ty = dumbTemp(2); Tz = dumbTemp(3); 
elseif Mode == 5
    % Horizontal
    %{
    theta = acosd(dot(v,r)/norm(r)/norm(v));
    A = [h_norm(1), h_norm(2), h_norm(3);
         r(1) r(2) r(3);
         v(1) v(2) v(3)];
    b = [0;0;norm(v)*cosd(90-theta)];
    temp = A\b;
    temp = temp'/norm(temp);
    Tx = temp(1); Ty = temp(2); Tz = temp(3);
    %}
    [Te,Tn,Tu] = sph2cart((LAz)/180*pi,(0)/180*pi,1); % get cartesian T vect in NEU frame
    dumbTemp = (R\[Tn;Te;Tu])';
    Tx = dumbTemp(1); Ty = dumbTemp(2); Tz = dumbTemp(3); 
elseif Mode == 6
    % Maintain Altitude
end

end