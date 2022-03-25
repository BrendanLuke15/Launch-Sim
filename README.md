# Launch Vehicle Simulator
**This is a relatively simple low-fidelity sim for launch vehicles.**

Some features:
- Simple atmospheric model from [Braeunig's Rocket & Space Technology](http://www.braeunig.us/space/atmmodel.htm) (US Standard Atmos)
- 2D & 3D versions
  - WGS-84 ellipsoid for 3D
- constant pitch rate pitch program
- crude vehicle guidance

-----

**Vehicle Design:**

Design the launch vehicle and payload, booster stage supports varaible specific impulse and thrust levels:

```
% SpaceX Falcon 9 w/ Crew Dragon
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

% Vehicle Guidance
P_TV = 0.060; % pitch thrust vector, 'Angle of Attack' (deg)
t_coast = 10; % coast period (s)
h_vert = 1000; % initial vertical trajectory height (m)
```


**Simulation Parameters:**

Configure aspects of the launch & simulation:

```
% sim parameters
dt = 0.1; % time step (s)
LaunchSite = [28.6, -80.7]; % lat, lon coords of launch site (deg)
h_orb = 225; % orbital altitude target (km)
smaTol = 10; % +/- tolerance on semi-major axis (km)
i_orb = 50;%51.6;%LaunchSite(1); % target orbital inclination (°)
launchSouth = false;%true; % direction of launching azimuth, 'North' (false) or 'South' (true)
postInsertion_Coast = 100; % length of time to continue sim post insertion (s)
```

# Sample Outputs:

![image](https://user-images.githubusercontent.com/31905278/160215415-25b4ba5c-6d43-48fb-80d0-d430da48e2f9.png)
