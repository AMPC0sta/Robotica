% Define Keplerian orbital elements to Aurora Mission
a = 6903e3;        % Semi-major axis in meters
e = 0.003622;      % Eccentricity ( almost circular)
i = 90;            % Inclination in degrees (polar orbit)
RAAN = 90;         % Right Ascension of Ascending Node in degrees
omega = 0;         % Argument of periapsis in degrees
nu = 0;            % True anomaly at epoch (in degrees)
queryTime = datetime("now",TimeZone="UTC");

startTime = queryTime - hours(12);
stopTime = queryTime + hours(12);
sampleTime = 60;

scenario = satelliteScenario();
% Set the scenario's start and stop time
scenario.StartTime = startTime;  % Start time
scenario.StopTime = stopTime;   % Stop time
scenario.SampleTime = sampleTime;

% Create the satellite and add it to the scenario using Keplerian elements
satelliteObj = satellite(scenario, a, e, i, RAAN, omega, nu,"OrbitPropagator","two-body-keplerian","Name","AuroraCubeSat");

% Open the satellite scenario viewer to visualize the satellite's orbit
viewer = satelliteScenarioViewer(scenario);

% Set initial camera position
