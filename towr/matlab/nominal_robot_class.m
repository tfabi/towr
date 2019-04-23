%% Nominal robot classes

%%Universal look at CAD to get offset z_shoulder to z_com
x_nom = 0.34;
y_nom = 0.19;
z_nom = 0;

nom_hip.universal.LF = [x_nom, y_nom, z_nom];
nom_hip.universal.LH = [-x_nom, y_nom, z_nom];
nom_hip.universal.RF = [x_nom, -y_nom, z_nom];
nom_hip.universal.RH = [-x_nom, -y_nom, z_nom];

%%Speedy

%%Massivo

%%Mini

%%Robot

nominal_hip_position = nom_hip.universal;
