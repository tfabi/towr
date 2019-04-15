%% Plot values
close all;


%% base position and velocity
figure();
subplot(5,2,1);
plot(t, ts_base_pos.Data(:,1));
ylabel('x position [m]');
grid on
title('Center of mass position')
subplot(5,2,3);
plot(t, ts_base_pos.Data(:,2));
ylabel('y position [m]');
grid on
subplot(5,2,5);
plot(t, ts_base_pos.Data(:,3));
xlabel('time [s]');
ylabel('z position [m]');
grid on

subplot(5,2,2);
plot(t, ts_base_twist.Data(:,1));
ylabel('x velocity [m/s]');
grid on
title('Center of mass velocity')
subplot(5,2,4);
plot(t, ts_base_twist.Data(:,2));
ylabel('y velocity [m/s]');
grid on
subplot(5,2,6);
plot(t, ts_base_twist.Data(:,3));
xlabel('time [s]');
ylabel('z velocity [m/s]');
grid on


%% end effector forces
subplot(5,2,[7, 8, 9, 10])
% for flying trot
%plot(t, ee_force_total_0, 'b', t, ee_force_total_1, 'g', t, ee_force_total_2, 'g', t, ee_force_total_3, 'b')
% for walk
plot(t, foot_0.force(:,3), 'b', t, foot_1.force(:,3), 'g', t, foot_2.force(:,3), 'r', t, foot_3.force(:,3), 'bl')
title('End Effector forces');
xlabel('time [s]')
ylabel('end effector force [N]')
grid on

%%  Generate pdf from the figure for paper
% width  = 20;
% height = 15;
% 
% f.Units = 'centimeters';
% f.PaperUnits = 'centimeters';
% f.Position = [0, 0, width, height];
% f.PaperSize = [width, height];
% f.PaperPositionMode = 'auto';

%saveas(f, 'plots_', 'png')

% fn = 'side_stepping';
% 
% saveas(fh, fn, 'pdf')
% system(['pdfcrop ' fn ' ' fn]);
