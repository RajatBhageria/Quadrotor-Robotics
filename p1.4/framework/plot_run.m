
function plot_run(h)
%
% plot the data stored in h
%
    fig1 = figure(1);
    fig1.Name = 'position';
    plot_position(h);
    fig2 = figure(2);
    fig2.Name = 'velocity';
    plot_velocity(h);
    fig3 = figure(3);
    fig3.Name = 'acceleration';
    plot_acceleration(h);   
end

function plot_position(h)
  t=subplot(3,1,1);
  plot_data(h, 'x[m]', 'pos_des_x', 'pos_x');
  subplot(3,1,2);
  plot_data(h, 'y[m]', 'pos_des_y', 'pos_y');
  subplot(3,1,3);
  plot_data(h, 'z[m]', 'pos_des_z', 'pos_z');
  xlabel('time [s]');
  title(t,'position');
end

function plot_velocity(h)
  t=subplot(3,1,1);
  plot_data(h, 'vx[m/s]', 'vel_des_x', 'vel_x');
  subplot(3,1,2);
  plot_data(h, 'vy[m/s]', 'vel_des_y', 'vel_y');
  subplot(3,1,3);
  plot_data(h, 'vz[m/s]', 'vel_des_z', 'vel_z');
  xlabel('time [s]');
  title(t,'velocity');
end

function plot_acceleration(h)
  t=subplot(3,1,1);
  plot_data(h, 'acc x [m/s^2]', 'acc_des_x');
  subplot(3,1,2);
  plot_data(h, 'acc y [m/s^2]', 'acc_des_y');
  subplot(3,1,3);
  plot_data(h, 'acc z [m/s^2]', 'acc_des_z');
  xlabel('time [s]');
  title(t,'acceleration');
end
