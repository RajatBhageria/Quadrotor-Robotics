classdef QuadrotorROS
    % ROS interface for basic quadrotor control in MEAM 620
    % Written by Alex Zhu
    % Modified by Justin Thomas, Bernd Pfrommer
    
    properties
        command_publisher
        vicon_subscriber
        service_clients
        killswitch_service
    end
    
    % Data from a callback function must be stored in a constant handle
    % class in order to update properly. (This won't work with multiple
    % quadrotor objects).
    properties(Constant)
        data = SharedData;
        dplt = PlotData;
    end
    
    methods
        function [quadrotor] = QuadrotorROS(quadname)
            vicon_topic = [quadname, '/odom'];
            command_topic = [quadname, '/so3cmd_to_crazyflie/cmd_vel_fast/'];
            command_msg_type = 'geometry_msgs/Twist';
            disp('Shutting down existing ros nodes...');
            rosshutdown;
            pause(2);
            rosinit('http://demo-nuc:11311'); %, 'NodeHost', '192.168.129.233'); %rosinit('http://192.168.129.201:11311');
            pause(2);
            quadrotor.command_publisher = rospublisher(command_topic, command_msg_type);
            
            quadrotor.data.state = 'backend_control';
            quadrotor.data.t_init = [];
            quadrotor.data.qd = [];
            quadrotor.data.history = [];
            quadrotor.dplt.dplt = [];
            quadrotor.killswitch_service = rossvcserver(['/kill_matlab_', quadname], 'std_srvs/Trigger', @quadrotor.killswitch);
            quadrotor.vicon_subscriber = rossubscriber(vicon_topic, 'nav_msgs/Odometry', @quadrotor.viconCallback);
            quadrotor.service_clients.motors = rossvcclient(['/', quadname, '/mav_services/motors']);
            quadrotor.service_clients.takeoff = rossvcclient(['/', quadname, '/mav_services/takeoff']);
            quadrotor.service_clients.hover = rossvcclient(['/', quadname, '/mav_services/hover']);
            quadrotor.service_clients.land = rossvcclient(['/', quadname, '/mav_services/land']);
            quadrotor.service_clients.transition = rossvcclient(['/', quadname, '/trackers_manager/transition']);
            
            quadrotor.data.iter = 0;
        end
        
        function [quadrotor, response] = killswitch(quadrotor, service, reqmsg, defrespmsg)
            quadrotor.data.state = 'backend_control';
            response = defrespmsg;
            response.Success = true;
        end
        
        function [quadrotor] = shover(quadrotor)
          student_control_hover(quadrotor.data.qd);   
          quadrotor.transition('std_trackers/NullTracker');
          quadrotor.data.state = 'hover';
        end
        
        function [quadrotor] = sgo_to_waypoint(quadrotor, path)
            student_control_waypt(quadrotor.data.t, quadrotor.data.qd, path);
            quadrotor.transition('std_trackers/NullTracker');
            quadrotor.data.state = 'waypoint';
        end
        
        function [quadrotor] = dryrun(quadrotor, path)
            student_control_waypt(quadrotor.data.t, quadrotor.data.qd, path);
            quadrotor.data.state = 'dryrun';
        end
                
        function [quadrotor] = viconCallback(quadrotor, sub, msg)
            t = msg.Header.Stamp.Sec + ...
                msg.Header.Stamp.Nsec/10^9;
              
            if isempty(quadrotor.data.t_init)
              quadrotor.data.t_init = t;
            end
            
            t = t-quadrotor.data.t_init;
            quadrotor.data.t = t;
              
            position = [...
                msg.Pose.Pose.Position.X;...
                msg.Pose.Pose.Position.Y;...
                msg.Pose.Pose.Position.Z];
            quaternion = [...
                msg.Pose.Pose.Orientation.W;...
                msg.Pose.Pose.Orientation.X;...
                msg.Pose.Pose.Orientation.Y;...
                msg.Pose.Pose.Orientation.Z];
            lin_vel = [...
                msg.Twist.Twist.Linear.X;...
                msg.Twist.Twist.Linear.Y;...
                msg.Twist.Twist.Linear.Z];
            ang_vel = [...
                msg.Twist.Twist.Angular.X;...
                msg.Twist.Twist.Angular.Y;...
                msg.Twist.Twist.Angular.Z];
            
            R = quat2rotm(quaternion');
            [phi,theta,psi] = RotToRPY_ZXY(R);
              
            qd{1}.pos = position;
            qd{1}.vel = lin_vel;
            qd{1}.euler = [phi,theta,psi]';
            qd{1}.omega = ang_vel;              
            
            quadrotor.data.qd = qd;
   
            switch quadrotor.data.state
              case 'backend_control'
                return
              case 'hover'
                desired_state = student_control_hover();
              case {'waypoint', 'dryrun'}
                desired_state = student_control_waypt(t);
            end
                     
            qd{1}.pos_des      = desired_state.pos;
            qd{1}.vel_des      = desired_state.vel;
            qd{1}.acc_des      = desired_state.acc;
            qd{1}.yaw_des      = desired_state.yaw;
            qd{1}.yawdot_des   = desired_state.yawdot;
            params = crazyflie;
                
            [~, ~, mtrpy, drpy] = controller(qd, t, 1, params);
            trpy = [mtrpy(1)*1000/params.grav, mtrpy(2:4)];
            
            % for debugging, add an extra output to your controller,
            % and append that output to an array like so:
            % [~, ~, mtrpy, drpy, d] = controller(qd, t, 1, params);
            % quadrotor.dplt.dplt = [quadrotor.dplt.dplt; d];
            % Then add another member function to plot the data

            h = [t, qd{1}.pos_des', qd{1}.vel_des', qd{1}.acc_des', ...
                 qd{1}.yaw_des, qd{1}.yawdot_des, ...
                qd{1}.pos', qd{1}.vel', qd{1}.euler', qd{1}.omega', ...
                 mtrpy(1:4)];
            quadrotor.data.history = [quadrotor.data.history; h];
            
            if ~strcmp(quadrotor.data.state, 'dryrun')
                quadrotor.publishCommand(trpy, drpy, qd{1});
            end
        end
 
        function [quadrotor] = publishCommand(quadrotor, trpy, drpy, qd)
            % Need to decide on structure of msg first
            msg = rosmessage('geometry_msgs/Twist');

            % Thrust is already in [g]
            % Convert to 0 to 1 thrust scale
            c1 = -0.6709;
            c2 = 0.1932;
            c3 = 13.0652;
            thrust_pwm = c1 + c2 * sqrt(c3 + trpy(1));
            thrust_pwm = min(thrust_pwm, 0.9);
            
            % Scale to full range
            thrust_pwm_max = 60000;
            msg.Linear.Z = thrust_pwm * thrust_pwm_max;
            
            msg.Linear.X = trpy(3) * 180 / pi();
            msg.Linear.Y = trpy(2) * 180 / pi();
            
            R = RPYtoRot_ZXY(qd.euler(1), qd.euler(2), qd.euler(3));
            yaw_cur = atan2(R(2,1), R(1,1));
            R_des = RPYtoRot_ZXY(0,0,trpy(4));
            yaw_des = atan2(R_des(2,1), R_des(1,1));
            
            e_yaw = yaw_des - yaw_cur;
            if e_yaw > pi
              e_yaw = e_yaw - 2 * pi;
            elseif e_yaw < -pi
              e_yaw = e_yaw + 2 * pi;
            end
            kr = 0.1;
            yaw_rate_des = (-kr * e_yaw) + drpy(3);
            yaw_rate_des = yaw_rate_des * 180 / pi;
            msg.Angular.Z = yaw_rate_des;
                       
            quadrotor.data.iter = quadrotor.data.iter + 1;

            send(quadrotor.command_publisher, msg);
        end
        
        function [quadrotor] = takeoff(quadrotor)
          quadrotor.motors(true);
          pause(0.5);
          testreq = rosmessage(quadrotor.service_clients.takeoff);
          testresp = call(quadrotor.service_clients.takeoff,testreq,'Timeout',3);
          if testresp.Success
            disp('Taking off');
            quadrotor.data.state = 'backend_control';
          end
        end

        function [quadrotor] = mhover(quadrotor)
          testreq = rosmessage(quadrotor.service_clients.hover);
          testresp = call(quadrotor.service_clients.hover,testreq,'Timeout',3);
          if testresp.Success
            disp('Hovering');
            quadrotor.data.state = 'backend_control';
          end
        end
        
        function [quadrotor] = land(quadrotor)
          testreq = rosmessage(quadrotor.service_clients.land);
          testresp = call(quadrotor.service_clients.land,testreq,'Timeout',3);
          if testresp.Success
            disp('Landing');
            quadrotor.data.state = 'backend_control';
          end
          
          pause(3);
          quadrotor.motors(false);
        end
        
        function [quadrotor] = motors(quadrotor, onoff)
          testreq = rosmessage(quadrotor.service_clients.motors);
          testreq.Data = onoff;
          testresp = call(quadrotor.service_clients.motors,testreq,'Timeout',3);
          if testresp.Success
            disp(['Motors ', num2str(onoff)]);
            quadrotor.data.state = 'backend_control';
          end
        end
        
        function [quadrotor] = transition(quadrotor, str)
          testreq = rosmessage(quadrotor.service_clients.transition);
          testreq.Tracker = str;
          call(quadrotor.service_clients.transition,testreq,'Timeout',3);
        end
               
        function save_data(quadrotor, name)
            h = quadrotor.data.history;
            save(['data/', name], 'h');
        end
        
        function plot(quadrotor)
            if ~isempty(quadrotor.data.history)
                plot_run(quadrotor.data.history);
            end
        end

        function plot_angles(quadrotor)
            if ~isempty(quadrotor.data.history)
                plot_angles(quadrotor.data.history);
            end
        end

        function plot_3d(quadrotor, wpt)
            if ~isempty(quadrotor.data.history)
                if (isempty(wpt))
                    plot_3d(quadrotor.data.history);
                else
                    plot_3d(quadrotor.data.history, wpt);
                end
            end
        end
        
    end
    
    methods (Static)
    end
end

