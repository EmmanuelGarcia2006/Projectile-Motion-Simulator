
function ProjectileMotionApp

clc;

%% CREATE APPLICATION WINDOW

fig = uifigure('Name','Projectile Motion Simulator', ...
    'Position',[100 100 1100 720], ...
    'Color',[0.95 0.96 0.98]);

% Main title
uilabel(fig, ...
    'Text','PROJECTILE MOTION SIMULATOR', ...
    'Position',[300 665 500 35], ...
    'FontSize',22, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center');

uilabel(fig, ...
    'Text','Compare projectile motion with and without air resistance', ...
    'Position',[300 640 500 25], ...
    'FontSize',12, ...
    'HorizontalAlignment','center');

%% INPUT PANEL

panel = uipanel(fig, ...
    'Title','Projectile Input Parameters', ...
    'Position',[20 390 300 230], ...
    'FontSize',14);

% Initial velocity
uilabel(panel,'Text','Initial Velocity (m/s)', ...
    'Position',[15 165 160 25]);

velocityInput = uieditfield(panel,'numeric', ...
    'Position',[175 165 100 25], ...
    'Value',25,'Limits',[0.01 Inf]);

% Launch angle
uilabel(panel,'Text','Launch Angle (degrees)', ...
    'Position',[15 125 160 25]);

angleInput = uieditfield(panel,'numeric', ...
    'Position',[175 125 100 25], ...
    'Value',45,'Limits',[1 89]);

% Mass
uilabel(panel,'Text','Mass (kg)', ...
    'Position',[15 85 160 25]);

massInput = uieditfield(panel,'numeric', ...
    'Position',[175 85 100 25], ...
    'Value',0.50,'Limits',[0.001 Inf]);

% Surface area
uilabel(panel,'Text','Surface Area (m^2)', ...
    'Position',[15 45 160 25]);

areaInput = uieditfield(panel,'numeric', ...
    'Position',[175 45 100 25], ...
    'Value',0.01,'Limits',[0.000001 Inf]);

%% CALCULATE BUTTON

calculateButton = uibutton(fig,'push', ...
    'Text','CALCULATE PROJECTILE MOTION', ...
    'Position',[35 340 270 40], ...
    'FontSize',13, ...
    'FontWeight','bold', ...
    'BackgroundColor',[0.1 0.45 0.75], ...
    'FontColor',[1 1 1], ...
    'ButtonPushedFcn',@calculateMotion);

%% INPUT INFORMATION DISPLAY

infoLabel = uilabel(fig, ...
    'Text','Enter your projectile data and click Calculate.', ...
    'Position',[350 575 700 35], ...
    'FontSize',12, ...
    'FontWeight','bold');

%% RESULTS TABLE

uilabel(fig, ...
    'Text','SIMULATION RESULTS', ...
    'Position',[350 530 300 25], ...
    'FontSize',15, ...
    'FontWeight','bold');

resultsTable = uitable(fig, ...
    'Position',[350 425 700 100], ...
    'ColumnName',{'Measurement','Without Air Resistance', ...
                  'With Air Resistance'}, ...
    'ColumnWidth',{200 220 220}, ...
    'Data',{'Maximum Height (m)',0,0; ...
            'Flight Time (s)',0,0; ...
            'Horizontal Range (m)',0,0});

%% TRAJECTORY GRAPH

trajectoryAxes = uiaxes(fig, ...
    'Position',[30 65 500 280]);

title(trajectoryAxes,'Projectile Trajectory');
xlabel(trajectoryAxes,'Horizontal Distance (m)');
ylabel(trajectoryAxes,'Vertical Height (m)');
grid(trajectoryAxes,'on');

%% SPEED GRAPH

speedAxes = uiaxes(fig, ...
    'Position',[570 65 500 280]);

title(speedAxes,'Projectile Speed Comparison');
xlabel(speedAxes,'Time (seconds)');
ylabel(speedAxes,'Speed (m/s)');
grid(speedAxes,'on');

%% MAIN CALCULATION FUNCTION

    function calculateMotion(~,~)

        % Read user inputs

        v0 = velocityInput.Value;
        theta = angleInput.Value;
        m = massInput.Value;
        A = areaInput.Value;

        % Physics constants

        g = 9.81;
        rho = 1.225;
        Cd = 0.47;

        % Initial velocity components

        vx0 = v0*cosd(theta);
        vy0 = v0*sind(theta);

        %% WITHOUT AIR RESISTANCE

        T_noAir = 2*vy0/g;

        t_noAir = linspace(0,T_noAir,300);

        x_noAir = vx0*t_noAir;

        y_noAir = vy0*t_noAir ...
            - 0.5*g*t_noAir.^2;

        maxHeight_noAir = max(y_noAir);
        range_noAir = max(x_noAir);

        vx_noAir = vx0*ones(size(t_noAir));
        vy_noAir = vy0-g*t_noAir;

        speed_noAir = sqrt(vx_noAir.^2+vy_noAir.^2);

        %% WITH AIR RESISTANCE

        initialConditions = [0;0;vx0;vy0];

        options = odeset( ...
            'Events',@hitGround, ...
            'RelTol',1e-8, ...
            'AbsTol',1e-10);

        [t_air,Y] = ode45( ...
            @(t,Y) projectileODE(t,Y,m,A,rho,Cd,g), ...
            [0 100],initialConditions,options);

        x_air = Y(:,1);
        y_air = Y(:,2);

        vx_air = Y(:,3);
        vy_air = Y(:,4);

        speed_air = sqrt(vx_air.^2+vy_air.^2);

        maxHeight_air = max(y_air);
        range_air = x_air(end);
        flightTime_air = t_air(end);

        %% UPDATE RESULTS TABLE

        resultsTable.Data = {
            'Maximum Height (m)', ...
            maxHeight_noAir,maxHeight_air;

            'Flight Time (s)', ...
            T_noAir,flightTime_air;

            'Horizontal Range (m)', ...
            range_noAir,range_air
            };

        %% DISPLAY INPUT INFORMATION

        infoLabel.Text = sprintf( ...
            'Velocity: %.2f m/s | Angle: %.2f degrees | Mass: %.3f kg | Area: %.4f m^2', ...
            v0,theta,m,A);

        %% UPDATE TRAJECTORY GRAPH

        cla(trajectoryAxes);

        plot(trajectoryAxes,x_noAir,y_noAir, ...
            'b-','LineWidth',2);

        hold(trajectoryAxes,'on');

        plot(trajectoryAxes,x_air,y_air, ...
            'r--','LineWidth',2);

        legend(trajectoryAxes, ...
            {'Without Air Resistance','With Air Resistance'}, ...
            'Location','best');

        title(trajectoryAxes,'Projectile Trajectory');
        xlabel(trajectoryAxes,'Horizontal Distance (m)');
        ylabel(trajectoryAxes,'Vertical Height (m)');

        grid(trajectoryAxes,'on');
        hold(trajectoryAxes,'off');

        %% UPDATE SPEED GRAPH

        cla(speedAxes);

        plot(speedAxes,t_noAir,speed_noAir, ...
            'b-','LineWidth',2);

        hold(speedAxes,'on');

        plot(speedAxes,t_air,speed_air, ...
            'r--','LineWidth',2);

        legend(speedAxes, ...
            {'Without Air Resistance','With Air Resistance'}, ...
            'Location','best');

        title(speedAxes,'Projectile Speed Comparison');
        xlabel(speedAxes,'Time (seconds)');
        ylabel(speedAxes,'Speed (m/s)');

        grid(speedAxes,'on');
        hold(speedAxes,'off');

        %% DISPLAY COMPLETION MESSAGE

        disp('PROJECTILE MOTION RESULTS');
        fprintf('Velocity: %.2f m/s\n',v0);
        fprintf('Angle: %.2f degrees\n',theta);
        fprintf('Mass: %.3f kg\n',m);
        fprintf('Surface Area: %.4f m^2\n',A);

        fprintf('\nWITHOUT AIR RESISTANCE\n');
        fprintf('Maximum Height: %.2f m\n',maxHeight_noAir);
        fprintf('Flight Time: %.2f s\n',T_noAir);
        fprintf('Horizontal Range: %.2f m\n',range_noAir);

        fprintf('\nWITH AIR RESISTANCE\n');
        fprintf('Maximum Height: %.2f m\n',maxHeight_air);
        fprintf('Flight Time: %.2f s\n',flightTime_air);
        fprintf('Horizontal Range: %.2f m\n',range_air);

    end

end

%% PROJECTILE MOTION DIFFERENTIAL EQUATIONS

function dY = projectileODE(~,Y,m,A,rho,Cd,g)

    vx = Y(3);
    vy = Y(4);

    speed = sqrt(vx^2+vy^2);

    drag = 0.5*rho*Cd*A*speed^2;

    if speed > 0
        ax = -(drag/m)*(vx/speed);
        ay = -g-(drag/m)*(vy/speed);
    else
        ax = 0;
        ay = -g;
    end

    dY = [vx;vy;ax;ay];

end

%% STOP SIMULATION WHEN PROJECTILE HITS GROUND

function [value,isterminal,direction] = hitGround(~,Y)

    value = Y(2);

    isterminal = 1;

    direction = -1;

end
