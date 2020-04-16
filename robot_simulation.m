% double nested loopy boi
A = zeros(20,3);
%B = zeros(20,3);
for i = 1:3
    parfor z = 1:20
        control_type = i;
        disp(z);
        botscore(z,i) = robot_sim(control_type);
        A(z,i) = botscore(z,i);
        %B(i,z) = botscore(1,2);
    end
    csvwrite('data.csv',A);
    disp(A);
    %disp(B);
end

function botscore = robot_sim(control_type)
    addpath('Setup Simulation');
    addpath('Execute Simulation');
    addpath('Execute Simulation/Control');
    addpath('Plot');

    % load parameters
    robot = load_physical_parameters();
    control = load_control_parameters(control_type);
    simulation_params = load_simulation_parameters();
    impulse_response = load_impulse_response(robot, simulation_params);

    % initialize variables
    sim = initialize_simulation_data(robot, control, impulse_response, simulation_params);

    % go through the simulation and control the robot
    for n = 2:length(sim.time)
        sim = update_control(sim);
        sim = update_simulation(sim);
    end

    % plot results and animation
    plot_animation(sim);
    plot_performance(sim);
    plot_impulse_response(sim);
    
    % gather score
    botscore = (sim.score(n));
    %botscore(1,2) = sim.position;
end