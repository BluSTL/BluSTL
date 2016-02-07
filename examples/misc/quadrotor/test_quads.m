addpath env
%% Create the system
QS = quad_system;
QS = init_control(QS);
QS = QS.run_open_loop(QS.controller);

%QS = QS.run_deterministic(QS.controller);


