close all
clear
restoredefaultpath; matlabrc;

%% Set-up path
frost_path = 'D:\Software\MATLAB_lib\frost-dev-master';
addpath(frost_path)

frost_addpath % Initialized frost

export_path = 'gen/opt';
utils.init_path(export_path)

%% robot model setting:
cur = utils.get_root_path();
urdf=fullfile(cur,'urdf','Slide_pendu.urdf');

delay_set = false;
load_sym  = false;
if load_sym    
    load_path   = 'gen/sym'; 
    utils.init_path(load_path);
else
    load_path   = []; 
end

%% load the robot model
robot = sys.LoadModel(urdf, load_path, delay_set);
%%
bound0=opt.GetBounds(robot);
RC=bound0.RightStance;
time=RC.time;
states=RC.states;
gains= RC.gains;

bounds=struct('time',RC.time,'states',RC.states,'inputs',RC.inputs,...
    'prams',RC.params,...
    'kp',RC.gains.kp,...
    'kd',RC.gains.kd,...
    'rd',0);
opts = struct('ConstantTimeHorizon',[0,1]',... $NaN - variable time
    'DerivativeLevel',1,... % either 1 (only Jacobian) or 2 (both Jacobian and Hessian)
    'EqualityConstraintBoundary',0); % non-zero positive small value will relax the equality constraints

nlp = TrajectoryOptimization('Trial',robot,2,bounds,opts);

%%
Trial_cost(nlp, bounds);
nlp.update()
nlp_save=nlp;
%%
solver = IpoptApplication(nlp);