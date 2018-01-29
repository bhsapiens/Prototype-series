function nlp = Trial_cost(nlp, bounds, varargin)

% add custom cost function
sys = nlp.Plant;
dx = sys.States.dx;

% 1. target states: 200*(x_f-x_des)'*P1*(x_f-x_des)
rd = bounds.rd;
x_des_val = [0,pi/2];
x_f   = sys.States.x;
x_d = SymVariable('xd',[2,1]);

tar_pos = 100*(x_f-x_d).'*(x_f-x_d);
tar_pos2 = 100*(x_f).'*x_f;

tar_pos_fn = SymFunction('target_pos',tar_pos,{x_f},{x_d});
tar_pos_fn2 = SymFunction('target_pos',tar_pos2,{x_f});

nlp = addNodeCost(nlp, tar_pos_fn, 'x', 'last', x_des_val);
nlp = addRunningCost(nlp, tar_pos_fn2, {'x'});
end