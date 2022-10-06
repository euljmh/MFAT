clear;clc;
%% path
addpath('new\','models\','try\','work\');

%% Preprocess
tutorial_unexpected_unbalance

%% model
rot = rotfe('tutorial_unexpected_unbalance');

% 突加不平衡量设置
UNBALANCE=[8 0 5e-5;18 -5e-5 0];% 节点编号必须从小到大排列
rot = new_unbalance(rot,UNBALANCE);

% transformation
FnNode = 4;
FeNode = [8,18];

F = sort([FnNode,FeNode]);
[~,rot.FnNodeId] = ismember(FnNode,F);
[~,rot.FeNodeId] = ismember(FeNode,F);
vec = [rot.FnNodeId rot.FeNodeId;numel(rot.FNodeDir)/2+rot.FnNodeId numel(rot.FNodeDir)/2+rot.FeNodeId];
vec = reshape(vec,1,[]);

rot.FORCE_DOF = rot.FORCE_DOF(vec);

%% define parameters
logic = 0;% 是否施加控制力
freqs = 48;

%% Run Simulation
set_param('run_sim_unbalance/w_set','H',strcat(num2str(freqs),'*60'));
set_param('run_sim_unbalance/N','Value','0');
set_param('run_sim_unbalance/if_apply','Value',num2str(logic));

tic;
Simout = sim('run_sim_unbalance',30,[]);
toc;

function [rot] = new_unbalance(rot,UNBALANCE)
    
    Fu_cos=zeros(size(UNBALANCE,1)*2,1); Fu_sin = Fu_cos;
    elenum = size(UNBALANCE,1);
    for i = 1:elenum
        Fu_cos(1:2:end) = UNBALANCE(:,2); % x
        Fu_cos(2:2:end) = UNBALANCE(:,3); % y

        Fu_sin(1:2:end) = -UNBALANCE(:,3); % x - sine component
        Fu_sin(2:2:end) = UNBALANCE(:,2); % y sine
    end

    rot.new_fu_cos = Fu_cos;
    rot.new_fu_sin = Fu_sin;
end