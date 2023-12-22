addpath(fullfile('..', 'src'));

            for i = 1 : delay
            %    x_ = RK4(x_,obj.mem_u,Ts,obj.rocket);
        x_ = x_ + Ts*obj.rocket.f(x_,obj.mem_u);
            end
%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable