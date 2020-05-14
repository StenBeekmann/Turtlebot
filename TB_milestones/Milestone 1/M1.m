clc
close all
clear all

%%
% load file
load identification_data/v_step_0_w_step_0.5_enc.mat
tick_left = all_enc_left;
tick_right = all_enc_right;
t = all_time;

[s_car1,th_car1,sdot1,thdot1,t1]=ModelValues(tick_left,tick_right,t);
 % plot of encoder values
    figure()
    subplot(2,1,1)
    plot(all_time,all_enc_left)
    title 'enc-left'
    grid minor
    subplot(2,1,2)
    plot(all_time,all_enc_right)
    title 'enc-right'
    grid minor

    % plot position & rotational position
    figure()
    subplot(2,1,1)
    plot(t1,s_car1)
    title('s vs time')
    ylim([0,1])
    subplot(2,1,2)
    plot(t1,th_car1)
    title('th vs time')

    % Plot of V & W values
    figure()
    subplot(2,1,1)
    plot(t1(1,1:54),sdot1(1,1:54))
    title 'velocity'
    grid minor
    ylim([0,1])
    subplot(2,1,2)
    plot(t1(1,1:54),thdot1(1,1:54))
    title 'rotational velocity'
    grid minor
%% Step W 1
load identification_data/v_step_0_w_step_1_enc.mat
tick_left = all_enc_left;
tick_right = all_enc_right;
t = all_time;
[s_car2,th_car2,sdot2,thdot2,t2]=ModelValues(tick_left,tick_right,t);

   % Plot of V & W values
    figure()
    subplot(2,1,1)
    plot(t2(1,1:54),sdot2(1,1:54))
    title 'velocity'
    grid minor
    ylim([0,1])
    subplot(2,1,2)
    plot(t2(1,1:54),thdot2(1,1:54))
    title 'rotational velocity'
    grid minor




        
        
        