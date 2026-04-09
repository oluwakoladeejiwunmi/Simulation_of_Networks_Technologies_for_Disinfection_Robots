%% Robot Path Model with Different Network Conditions
clc; 
clear;
close all;
rng(42);

% Global Parameters
N          = 5000;
dt         = 0.001;
No_of_trials = 50;
Kp         = 2.5;
V_nom      = 1.0;

% Reference Position
V_ref= zeros(1,N);   W_ref= zeros(1,N);
V_ref(1:1000)= 1;   W_ref(1:1000)=  0;
V_ref(1001:2000)= 1;   W_ref(1001:2000)=  1.5;
V_ref(2001:3000)= 1;   W_ref(2001:3000)=  0;
V_ref(3001:4000)= 1;   W_ref(3001:4000)= -1.5;
V_ref(4001:5000)= 1;   W_ref(4001:5000)=  0;

[X_ref, Y_ref, Theta_ref] = compute_ideal(V_ref, W_ref, N, dt);

% Network Parameters
% Ideal Network
NET(1).name  = 'Ideal';
NET(1).model = 'ideal';
NET(1).base_delay = 0;
NET(1).jitter_mu = 0;
NET(1).jitter_sig = 0;
NET(1).loss_good = 0;
NET(1).loss_bad = 0;
NET(1).p_gb = 0;
NET(1).p_bg= 1;

% 5G URLLC
NET(2).name = '5G URLLC';
NET(2).model = 'urllc';
NET(2).base_delay = 1;
NET(2).jitter_mu = 0.1;  
NET(2).jitter_sig = 0.3;
NET(2).loss_good = 1e-5;
NET(2).loss_bad = 1e-3;
NET(2).p_gb = 5e-5;
NET(2).p_bg = 0.9;

% Wi-FI 7
NET(3).name = 'WiFi 7';
NET(3).model = 'wifi';
NET(3).base_delay = 5;
NET(3).jitter_mu = 1.5;
NET(3).jitter_sig = 1.2;
NET(3).loss_good = 5e-4;
NET(3).loss_bad = 0.08;
NET(3).p_gb = 0.01;
NET(3).p_bg = 0.15;

% 4G LTE
NET(4).name = '4G LTE';
NET(4).model = 'lte';
NET(4).base_delay = 50;
NET(4).jitter_mu  = 8;
NET(4).jitter_sig = 6;
NET(4).loss_good = 0.005;
NET(4).loss_bad = 0.12;
NET(4).p_gb = 0.02;
NET(4).p_bg = 0.08;

COLORS   = {'k', [0.15 0.45 1.0], [0.10 0.72 0.20], [0.95 0.20 0.20]};
Net_Nums = numel(NET);

% Monte Carlo Simulation
CTE_all       = cell(Net_Nums, 1);
PATH_all      = cell(Net_Nums, 1);
DELAY_samples = cell(Net_Nums, 1);
fprintf('Running Monte Carlo (%d trials x %d networks)...\n\n', No_of_trials, Net_Nums);

for n = 1:Net_Nums
    cte_mat   = zeros(No_of_trials, N);
    X_mat     = zeros(No_of_trials, N);
    Y_mat     = zeros(No_of_trials, N);
    d_samp    = zeros(1, No_of_trials * 200);

    for trial = 1:No_of_trials
        [X, Y, CTE, delays] = robot_controlled( ...
            X_ref, Y_ref, Theta_ref, W_ref, NET(n), N, dt, Kp, V_nom);

        cte_mat(trial,:) = CTE;
        X_mat(trial,:)   = X;
        Y_mat(trial,:)   = Y;
        idx = (trial-1)*200 + (1:200);
        d_samp(idx) = delays(1:200);
    end

    CTE_all{n}       = cte_mat;
    PATH_all{n}      = struct('X', X_mat, 'Y', Y_mat);
    DELAY_samples{n} = d_samp;
    fprintf('  [%d/%d]  %-12s  done\n', n, Net_Nums, NET(n).name);
end


% Metrics and Statistics
fprintf('\n%-12s  %12s  %12s  %12s\n','Network','RMS CTE(m)','95th Pct(m)','Max CTE(m)');
fprintf('%s\n', repmat('-',1,56));
rms_vals  = zeros(1,Net_Nums);
pct95_vals = zeros(1,Net_Nums);
max_vals  = zeros(1,Net_Nums);
for n = 1:Net_Nums
    flat          = CTE_all{n}(:);
    rms_vals(n)   = sqrt(mean(flat.^2));
    pct95_vals(n) = prctile(flat, 95);
    max_vals(n)   = max(flat);
    fprintf('%-12s  %12.4f  %12.4f  %12.4f\n', ...
        NET(n).name, rms_vals(n), pct95_vals(n), max_vals(n));
end


% Load Sensitivity(Competing Devices)
num_devices = [1, 5, 10, 20, 50, 100];
load_rms    = zeros(Net_Nums, length(num_devices));

fprintf('Running load sensitivity...\n');
for d_idx = 1:length(num_devices)
    nd = num_devices(d_idx);
    for n = 2:Net_Nums
        net_load = NET(n);
        switch NET(n).model
            case 'urllc'
                scale = 1 + max(0,(nd-50)/50)*0.5;
                net_load.base_delay = NET(n).base_delay * scale;
                net_load.loss_good  = min(NET(n).loss_good*(1+nd/200), 0.01);
            case 'wifi'
                net_load.base_delay = NET(n).base_delay + NET(n).base_delay*0.4*(nd-1);
                net_load.jitter_mu  = NET(n).jitter_mu*(1+0.3*(nd-1));
                net_load.p_gb       = min(NET(n).p_gb*nd, 0.3);
            case 'lte'
                net_load.base_delay = NET(n).base_delay*(1+0.15*log(nd));
                net_load.jitter_mu  = NET(n).jitter_mu*(1+0.10*log(nd));
                net_load.p_gb       = min(NET(n).p_gb*(1+0.2*nd), 0.15);
        end
        acc = 0;
        for t = 1:10
            [~,~,CTE,~] = robot_controlled(X_ref,Y_ref,Theta_ref,W_ref, ...
                net_load, N, dt, Kp, V_nom);
            acc = acc + sqrt(mean(CTE.^2));
        end
        load_rms(n,d_idx) = acc/10;
    end
end



%%  Local Functions
function [X, Y, Theta] = compute_ideal(V_cmd, W_cmd, N, dt)
    x=0; y=0; theta=0;
    X=zeros(1,N); Y=zeros(1,N); Theta=zeros(1,N);
    for i=1:N
        x     = x + V_cmd(i)*cos(theta)*dt;
        y     = y + V_cmd(i)*sin(theta)*dt;
        theta = theta + W_cmd(i)*dt;
        X(i)=x; Y(i)=y; Theta(i)=theta;
    end
end

function [X, Y, CTE, delay_log] = robot_controlled( ...
        X_ref, Y_ref, Theta_ref, W_ref, net, N, dt, Kp, V_nom)

    x=0; y=0; theta=0;
    X         = zeros(1,N);
    Y         = zeros(1,N);
    Theta     = zeros(1,N);
    CTE       = zeros(1,N);
    delay_log = zeros(1,N);
    v_app = V_nom;
    w_app = 0;
    ge_state = 1;   
 
    use_jitter = (net.jitter_mu > 0) && (net.jitter_sig > 0);
    if use_jitter
        mu_j  = net.jitter_mu;
        sig_j = net.jitter_sig;
        lnsig = sqrt(log(1 + (sig_j/mu_j)^2));
        lnmu  = log(mu_j) - 0.5*lnsig^2;
    else
        lnmu  = 0;
        lnsig = 0;
    end

    base_delay_int = max(0, round(double(net.base_delay)));
    for i = 1:N

        % Stores state at start of each step
        X(i)     = x;
        Y(i)     = y;
        Theta(i) = theta;
        CTE(i)   = hypot(x - X_ref(i), y - Y_ref(i));

        if use_jitter
            raw_jitter = lognrnd(lnmu, lnsig);  
           
            if ~isfinite(raw_jitter)
                raw_jitter = 0;
            end
            jitter_int = max(0, round(raw_jitter)); 
        else
            jitter_int = 0;
        end
        total_delay = base_delay_int + jitter_int;
        delay_log(i) = total_delay;
        obs_i = max(1,  i - total_delay);  
        obs_i = min(obs_i, N);  
        x_obs  = X(obs_i);
        y_obs  = Y(obs_i);
        th_obs = Theta(obs_i);
        dx = X_ref(obs_i) - x_obs;
        dy = Y_ref(obs_i) - y_obs;
        if hypot(dx,dy) > 0.01
            theta_des = atan2(dy, dx);
        else
            theta_des = Theta_ref(obs_i);
        end
        h_err = atan2(sin(theta_des - th_obs), cos(theta_des - th_obs));
        v_new = V_nom;
        w_new = W_ref(obs_i) + Kp * h_err;

        % Gilbert-Elliott packet loss
        if ge_state == 1
            loss_prob    = net.loss_good;
            p_transition = net.p_gb;
        else
            loss_prob    = net.loss_bad;
            p_transition = net.p_bg;
        end
        if rand() < p_transition
            ge_state = 3 - ge_state;   
        end
        if rand() >= loss_prob
            v_app = v_new;
            w_app = w_new;
        end
 
        % Unicycle kinematics 
        x     = x + v_app*cos(theta)*dt;
        y     = y + v_app*sin(theta)*dt;
        theta = theta + w_app*dt;

    end
end

%% Graphs
% Figure 1 — Mean Paths
figure('Name','Robot Paths','Position',[30 530 820 470]);
plot(X_ref, Y_ref,'k-','LineWidth',3.5,'DisplayName','Reference (Ideal)');
hold on;
for n = 2:Net_Nums
    plot(mean(PATH_all{n}.X,1), mean(PATH_all{n}.Y,1), '--', ...
        'Color',COLORS{n},'LineWidth',2.2, ...
        'DisplayName',sprintf('%s (base=%dms)',NET(n).name,NET(n).base_delay));
end
plot(X_ref(1),Y_ref(1),'go','MarkerSize',12,'MarkerFaceColor','g','HandleVisibility','off');
plot(X_ref(end),Y_ref(end),'rs','MarkerSize',12,'MarkerFaceColor','r','HandleVisibility','off');
text(X_ref(1)+0.05,   Y_ref(1)-0.12,   'Start','FontSize',10);
text(X_ref(end)+0.05, Y_ref(end)-0.12, 'End',  'FontSize',10);
legend('Location','best','FontSize',11);
xlabel('X axis(m)','FontSize',12); ylabel('Y axis(m)','FontSize',12);
title('Average Robot Path with Different Networks(50 Simulations)','FontSize',13);
grid on; axis equal;

% Figure 2 — Average CTE
t_vec = (1:N)*dt;
figure('Name','Cross-Track Error','Position',[30 80 820 400]);
hold on;
for n = 2:Net_Nums
    mu    = mean(CTE_all{n},1);
    sigma = std(CTE_all{n},0,1);
    fill([t_vec,fliplr(t_vec)],[mu+sigma,fliplr(max(mu-sigma,0))], ...
        COLORS{n},'FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
    plot(t_vec, mu,'Color',COLORS{n},'LineWidth',2.0,'DisplayName',NET(n).name);
end
legend('Location','northwest','FontSize',11);
xlabel('Time (s)','FontSize',12); ylabel('CTE (m)','FontSize',12);
title('Average Cross Track Error (50 Simulations)','FontSize',13);
grid on;

% Figure 3 — Bar chart of Avg error vs 95th percentile error
figure('Name','Summary Metrics','Position',[870 530 540 420]);
plot_idx = 2:Net_Nums;
bar_data = [rms_vals(plot_idx); pct95_vals(plot_idx)]';
b = bar(bar_data,'grouped');
b(1).FaceColor = [0.20 0.60 1.00];   
b(2).FaceColor = [0.95 0.45 0.10];   

ax = gca;
set(ax,'XTickLabel',{NET(plot_idx).name},'FontSize',12,'XTickLabelRotation',0);
ylabel('CTE (m)','FontSize',12);
title('Average error vs 95th percentile error','FontSize',13);
subtitle(sprintf('p-value = %.2e', p_anova),'FontSize',10,'Color',[0.3 0.3 0.3]);
legend({'Avg CTE','95th Pct'},'Location','northwest','FontSize',11);
grid on;

ymax = max(pct95_vals(plot_idx))*1.2;
ylim([0 ymax]);

for k=1:numel(plot_idx)
    text(k-0.15, rms_vals(plot_idx(k))+ymax*0.02,  sprintf('%.3f',rms_vals(plot_idx(k))), ...
        'HorizontalAlignment','center','FontSize',9,'FontWeight','bold');
    text(k+0.15, pct95_vals(plot_idx(k))+ymax*0.02, sprintf('%.3f',pct95_vals(plot_idx(k))), ...
        'HorizontalAlignment','center','FontSize',9,'FontWeight','bold');
end


% Figure 4 — Load sensitivity vs Competing devices
figure('Name','Load Sensitivity','Position',[30 80 820 400]);
hold on;
for n = 2:Net_Nums
    plot(num_devices, load_rms(n,:),'o-','Color',COLORS{n},'LineWidth',2.2, ...
        'MarkerSize',7,'MarkerFaceColor',COLORS{n},'DisplayName',NET(n).name);
end
legend('Location','northwest','FontSize',11);
xlabel('Number of Competing Devices','FontSize',12);
ylabel('Avg Cross-Track Error(m)','FontSize',12);
title('Load Sensitivity: Avg CTE vs Competing Devices','FontSize',13);
grid on; set(gca,'XScale','log');
xticks(num_devices);
xticklabels(arrayfun(@num2str, num_devices,'UniformOutput',false));

% Figure 5 — Zoomed turn(Focused on first turn)
figure('Name','Turn Zoom','Position',[870 530 540 400]);
plot(X_ref(1001:2000),Y_ref(1001:2000),'k-','LineWidth',3.5,'DisplayName','Reference');
hold on;
for n = 2:Net_Nums
    plot(mean(PATH_all{n}.X(:,1001:2000),1), ...
         mean(PATH_all{n}.Y(:,1001:2000),1), ...
        '--','Color',COLORS{n},'LineWidth',2.2,'DisplayName',NET(n).name);
end
legend('Location','best','FontSize',11);
xlabel('X axis(m)','FontSize',12); ylabel('Y axis(m)','FontSize',12);
title('Zoomed: Left Turn (t = 1–2 s)','FontSize',13);
grid on; axis equal;