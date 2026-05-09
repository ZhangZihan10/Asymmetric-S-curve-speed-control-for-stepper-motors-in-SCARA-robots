function traj = plan_multiwaypoint_asym_scurve(waypoints, cfg)
%PLAN_MULTIWAYPOINT_ASYM_SCURVE Multi-waypoint asymmetric S-curve planner.
%   traj = plan_multiwaypoint_asym_scurve(waypoints, cfg)
%   waypoints: [N x 1] position points (step unit or rad)
%   cfg fields:
%       Ts        sample time (s)
%       v_max     max speed
%       a_acc     max accel in acceleration part (>0)
%       a_dec     max accel in deceleration part (>0)
%       j_acc     max jerk in acceleration part (>0)
%       j_dec     max jerk in deceleration part (>0)
%       v_start   start speed (optional, default 0)
%       v_end     end speed   (optional, default 0)
%
%   output struct fields:
%       t, pos, vel, acc, jerk, seg_id

arguments
    waypoints (:,1) double
    cfg struct
end

required = {'Ts','v_max','a_acc','a_dec','j_acc','j_dec'};
for k = 1:numel(required)
    if ~isfield(cfg, required{k})
        error('Missing cfg.%s', required{k});
    end
end

Ts = cfg.Ts;
v_max = cfg.v_max;
a_acc = abs(cfg.a_acc);
a_dec = abs(cfg.a_dec);
j_acc = abs(cfg.j_acc);
j_dec = abs(cfg.j_dec);

v_start = 0;
v_end = 0;
if isfield(cfg,'v_start'), v_start = cfg.v_start; end
if isfield(cfg,'v_end'), v_end = cfg.v_end; end

if numel(waypoints) < 2
    error('waypoints must contain at least 2 points');
end

d = diff(waypoints);
dir = sign(d);
dir(dir==0) = 1;
L = abs(d);
M = numel(L);

% Corner speed limit from neighboring segment lengths
v_wp = zeros(M+1,1);
v_wp(1) = abs(v_start);
v_wp(end) = abs(v_end);
for i = 2:M
    v_wp(i) = min(v_max, sqrt(2*min(a_acc,a_dec)*min(L(i-1),L(i))));
end

% Forward-backward pass to satisfy distance feasibility
for i = 1:M
    v_wp(i+1) = min(v_wp(i+1), sqrt(max(0,v_wp(i)^2 + 2*a_acc*L(i))));
end
for i = M:-1:1
    v_wp(i) = min(v_wp(i), sqrt(max(0,v_wp(i+1)^2 + 2*a_dec*L(i))));
end

% Build full trajectory arrays
pos = waypoints(1);
vel = v_wp(1)*dir(1);
acc = 0;
jerk = 0;
t = 0;
seg_id = 1;

for i = 1:M
    vi = v_wp(i);
    vf = v_wp(i+1);
    vmax_seg = v_max;
    Li = L(i);
    sgn = dir(i);

    % Find reachable cruise speed for this segment
    v_peak_low = max(vi, vf);
    v_peak_high = v_max;
    for it = 1:40
        vmid = 0.5*(v_peak_low + v_peak_high);
        s_need = dist_with_jerk_limit(vi, vmid, a_acc, j_acc) + ...
                 dist_with_jerk_limit(vf, vmid, a_dec, j_dec);
        if s_need > Li
            v_peak_high = vmid;
        else
            v_peak_low = vmid;
        end
    end
    v_peak = min(vmax_seg, v_peak_low);

    s_acc = dist_with_jerk_limit(vi, v_peak, a_acc, j_acc);
    s_dec = dist_with_jerk_limit(vf, v_peak, a_dec, j_dec);
    s_cruise = max(0, Li - s_acc - s_dec);

    [a_acc_prof, j_acc_prof] = accel_profile(vi, v_peak, a_acc, j_acc, Ts);
    n_acc = numel(a_acc_prof);

    v_tmp = vi;
    s_tmp = 0;
    for k = 1:n_acc
        a_k = a_acc_prof(k);
        j_k = j_acc_prof(k);
        v_next = max(0, v_tmp + a_k*Ts);
        ds = 0.5*(v_tmp + v_next)*Ts;
        s_tmp = s_tmp + ds;
        v_tmp = v_next;
        pos(end+1,1) = pos(end) + sgn*ds; %#ok<AGROW>
        vel(end+1,1) = sgn*v_tmp; %#ok<AGROW>
        acc(end+1,1) = sgn*a_k; %#ok<AGROW>
        jerk(end+1,1) = sgn*j_k; %#ok<AGROW>
        t(end+1,1) = t(end) + Ts; %#ok<AGROW>
        seg_id(end+1,1) = i; %#ok<AGROW>
    end

    if s_cruise > 0
        n_cr = max(1, round((s_cruise/max(v_peak,1e-9))/Ts));
        for k = 1:n_cr
            ds = v_peak*Ts;
            pos(end+1,1) = pos(end) + sgn*ds; %#ok<AGROW>
            vel(end+1,1) = sgn*v_peak; %#ok<AGROW>
            acc(end+1,1) = 0; %#ok<AGROW>
            jerk(end+1,1) = 0; %#ok<AGROW>
            t(end+1,1) = t(end) + Ts; %#ok<AGROW>
            seg_id(end+1,1) = i; %#ok<AGROW>
        end
    end

    [a_dec_prof, j_dec_prof] = accel_profile(v_peak, vf, a_dec, j_dec, Ts);
    n_dec = numel(a_dec_prof);
    v_tmp = v_peak;
    for k = 1:n_dec
        a_k = -a_dec_prof(k);
        j_k = -j_dec_prof(k);
        v_next = max(0, v_tmp + a_k*Ts);
        ds = 0.5*(v_tmp + v_next)*Ts;
        v_tmp = v_next;
        pos(end+1,1) = pos(end) + sgn*ds; %#ok<AGROW>
        vel(end+1,1) = sgn*v_tmp; %#ok<AGROW>
        acc(end+1,1) = sgn*a_k; %#ok<AGROW>
        jerk(end+1,1) = sgn*j_k; %#ok<AGROW>
        t(end+1,1) = t(end) + Ts; %#ok<AGROW>
        seg_id(end+1,1) = i; %#ok<AGROW>
    end

    % Force end-point correction at each waypoint
    pos(end) = waypoints(i+1);
    vel(end) = sgn*vf;
end

traj = struct('t', t, 'pos', pos, 'vel', vel, 'acc', acc, 'jerk', jerk, ...
              'seg_id', seg_id, 'v_wp', v_wp);
end

function s = dist_with_jerk_limit(v0, v1, amax, jmax)
if v1 <= v0
    s = 0;
    return;
end
Dv = v1 - v0;
Tj = amax/jmax;
Dv_tri = amax^2/jmax;
if Dv <= Dv_tri
    % Triangular accel profile
    apeak = sqrt(Dv*jmax);
    T = apeak/jmax;
    s = 2*(v0*T + (1/6)*jmax*T^3) + (v1 - Dv/2)*0;
    % Compact closed-form for triangular jerk-limited acceleration distance
    s = (v0 + v1)*T;
else
    Tc = (Dv - Dv_tri)/amax;
    s1 = v0*Tj + (1/6)*jmax*Tj^3;
    v_t1 = v0 + 0.5*jmax*Tj^2;
    s2 = v_t1*Tc + 0.5*amax*Tc^2;
    v_t2 = v_t1 + amax*Tc;
    s3 = v_t2*Tj + 0.5*amax*Tj^2 - (1/6)*jmax*Tj^3;
    s = s1 + s2 + s3;
end
end

function [a_prof, j_prof] = accel_profile(v0, v1, amax, jmax, Ts)
if v1 <= v0 + 1e-12
    a_prof = zeros(0,1);
    j_prof = zeros(0,1);
    return;
end

Dv = v1 - v0;
Tj = amax/jmax;
Dv_tri = amax^2/jmax;

if Dv <= Dv_tri
    apeak = sqrt(Dv*jmax);
    Tj_use = apeak/jmax;
    n1 = max(1, ceil(Tj_use/Ts));
    n2 = n1;
    a_up = linspace(0, apeak, n1)';
    a_dn = linspace(apeak, 0, n2)';
    a_prof = [a_up; a_dn];
else
    Tc = (Dv - Dv_tri)/amax;
    n1 = max(1, ceil(Tj/Ts));
    n2 = max(1, ceil(Tc/Ts));
    n3 = n1;
    a_up = linspace(0, amax, n1)';
    a_ct = amax*ones(n2,1);
    a_dn = linspace(amax, 0, n3)';
    a_prof = [a_up; a_ct; a_dn];
end

j_prof = [diff(a_prof)/Ts; 0];
end
