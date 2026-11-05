% run_all_tests.m  -- batch tests for the PV-BESS AFPID paper (Simulink only).
% Fill in the CONFIG block, then run section by section. Each section writes a CSV
% that can be pasted into the paper tables. Nothing here has been run by the
% author of this script (no MATLAB/Simulink available); treat as a template.
%% CONFIG ------------------------------------------------------------------
mdl   = 'YOUR_MODEL';                 % Simulink model name
ctrls = {'PI','PO','INC','FPID','AFPID'};   % set the controller-select variable below
ctrlVar = 'ctrl_sel';                 % workspace variable/constant that picks the controller (1..5)
sig.Vbus = 'Vdcbus_pu'; sig.Vpv='Vpv_pu'; sig.Ipv='Ipv_pu'; sig.Ppv='Ppv'; sig.Pmpp='Pmpp'; sig.Ibat='Ibat'; sig.Vref=1.0;
Tsim = 6.0;                           % s (paper); ode45, MaxStep 1e-6 as in Table VI
load_open(mdl);
met = @(t,v,ref,tstep) struct( ...
   'dev',  max(abs(v(t>=tstep)-ref)), ...
   'ts2',  settle(t,v,ref,tstep,0.02), ...
   'IAE',  trapz(t(t>=tstep),abs(ref-v(t>=tstep))), ...
   'ISE',  trapz(t(t>=tstep),(ref-v(t>=tstep)).^2));
%% 1. Irradiance step 1000->600 W/m2 at t=3.0 s  -----------------------------
R=table();
for c=1:numel(ctrls)
  si = Simulink.SimulationInput(mdl); si=si.setVariable(ctrlVar,c);
  si=si.setVariable('G_step_time',3.0).setVariable('G_before',1000).setVariable('G_after',600);
  so=sim(si); [t,Vb]=getsig(so,sig.Vbus);
  m=met(t,Vb,sig.Vref,3.0); eta=mpptEff(so,sig,3.0);   % define mpptEff: int(Ppv)/int(Pmpp)
  R=[R; table(string(ctrls{c}),m.dev,m.ts2,m.IAE,m.ISE,eta,'VariableNames',{'ctrl','dev','ts','IAE','ISE','MPPTeff'})]; %#ok<AGROW>
end
writetable(R,'viC_irradiance.csv');
%% 2. Load step: +20 % in RL at t=4.5 s ---------------------------------------
R=table();
for c=1:numel(ctrls)
  si=Simulink.SimulationInput(mdl).setVariable(ctrlVar,c).setVariable('RL_step_time',4.5).setVariable('RL_gain',1.2);
  so=sim(si); [t,Vb]=getsig(so,sig.Vbus); m=met(t,Vb,sig.Vref,4.5);
  R=[R; table(string(ctrls{c}),m.dev,m.ts2,m.IAE,m.ISE,mpptEff(so,sig,4.5),'VariableNames',{'ctrl','dev','ts','IAE','ISE','MPPTeff'})]; %#ok<AGROW>
end
writetable(R,'viC_load.csv');
%% 3. SOC clamped at 20 % and 90 % (state the initial SOC in Table VI) ---------
R=table();
for soc=[0.20 0.90]
 for c=1:numel(ctrls)
  si=Simulink.SimulationInput(mdl).setVariable(ctrlVar,c).setVariable('SOC_init',soc).setVariable('SOC_min',0.20).setVariable('SOC_max',0.90);
  so=sim(si); [t,Vb]=getsig(so,sig.Vbus); m=met(t,Vb,sig.Vref,1.0);   % black-start instant
  R=[R; table(soc,string(ctrls{c}),m.dev,m.ts2,m.IAE,m.ISE,'VariableNames',{'SOC','ctrl','dev','ts','IAE','ISE'})]; %#ok<AGROW>
 end
end
writetable(R,'viC_soc.csv');
%% 4. Sensitivity: +-20 % on L, C, Rb, PV constants + irradiance noise --------
% Uniform random +-20 % per run, N runs per controller; report mean +- std and min..max.
N=30; rng(1); names={'L_PV','L_b','L_L','C1','C2','C3','Rb','Rp','Ipv','pv_a','pv_beta'};
nom=cellfun(@(n) evalin('base',n),names);
R=table();
for c=1:numel(ctrls)
 for k=1:N
  si=Simulink.SimulationInput(mdl).setVariable(ctrlVar,c);
  for j=1:numel(names), si=si.setVariable(names{j},nom(j)*(1+0.4*(rand-0.5))); end
  si=si.setVariable('noise_on',1).setVariable('noise_seed',k).setVariable('noise_sigma',0.01); % 1 % of 1000 W/m2, add a band-limited noise block on irradiance
  so=sim(si); [t,Vb]=getsig(so,sig.Vbus); m=met(t,Vb,sig.Vref,1.0);
  R=[R; table(string(ctrls{c}),k,m.ts2,m.IAE,m.ISE,mpptEff(so,sig,1.0),'VariableNames',{'ctrl','run','ts','IAE','ISE','MPPTeff'})]; %#ok<AGROW>
 end
end
writetable(R,'sensitivity_runs.csv');
S=groupsummary(R,'ctrl',{'mean','std','min','max'},{'ts','IAE','ISE','MPPTeff'}); writetable(S,'sensitivity_summary.csv');
% also do one-at-a-time +-20 % (tornado) if the reviewer wants attribution per parameter.
%% 5. Power quality: ripple and THD at steady state (use a short fixed-step window)
% Log Vdcbus, iL_PV (not averaged) and the inverter current over e.g. t=5.0-5.2 s (10 fundamental cycles).
c=6-1; si=Simulink.SimulationInput(mdl).setVariable(ctrlVar,5); so=sim(si);
[t,Vb]=getsig(so,'Vdcbus_V'); [~,iL]=getsig(so,'iL_PV'); [~,ia]=getsig(so,'i_inv_a');
w=t>=5.0&t<=5.2; tt=linspace(5.0,5.2,2e5)'; Vb=interp1(t(w),Vb(w),tt); iL=interp1(t(w),iL(w),tt); ia=interp1(t(w),ia(w),tt);
rip.Vpp=max(Vb)-min(Vb); rip.iLpp=max(iL)-min(iL);
% THD of inverter current: 50 Hz, harmonics up to 50th
fs=1/(tt(2)-tt(1)); n=round(0.2*50)*round(fs/50/ (fs/50)); thd=thdOf(ia,fs,50,50);
disp(rip); fprintf('THD = %.2f %%\n',100*thd);
% run for FPID too (ctrl 4) and compare against Eq.(7)/(14c): dI = Vpv*d/(L*fsw)
%% 6. Averaged / small-signal vs switched step response ------------------------
% a) switched: Simulink model with the real PWM blocks, apply +5 % step in Vref at t=2 s
% b) averaged: same model with the converter replaced by Averaged Switch blocks (or the ss model)
% c) small-signal: closed-loop T(s)=Lo/(1+Lo) from Eq.(31); step data in smallsignal_step_Ki0.77_R14.4.csv
%    (computed from Table VI values, R=14.4 ohm; replace by linmod/linearize of your model).
ss=readmatrix('smallsignal_step_Ki0.77_R14.4.csv');
% so_sw=sim(...switched...); so_av=sim(...averaged...);
% figure; plot(t_sw,Vb_sw,t_av,Vb_av,'--',ss(:,1)+2,1+0.05*(ss(:,2)-1)... ,':'); legend('switched','averaged','small-signal');
% Linearized margins to replace Table II:
%   io=linio(mdl,'Vref_in','input'); io(2)=linio(mdl,'Vbus_meas','output'); sys=linearize(mdl,io);
%   [Gm,Pm,Wcg,Wcp]=margin(sys); allmargin(sys)
%% 7. Dump the FIS rule base for Table VIII --------------------------------------
% fis=readfis('fuzzypid.fis'); showrule(fis,1:49,'verbose'); for k=1:3, fprintf('%s: range [%g %g]\n',fis.Outputs(k).Name,fis.Outputs(k).Range); end
% for each input/output MF: [fis.Inputs(1).MembershipFunctions.Type] and .Parameters give exact break-points for Table VII.
%% 8. Parameter checks asked about (Section 6 of the to-do list) ---------------
% get_param(mdl,'Solver'), get_param(mdl,'MaxStep')   % expect 'ode45' and '1e-6'
% evalin('base','Rb'), evalin('base','C2')            % confirm Rb = 1 ohm
% PV array block: get_param([mdl '/PV Array'],'Ns'), get_param([mdl '/PV Array'],'Np')
% battery block initial SOC: get_param([mdl '/Battery'],'SOC')
%% helpers ----------------------------------------------------------------------
function [t,v]=getsig(so,name), s=so.logsout.get(name).Values; t=s.Time; v=s.Data; end
function ts=settle(t,v,ref,t0,band), i=t>=t0; tt=t(i); e=abs(v(i)-ref)>band*abs(ref); k=find(e,1,'last'); if isempty(k), ts=0; else, ts=tt(k)-t0; end, end
function eta=mpptEff(so,sig,t0), [t,P]=getsig(so,sig.Ppv); [~,Pm]=getsig(so,sig.Pmpp); i=t>=t0; eta=100*trapz(t(i),P(i))/trapz(t(i),Pm(i)); end
function thd=thdOf(x,fs,f0,nh), X=abs(fft(x.*hann(numel(x))))/numel(x); f=(0:numel(x)-1)'*fs/numel(x);
  h=zeros(nh,1); for k=1:nh, [~,i]=min(abs(f-k*f0)); h(k)=max(X(max(i-2,1):i+2)); end, thd=sqrt(sum(h(2:end).^2))/h(1); end
