function load_gnss_quality_signals(datFile)
%LOAD_GNSS_QUALITY_SIGNALS  Push NSV/HPE/VPE/PDOP/SOL into base workspace.
%   Called by the Simulink model InitFcn before every simulation to ensure
%   that the From-Workspace blocks inside INSTRUMENT DECISION1 find their
%   data, even if the user batch script has just cleared the workspace.
%
%   Column map of the .dat file:
%     1=SOD 2=LON 3=LAT 4=ALT 5=CLK 6=GGTO 7=SOL 8=NSVVIS 9=NSV
%     10=HPE 11=VPE 12=EPE 13=NPE 14=UPE 15=HDOP 16=VDOP 17=PDOP

if nargin < 1 || isempty(datFile)
    datFile = 'cov_perturb_POS_s6a_Y24D011_fixed.dat';
end

datFile = inoas_data_file(datFile);

if ~isfile(datFile)
    error('load_gnss_quality_signals: file not found: %s', datFile);
end

fid = fopen(datFile, 'r');
raw = textscan(fid, repmat('%f',1,17), 'HeaderLines', 1);
fclose(fid);

t_dat = raw{1};

sol  = double(raw{7});
nsv  = double(raw{9});
hpe  = double(raw{10});
vpe  = double(raw{11});
pdop = double(raw{17});

% The first rows of the file can contain no-fix placeholders at t=0 before
% the first valid PPP solution. If these zeros are fed directly to the
% decision FSM, the model immediately declares a GNSS emergency and can stay
% in Kalman mode for the whole run. For simulation starts that assume an
% already available navigation solution, hold the first valid quality sample
% backward over the initial placeholder interval.
valid_idx = find(sol >= 0.5 & nsv >= 4 & pdop > 0, 1, 'first');
if ~isempty(valid_idx) && valid_idx > 1
    sol(1:valid_idx-1)  = sol(valid_idx);
    nsv(1:valid_idx-1)  = nsv(valid_idx);
    hpe(1:valid_idx-1)  = hpe(valid_idx);
    vpe(1:valid_idx-1)  = vpe(valid_idx);
    pdop(1:valid_idx-1) = pdop(valid_idx);
end

ts_gnss_sol  = timeseries(sol,  t_dat, 'Name', 'gnss_sol');
ts_gnss_nsv  = timeseries(nsv,  t_dat, 'Name', 'gnss_nsv');
ts_gnss_hpe  = timeseries(hpe,  t_dat, 'Name', 'gnss_hpe');
ts_gnss_vpe  = timeseries(vpe,  t_dat, 'Name', 'gnss_vpe');
ts_gnss_pdop = timeseries(pdop, t_dat, 'Name', 'gnss_pdop');

assignin('base', 'ts_gnss_sol',  ts_gnss_sol);
assignin('base', 'ts_gnss_nsv',  ts_gnss_nsv);
assignin('base', 'ts_gnss_hpe',  ts_gnss_hpe);
assignin('base', 'ts_gnss_vpe',  ts_gnss_vpe);
assignin('base', 'ts_gnss_pdop', ts_gnss_pdop);

if evalin('base', 'exist(''Ts'',''var'')') == 0
    assignin('base', 'Ts', 1);
end

end
