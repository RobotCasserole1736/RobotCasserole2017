fpath = ".\\log_2016-07-30_122429.csv";
data = csvread(fpath);
time = data(3:end,1);
shooterRPM = data(3:end,53);

dt = diff(time);

RPMps = diff(shooterRPM)./dt;


RPMpsps = diff(RPMps)./dt(1:end-1);


RPMpspsFilt = zeros(length(RPMpsps),1);

RPMpspsFilt(1) = RPMpsps(1);
RPMpspsFilt(2) = RPMpsps(2);
for idx = 3:length(RPMpsps)
  RPMpspsFilt(idx) = (RPMpsps(idx) + RPMpsps(idx-1) + RPMpsps(idx-2))/3;
end



plot(time(1:end-2), shooterRPM(1:end-2), time(1:end-2), RPMpspsFilt);