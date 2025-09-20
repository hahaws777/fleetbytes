%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSC C85 - Fundamentals of Robotics and Automated Systems
% UTSC - Fall 2021
%
% Starter code (c) F. Estrada, August 2021
% 
% Sensors and Signal Processing
%
%  You may have heard there are all kinds of plans to
% send humans to Mars. Eventually, some think we may
% establish long-term habitats on the martian surface
% occupied for long periods by 'martians'.
%
%  One of the challenges of maintaining a long term
% presence on Mars is that martian gravity is much
% weaker than Earth's, and though it's still much
% better than long-term living in space, martian
% explorers would need to keep a serious exercise
% program in order to prevent physical deterioration.
%
%  To help with this task, we've developed the
% FleetByte(tm). A device worn on a person's wrist
% that keeps track of their exercise. It's similar to
% devices you may be familiar with (or indeed which
% you may be wearing). Our goal here is to design
% the sensor and signal processing software that
% will convert the raw measurements provided by
% sensors in the device into accurate estimates
% of what the human wearing it is doing.
%
% Your task is to:
%
% a) Understand the different sensors available,
%    the values they report, and their noise profile.
% b) Implement suitable noise reduction and estimation
%    routines so as to obtain estimates of state variables
%    that are as close as possible to their actual values.
% c) Apply the ideas we discussed in lecture: Noise
%    filtering, consistency, and information redundancy
%    in order to obtain good estimates for state variables.
%
% [xyzRMS,velRMS,angRMS,hrRMS]=FleetByte(secs, map, deb)
%
%   secs - number of (virtual) seconds to run the simulation for.
%          Each call to Sim1() returns sensor readings for 1 sec,
%          so this is in effect the numbe or rounds of simulation
%          you want. 
%
%   map - Select map (1 or 2), each is a crop from the global Mars
%         elevation map from NASA - image in public domain. Note
%         that motion on the map is *not to scale*, the map corrsponds
%         to a huge area on Mars, and I want to show motion on this
%         map. So we will pretend it corresponds to an area roughly
%         .5 x .5 Km in size.
%
%  deb - If set to 1, this script will plot the returned hearrate
%          sensor output (so you can see what it looks like and think
%          about how to get a heartrate out of it), and print out
%          the sensor readings returned by Sim1(). You can add your
%          own debug/testing output as well.
%               
% - delta_t - maximum change in rover direction per unit of time, in radians
%
% Return values:
%
% xyzRMS - The RMS error for position estimates obtained over the spevified number of frames in meters
% velRMS - The RMS error for velocity estimates (Km/h)
% angRMS - The RMS error for running direction estimates (in radians)
% hrRMS - The RMS error in heartrate estimates, in BPM.
%
%  On Board Sensors:
%
%  MPS - Martian Positioning System - reports 3D position anywhere on Mars
%        to within a small displacement from actual location. Like its
%        Earthly cousin, MPS has an expected location error. For 
%        a typical wearable device, on Earth, location error is
%        within 5m of the actual location 
%        (https://www.gps.gov/systems/gps/performance/accuracy/)
%        Our FleetByte has a similar receiver, but due to the lower
%        density of Martian atmosphere, distortion due to armospheric
%        effects is lower. Under open sky this means a typical location
%        accuracy of less than 1.5m.
%
%        Note: On Mars we don't have to worry about buildings. On Earth things
%          are more difficult since buildings reflect GPS signals leading
%          to increased error in position estimates.
%
%  Heart Rate Sensor (HRS) - This one is interesting. Modern wearable 
%        HR monitors typically use light reflection from 
%        arterial blood to determine the heart rate - the
%        pulsing blood creates a periodic waveform in the 
%        reflected light. Issues with noise, low signal-to-noise
%        ratio, and effects due to skin colour, thickness, and
%        even ambient light combine to produce a fairly noisy
%        signal. The HR sensor will return an array consisting
%        of the signal measured over the last 10 seconds, from
%        which you will estimate the actual heartrate.
%        If you're very curious, this manufacturer has a
%        very thorough description of how their sensor works and
%        the different technical issues involved in computing a
%        heartrate from it ** YOU ARE NOT EXPECTED TO READ 
%        THROUGH AND IMPLEMENT THIS, IT'S THERE IN CASE YOU 
%        WANT TO LEARN MORE **
%        https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
%
%  Rate gyro (RG) - A fairly standard rate gyro, returns the measured
%              change in angle for the direction of motion (i.e.
%              tells you by how many radians this direction changed
%              in between readings). 
%
%              Somewhat noisy, but this assuming the user doesn't
%              move their arms in weird directions while running
%              it won't be affected by periodic arm motions.
%
%  ** The simulation returns to you the values measured by each
%  ** of these sensors at 1 second intervals. It's up to you to
%  ** decide how best to use/combine/denoise/filter/manipulate
%  ** the sensor readings to produce a good estimate of the actual
%  ** values of the relevant variables (which are also returned
%  ** by the simulation for the purpose of evaluating your
%  ** estimates' accuracy - needless to say, you can't use these
%  ** in any way, shape, or form, to accompish your task.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xyzRMS,velRMS,angRMS,hrRMS]=FleetByte(secs, map, deb)

%pkg load image;             %%% Comment this out for MATLAB

close all;
%%%%%%%%%% YOU CAN ADD ANY VARIABLES YOU MAY NEED BETWEEN THIS LINE... %%%%%%%%%%%%%%%%%


prev_xyz = [256 256 .5]

vel_prev = 0

Rg_lp =0

theta_prev = 0
pos_for_dir =[0,0,0]



%%%%%%%%%% ... AND THIS LINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

idx=1;
while(idx<=secs)               %% Main simulation loop

 [MPS,HRS,Rg]=Sim1(map);       % Simulates 1-second of running and returns the sensor readings



                         
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % TO DO:
 %  Sim1() returns noisy readings for global position (x,y,z), a heart-rate
 %    sensor signal array for the last 10 seconds, and a value for the rate
 %    gyro (the amount of rotation in radians by which the running direction
 %    changed over the last second).
 %
 %    In the space below, write code to:
 %
 %    - Estimate as closely as possible the actual 3D position of the jogger
 %
 %    - Compute the current hear-rate (this will require some thought, make
 %      sure to look closely at the plot of HRS, and think of ways in which
 %      you can determine the heart rate from this). Remember the data 
 %      in the plot corresponds to the last 10 seconds. And, just FYI, it's
 %      based on what the actual data returned from a typical wrist-worn
 %      heart rate monitor returns. So it's fairly realistic in terms of what
 %      you'd need to process if you were actually implementing a FleetByte
 %
 %    - Estimate the running direction (huh? but the rate gyro only returns
 %      the change in angle over the last second! we don't know the initial
 %      running direction right?) - well, you don't, but you can figure it
 %      out :) - that's part of the exercise.
 %      * REFERENCE: - given a direction vector, if you want to apply a
 %         rotation by a particular angle to this vector, you simply 
 %         multiply the vector by the corresponding rotation matrix:
 %
 %            d1=R*d;
 %
 %         Where d is the input direction vector (a unit-length, column
 %         vector with 2 components). R is the rotation matrix for 
 %         the amount of rotation you want:
 %
 %           R=[cos(theta) -sin(theta)
 %              sin(theta) cos(theta)];
 %
 %         'theta' is in radians. Finally, d1 is the resulting direction vector.
 %
 %    - Estimate the running speed in Km/h - This is *not* returned by any
 %      of the sensor readings, so you have to estimate it (carefully). 
 %
 %    Goal: To get the estimates as close as possible to the real value for
 %          the relevant quantities above. The last part of the script calls
 %          the imulation code to plot the real values against your estimares
 %          so you can see how well you're doing. In particular, you want the
 %          RMS of each measurement to be as close to 0 as possible.
 %          RMS is a common measure of error, and corresponds to the square
 %          root of the average squared error between a measurement and the
 %          corresponding estimate, taken over time. 
 %    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 xyz=[128 128 .5];       % Replace with your computation of position, the map is 512x512 pixels in size
 hr=82;                  % Replace with your computation of heart rate
 di=[0 1];               % Replace with your computation for running direction, this should be a 2D unit vector
 vel=5;                  % Replace with your computation of running velocity, in Km/h
 
 if (deb==1)
     figure(5);clf;plot(HRS);
     fprintf(2,'****** For this frame: *******\n');
     fprintf(2,'MPS=[%f %f %f]\n',MPS(1),MPS(2),MPS(3));
     fprintf(2,'Rate gyro=%f\n',Rg);
     fprintf(2,'---> Press [ENTER] on the Matlab/Octave terminal to continue...\n');
     drawnow;
     pause;
 end;
 
 %%% SOLUTION:







 %% dealing with the heart rate calculation

fs = 120

 % -------- 预处理 --------
ecg = HRS(:);                 % 保证是列向量（1200x1）
ecg = movmedian(ecg, 5);      % 用窗口=5的中值滤波去掉尖峰噪声
ecg = normalize(ecg);         % 归一化信号到 [-1,1]，方便设阈值

% -------- 检测波峰 --------
% 阈值设为中位数 + 0.5*标准差（自适应阈值，避免固定阈值带来的问题）
th = median(ecg) + 0.5*std(ecg);

% 设置相邻两个峰的最小间隔，0.25 秒 ≈ 240 bpm 的上限
% （避免一个 QRS 波被分成多个峰）
minDist = round(0.25 * fs);

% 找峰：大于阈值，且相邻峰间隔 > minDist
[pks, locs] = findpeaks(ecg, ...
        'MinPeakHeight', th, ...      % 峰值最低高度
        'MinPeakDistance', minDist);  % 峰之间的最小采样点距离

% -------- 计算心率 --------
if numel(locs) >= 2               % 至少要有两个峰才能计算 RR 间期
    RR = diff(locs) / fs;         % RR 间期（秒），即相邻峰点时间差
    % 筛选合理的 RR 范围：0.25s ~ 2s → 心率范围 30 bpm ~ 240 bpm
    RR = RR(RR > 0.25 & RR < 2);
    disp(RR)
    % for rr, we will fix it by adding another place of bump if the
    % distance of two rr is too big

    medRR = median(RR);
    fixedRR = [];
    for i = 1:numel(RR)
        if RR(i) > 1.5*medRR & RR(i) <2.5*medRR
            fixedRR = [fixedRR, RR(i)/2, RR(i)/2];  % 拆成两个
        elseif RR(i) >2.5*medRR
            fixedRR = [fixedRR, RR(i)/3,RR(i)/3, RR(i)/3]; % 拆成三个
        else
            fixedRR = [fixedRR, RR(i)];
        end
    end
    RR = fixedRR;

    
    if ~isempty(RR) 
        % let's calculate the weighted average of the RR

        N = numel(RR);
        alpha = 2;         % 调节参数，1=线性, 2=平方，越大越陡
        W = (1:N).^alpha;


        RR_weighted = sum(RR .* W)/sum(W)
        
        % 如果筛选后还有有效 RR
        hr = 60 / RR_weighted;     % 心率 = 60 / 中位数(RR)，更稳健
    else
        hr = NaN;                 % 如果没有有效 RR，返回 NaN
    end
else
    hr = NaN;                     % 峰太少（<2）时算不出来
end

%%done with the heart rate calculation



%%% starting with the xyz calculation

alpha_now = 0.8;        % 当前测量的权重（你要的 0.7）
alpha_prev = 1 - alpha_now;   % 上一帧估计的权重（=0.3）

xyz = MPS

prev_xyz2 = prev_xyz % used for calculating the velocity



xyz = alpha_now * MPS + alpha_prev * prev_xyz;  % Update current position estimate
prev_xyz = xyz;



xyz(1) = max(1, min(512, xyz(1)));   % 限制 x 在 [1,512]
xyz(2) = max(1, min(512, xyz(2)));   % 限制 y 在 [1,512]
xyz(3) = max(0, xyz(3));             % 限制 z >= 0


% done with the xyz calculating


%%% doing the calculation on the direction


% % % doing calculations on the velocity

alpha_vel = 0.8
alpha_vel_prev = 1-alpha_vel

if idx ==1
    vel = norm(xyz(1:2) - prev_xyz2(1:2))
else
    vel = alpha_vel * vel + alpha_vel_prev * norm(xyz(1:2) - prev_xyz2(1:2));  % Update velocity estimate
end
vel = vel *3.6
vel_prev = vel


%%% end of calculating velocity

%%% CALCULATION FOR the angle

alpha_rg = 0.2;
alpha_rg_prev = 1 - alpha_rg;

if isempty(Rg_lp), Rg_lp = Rg; end
Rg_lp = alpha_rg*Rg + (1-alpha_rg)*Rg_lp;


% ===== Direction estimation (complementary fusion) =====
gamma_yaw = 0.35;    % 越大越信位移方向（0.2~0.5）
min_step  = 0.25;    % 只有位移>0.25m才纠偏，防抖动误导

if idx ==0
    theta_prev = 0;
    pos_for_dir = xyz;
end



% 1) 陀螺积分（短期基准）
theta_gyro = theta_prev + Rg_lp;
theta_gyro = atan2(sin(theta_gyro), cos(theta_gyro));  % wrap 到 [-pi,pi]

% 2) 位移角（长期参考，只在移动够大时使用）
disp_vec = xyz(1:2) - pos_for_dir(1:2);
if norm(disp_vec) > min_step
    theta_disp = atan2(disp_vec(2), disp_vec(1));
    % 在圆上做加权平均（避免角度跳变问题）
    theta = atan2( ...
              (1-gamma_yaw)*sin(theta_gyro) + gamma_yaw*sin(theta_disp), ...
              (1-gamma_yaw)*cos(theta_gyro) + gamma_yaw*cos(theta_disp) );
    pos_for_dir = xyz;                 % 只有纠偏时才更新锚点
else
    theta = theta_gyro;
end

% 3) 输出单位方向向量
di = [cos(theta)  sin(theta)];
theta_prev = theta;






 

    disp(idx)


  
 %%%%%%%%%%%%%%%%%%  DO NOT CHANGE ANY CODE BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
 % Let's use the simulation script to plot your estimates against the real values
 % of the quantities of interest and obtain error measures - notice we ignore the
 % returned XYZ, HRSt, and Rg values since they're the same we got above.
 [t1,t2,t3,xyzRMS,velRMS,angRMS,hrRMS]=Sim1(map, xyz,hr,di,vel);

 
 idx=idx+1; 
end;

%%%%% Interesting links you may want to browse - I used these while designing this exercise.
% https://www.rohm.com/electronics-basics/sensor/pulse-sensor
% https://valencell.com/blog/optical-heart-rate-monitoring-what-you-need-to-know/
% https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
