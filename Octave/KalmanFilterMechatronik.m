fsensor = 75;
dT = 1/fsensor;
magneticSensorVariance = 5e-4;  % The variance according to datasheet is actually 2.5e-5, but since it is known that the system will catch noise, a much higher value is needed
imuSensorVariance = 2e-3;

N=150;

t = linspace(0, N*dT, N);
testsig = pi - 2*pi*cos(pi*t/2);
testsig2 = testsig + 0.5*randn(size(t)); % setting up the testsignal. Noise is added to the blank testsignal

A=[1 dT;
    0 1]; % The matrices according to the formulas

H=[1 0];

Q=[magneticSensorVariance 0;
    0 magneticSensorVariance/dT]; %this is actually magneticSensorVariance*fsensor, but because this seems harder to get notationally, the division by dT is used.

R=imuSensorVariance;

xkCorrected = [0; 0];
PkCorrection = zeros(2);
Kprev = zeros(2,1);
K = zeros(2,1);
xfinal = zeros(1,N);
xfinalspeed = zeros(1,N);
KdiffRow = zeros(2,N);

for i=1:1:N % recursively estimate the signals according to the formulas

  xkEstimate = A*xkCorrected;
  PkEstimate = A*PkCorrection*A' + Q;

  Kprev=K;

  K = PkCorrection*H'*inv(H*PkCorrection*H'+R);

  Kdiff = K - Kprev;
  KdiffRow(:,i) = Kdiff;
  xkCorrected = xkEstimate + K*(testsig(i)-H*xkEstimate);
  xfinal(i) = xkCorrected(1);
  xfinalspeed(i) = xkCorrected(2);
  PkCorrection = (eye(2)-K*H)*PkEstimate;

end

% Plotting

figure('Name', 'Signal Overview');
subplot(1,3,1);plot(t, testsig); xlabel("t");ylabel("Testsignal");title("Testsignal without noise");
subplot(1,3,2);plot(t, testsig2); xlabel("t");ylabel("Testsignal");title("Testsignal with noise added");
subplot(1,3,3);plot(t, xfinal); xlabel("t");ylabel("filtered Signal");title("Filtered signal");

figure('Name', 'Absolute change of the Kalman Gain');
plot(t, KdiffRow); xlabel("t");ylabel("Absolute change");title("Difference of K_k and K_{k-1}");

figure('Name', 'Estimated derivative');
plot(t, xfinalspeed); xlabel("t");ylabel("Derivative");title("The estimated derivative of the testsignal");
