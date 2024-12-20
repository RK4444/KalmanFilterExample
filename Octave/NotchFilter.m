pkg load control;

fs = 77;
Ts = 1/fs;

f = 2;

wn = 2*pi*f*Ts;

a = 0.8;

n = [1 -2*cos(wn) 1];
d = [1 -2*a*cos(wn) a^2];

freqz(n,d);
figure;
notchfilt = tf(n, d, Ts);
bode(notchfilt);
