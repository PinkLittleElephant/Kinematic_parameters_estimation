clear;
%% Setup manipulator model via Peter Corke package
N = 3; % joints number
name = "2JM"; % robot name

% SETUP DH param
d = zeros(N,1);
a = zeros(N,1);
al = zeros(N,1);
oth = zeros(N,1); % offset

if N == 2
    a(1) = 1;
    d(1) = 0.1;
    a(2) = 1;
    oth(1) = pi/2;
end

if N == 3
    a(1) = 1;
    a(2) = 1;
    a(3) = 1;
    oth(1) = pi/2;
end
q = zeros(1,N);

for n = 1:N
    L(n) = Revolute('d',d(n),'a',a(n),'alpha',al(n),'offset',oth(n));
    L(n).qlim = [-pi,pi]; % setup joint limits
end

Robot = SerialLink(L,'name',name);

clear L d a al oth name n;

close all;
Robot.base = trotx(90);
Robot.plot(q);
Robot.teach();

%%  Main part of the script



t = (1:0.01:100)';
q = [pi*sin(0.1*t),pi*sin(7*t),pi*cos(100*t)];
C = [];

for n = 1:max(size(q))
    C(n) = cond(Robot.jacob0(q(n,:)));
end

plot(C');

disp('Done.');
