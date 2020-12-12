clear;
%% Setup manipulator model via Peter Corke package
N = 2; % joints number
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
    a(2) = 0.5;
    a(3) = 0.5;
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
q_trj = [];
C = [];

check = 3.5;
max_step = pi/180;
search = true;

% Let's find start position. where condition number is 0.9 * check 
qsp = zeros(1,N); % q start position
step_count = 0;
qsp_t = qsp;

while search
    for n = 1:N
        qsp_t(n) = qsp(n) + max_step;
        cn_t1 = cond(Robot.jacob0(qsp_t));
        qsp_t(n) = qsp(n) - max_step;
        cn_t2 = cond(Robot.jacob0(qsp_t));
        if cn_t1 < cn_t2
            qsp_t(n) = qsp(n) + max_step;
        end
        if cn_t1 < (0.9 * check) | cn_t2 < (0.9 * check)
            search = false;
            break;
        end
    end
    qsp = qsp_t;
    step_count = step_count + 1;
    if step_count > 1e3
        break;
    end
end
disp(['\n The start point is : ',char(num2str(qsp))])



disp('Done.');
