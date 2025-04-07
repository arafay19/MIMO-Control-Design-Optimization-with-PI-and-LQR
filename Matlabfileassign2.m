%% Design of MIMO PI Controller

A= [-3 0 0;
    0 -5 10;
    0 -10 -5];
B= [1 0 0;
    3 0 1;
    2 1 0];
C= [1 0 0;
    0 1 0;
    0 0 1];

D = 0;
x0 =[1;
    1;
    1];

% Is system safe to experiment
e_values = eig (A);
% all Eigen values have negative real parts
Co = ctrb (A,B);
rank (Co);
% and Controllbility matrix has full rank 
% system is safe to experiment with

% Does the I type Controller exists for this plant
% Condition to check rank (K_s)=dim (y)
% Since the dimension of y is 3

K_s = -C * inv(A) * B;
rank (K_s);

% Since rank (K_s) = dim (y), I type controller exists for this plant
% Now we calculate Integral gain first
% since K_s is a square matrix

K_i_tilde = inv(K_s);

% Now we include the proportional gain as well
% since K_s is a square matrix

K_p_tilde = inv(K_s);


%% LQR Controller Design

q1 = 25;
q2 = 20;
q3 = 10;
r1 = 0.5;
r2 = 0.2;
r3= 0.6;
Qy= [q1 0 0;
    0 q2 0;
    0 0 q3];
R= [r1 0 0;
    0 r2 0;
    0 0 r3];
Q = C'*Qy*C;
F = lqr(A,B,Q,R);

WW = -inv(C*inv(A-B*F)*B);



%% Limitng all the inputs under 10 units and plotting of inputs and outputs
Sys_sf= ss(A-B*F,B,C,D);

t = 0:0.5:5;

[y_ini,~,x_ini] = initial(Sys_sf,x0,t);

u_ini = -F*x_ini';

Ex_6_2_a_plot_fun(t,y_ini,u_ini);

%% System mismatch ad its impact
Anew = A*0.8;


%% Plot function

function [ ] = Ex_6_2_a_plot_fun(t,y,u)
figure(1)
subplot(3,2,1)
plot(t,y(:,1))
grid on
ylabel('$y_1$','interpreter','latex')
xlabel('');
axis ([0 5 -0.5 1.5]);


subplot(3,2,3)
plot(t,y(:,2))
grid on
ylabel('$y_2$','interpreter','latex')
xlabel('');
axis ([0 5 -0.5 1.5]);

subplot(3,2,5)
plot(t,y(:,3))
grid on
ylabel('$y_2$','interpreter','latex')
xlabel('$t$','interpreter','latex')
axis ([0 5 -0.5 1.5]);

subplot(3,2,2)
plot(t,u(1,:))
grid on
ylabel('$u_1$','interpreter','latex')
xlabel('');
axis ([0 5 -10 1]);

subplot(3,2,4)
plot(t,u(2,:))
grid on
ylabel('$u_2$','interpreter','latex')
xlabel('');
axis ([0 5 -0.5 1.5]);

subplot(3,2,6)
plot(t,u(3,:))
grid on
ylabel('$u_2$','interpreter','latex')
xlabel('$t$','interpreter','latex')
axis ([0 5 -0.5 1.5]);


end



