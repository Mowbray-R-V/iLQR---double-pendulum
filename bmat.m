%function A = fun_amat(x,u,dt)

%%% For finding A matrix 
clear; clc; close all
global A B Nx Nu pert MI L m nx ny tx ty g r   



m1 = 2;  m2= 3;

L1= 2;
L2=3;


MI1 = 2.21304881;
MI2 = 1.29371245; MI3 = 0.182329; MI4 = 1.59264E-05;
MI5 = 1.293712458; MI6 = 0.182329; MI7 = 1.59264E-05;

MI = [MI1;MI2 ];
L = [L1;L2];
m = [m1;m2];

g = 9.81; % gravity
Nx = 4;
Nu  = 2;
Tf = 1;
dt = 0.01;
Nt = round(Tf/dt)+1;
A = zeros(Nx,Nx);
B = zeros(Nx,Nu);
pert = 0.001;
nx = 0;
tx = 1;
ny = 1;
ty = 0;
% initial conditions
tht1=deg2rad(20);tht2=deg2rad(35);omg1=0.5;omg2=0.5;
x0 = [tht1;tht2;omg1;omg2];


u = zeros(Nu,Nt-1);
x = zeros(Nx,Nt);
x_prev = zeros(Nx);
x = x0

for i = 1:Nx
        
        for j = 1:(Nx)
           % fun_xdot(x,u,dt)
            xnew = x + fun_xdot(x,u,dt)*dt/2; % is this constant ??
            xplus = x;
            xplus(j) =  x(j) + pert; %%% pert if iside diffferentiations
           

            if i<= Nx/2

          
                    %%% Jacobian of  Velocity with  q  and qdot
                    f = xnew((Nx/2)+i);
                    F = x(i) + f*dt;
                    qddot = fun_qddotDP(xplus,u,dt);
                    fplus = xplus((Nx/2)+i) + qddot(i)*dt/2;  
                    Fplus = xplus(i) + fplus*dt; 
                    A(i,j) = (Fplus - F)/pert;


           
            elseif i>Nx/2
    
                %%%  Jacobian of acceleration  with  q  and qdot

                        if j<=Nx/2
                                %%% pert in q for jac with acceleration
                                xplusr =    xnew;
                                xplusr(j) = xnew(j) + pert; 
                                
                        elseif j > Nx/2        
                
                            %%% pert in qdot for jac with acceleration
                            xplusr = xnew;
                            xplusr(j) =  xnew(j) + pert; 
                            xplusr(j-(Nx/2)) =  xnew(j-(Nx/2)) + pert*dt/2;
                        end


                    
                fr = fun_qddotDP(xnew,u,dt);
                Fr = x(i) + fr(i-(Nx/2))*dt;
                fplusr = fun_qddotDP(xplusr,u,dt); 
                Fplusr = xplus(i) + fplusr(i-(Nx/2))*dt; 
                A(i,j) = (Fplusr - Fr)/pert;

            end    
                
        end  

   

end