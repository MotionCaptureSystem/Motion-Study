clear all

% N is the number of steps the simulation proceeds
% h is the discrete time step in sec
N=1000;
h=.01;
m=.001;

q=zeros(3,N);
a=2;
phi=atan(a);


Lambda=zeros(1,N);

theta0=pi/3;

syms Q1 Q3 L
x(1)=cos(theta0)*cos(phi);
x(2)=cos(theta0)*cos(phi);
z(1)=sin(theta0);
z(2)=sin(theta0);

Vq1=zeros(2,N);
Vq3=zeros(2,N);


for n=3:N
    
    eqn1=   m/h^2*(Q1-2*x(n-1)+x(n-2))           +2*L*x(n-1)*(1+a^2)   ==0;
    eqn2=   m/h^2*(Q3-2*z(n-1)+z(n-2))  +m*9.81  +2*L*z(n-1)           ==0;
    eqn3=   (1+a^2)*Q1^2                +Q3^2    -1                    ==0;
    
    V=solve([eqn1,eqn2,eqn3]);
    Vq1(:,n)=double(V.Q1);
    Vq3(:,n)=double(V.Q3);
    Vl=double(V.L);
    
    if norm([Vq1(1,n); Vq3(1,n)]-[x(n-1); z(n-1)])<norm([Vq1(2,n); Vq3(2,n)]-[x(n-1); z(n-1)])
               x(n)=Vq1(1,n);
               z(n)=Vq3(1,n);
    else
               x(n)=Vq1(2,n);
               z(n)=Vq3(2,n);
    end
    
    Lambda(n)=Vl(1);
     
    
end
y=a*x;
figure(); plot3(x,y,z)

