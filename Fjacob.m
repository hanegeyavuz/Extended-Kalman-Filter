function F = Fjacob(p, q, r, dt)
%
% H Function Update for EKF
%
% p, q ,r ; Orientation from Gyro
%
% dt derivative from sample time

F = zeros(4);

F(1,1)=1;
F(1,2)= -p*dt/2;
F(1,3)= -q*dt/2;
F(1,4)= -r*dt/2;

F(2,1)= p*dt/2;
F(2,2)=1;
F(2,3)= r*dt/2;
F(2,4)=-q*dt/2;

F(3,1)=q*dt/2;
F(3,2)=-r*dt/2;
F(3,3)=1;
F(3,4)=p*dt/2;

F(4,1)=r*dt/2;
F(4,2)=q*dt/2;
F(4,3)=-p*dt/2;
F(4,4)=1;

end