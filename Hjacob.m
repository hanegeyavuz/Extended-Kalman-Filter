function H = Hjacob(qt0, qt1, qt2, qt3, B)
%
% H function Update for EKF
%
%
%
g=9.8;
H = zeros(6,4);

H(1,1) = -qt2;
H(1,2) = qt3;
H(1,3) = -qt0;
H(1,4) = qt1;

H(2,1) = qt1;
H(2,2) = qt0;
H(2,3) = qt3;
H(2,4) = qt2;

H(3,1) = qt0;
H(3,2) = -qt1;
H(3,3) = -qt2;
H(3,4) = qt3;

H(4,1) = qt0*B(1)+qt3*B(2)-qt2*B(3);
H(4,2) = qt1*B(1)+qt2*B(2)+qt3*B(3);
H(4,3) = -qt2*B(1)+qt1*B(2)-qt0*B(3);
H(4,4) = -qt3*B(1)+qt0*B(2)+qt1*B(3);

H(5,1) = -qt3*B(1)+qt0*B(2)+qt1*B(3);
H(5,2) = qt2*B(1)-qt1*B(2)+qt0*B(3);
H(5,3) = qt1*B(1)+qt2*B(2)+qt3*B(3);
H(5,4) = -qt0*B(1)-qt3*B(2)+qt2*B(3);

H(6,1) = qt2*B(1)-qt1*B(2)+qt0*B(3);
H(6,2) = qt3*B(1)-qt0*B(2)-qt1*B(3);
H(6,3) = qt0*B(1)+qt3*B(2)-qt2*B(3);
H(6,4) = qt1*B(1)+qt2*B(2)+qt3*B(3);

end