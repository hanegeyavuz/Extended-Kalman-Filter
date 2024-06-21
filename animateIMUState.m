function animateIMUState(IMU_DATA, PhiSaved, ThetaSaved, PsiSaved, Nsamples)
    Cuboid = [-2, -3, -0.5; 2, -3, -0.5; 2, 3, -0.5; -2, 3, -0.5; -2, -3, 0.5; 2, -3, 0.5; 2, 3, 0.5; -2, 3, 0.5];
    fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    figure;
    view(3);
    Object_IMU = patch('Faces', fac, 'Vertices', [0, 0, 0]);
    Object_IMU.FaceColor = 'g';
    axis([-5 5 -5 5 -5 5]);
    title('IMU State');
    Realtime = xlabel(' ');

    for k = 1:Nsamples-1
        Rz = [cosd(PsiSaved(k)) -sind(PsiSaved(k)) 0; sind(PsiSaved(k)) cosd(PsiSaved(k)) 0; 0 0 1];
        Ry = [cosd(ThetaSaved(k)) 0 sind(ThetaSaved(k)); 0 1 0; -sind(ThetaSaved(k)) 0 cosd(ThetaSaved(k))];
        Rx = [1 0 0; 0 cosd(PhiSaved(k)) -sind(PhiSaved(k)); 0 sind(PhiSaved(k)) cosd(PhiSaved(k))];
        
        for j = 1:8
            Result_1(j, :, k) = (Rx * Ry * Rz * Cuboid(j, :)')';
        end
        
        set(Object_IMU, 'Vertices', Result_1(:, :, k));
        set(Realtime, 'String', sprintf('time = %.2f [s]', IMU_DATA(10, k)));
        drawnow;
    end
end
