function [lowBound, upBound] = get_temp_bnd(Tave, DTocc, DTunocc, Delta, N)
%    
%  generates comfort temperature zones     
% 
    
    lowBound=zeros(N,1);
    for k=1:1:(N/4)
        lowBound(k)= Tave - DTunocc;
        lowBound(0.25*N+k)= Tave - DTocc;
        lowBound(0.5*N+k)= Tave - DTocc;
        lowBound(0.75*N+k)= Tave - DTunocc;
    end

    for k= -Delta:1:Delta
        lowBound(N/4 + k)= ( (Tave-DTocc + Tave-DTunocc)/2 + (1/Delta) * (((Tave-DTocc) - (Tave-DTunocc))/2) * k  );
        lowBound(3*N/4 + k) = ( (Tave-DTocc + Tave-DTunocc)/2 - (1/Delta) * (((Tave-DTocc) - (Tave-DTunocc))/2) * k   );
    end

    upBound=zeros(N,1);
    for k=1:1:(N/4)
        upBound(k)= Tave + DTunocc;
        upBound(0.25*N+k)= Tave + DTocc;
        upBound(0.5*N+k)= Tave + DTocc;
        upBound(0.75*N+k)= Tave + DTunocc;
    end

   
    for k= -Delta:1:Delta
        upBound(N/4 + k)   = (  (Tave + DTocc + Tave + DTunocc)/2 + (1/Delta) * (((Tave + DTocc)-(Tave + DTunocc))/2) * k  );
        upBound(3*N/4 + k) = (   (Tave + DTocc + Tave + DTunocc)/2 - (1/Delta) * (((Tave + DTocc) - (Tave + DTunocc))/2) * k  );
    end
