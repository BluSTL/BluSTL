function [ Wn ] = STLC_sensing( Sys )

nw = Sys.nw;
time = Sys.time;
time_d = Sys.model_data.time;
Wref = Sys.Wref;

for iwx=1:nw
    Wn(iwx,:) = interp1( time , Wref(iwx,:)', time_d)';
end
end

