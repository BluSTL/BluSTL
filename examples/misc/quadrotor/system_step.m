function x1 = system_step(sysd, x0, u0, w0)
        x1 = [sysd.A*x0 + sysd.B*[u0; w0]];
end