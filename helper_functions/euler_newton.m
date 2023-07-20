f = @(t,y) -20*t*y^2;
f_y = @(t,y) -40*t*y;
t0 = 0;
T = 1;
y0 = 1;
h = 0.01;
tol = 1e-8;
N = 100;
y = imp_euler(f,f_y,t0,T,y0,h,tol,N)
plot(t0:h:T,y)
hold on
plot(t0:h:T,1./(1+10*(t0:h:T).^2))
function y = imp_euler(f,f_y,t0,T,y0,h,tol,N)
t = t0:h:T;
n = length(t);
y = zeros(n,1);
y(1) = y0;
for k = 1:n-1
    g = @(z) z - y(k) - h*f(t(k+1),z);
    gp = @(z) 1 - h*f_y(t(k+1),z) ;
    disp('Vor Newton call.')
    y(k+1) = newton(g,gp,y(k),tol,N);
    disp('nach Newton Call.')
end
end
function sol=newton(f,fp,x0,tol,N)
i=0;
sol = zeros(N,2);
fc=abs(f(x0));
while fc > tol
    xc = x0 - (f(x0)/fp(x0));
    fc=abs(f(xc));
    x0 = xc;
    i=i+1;
    if (i>N)
        fprintf('Method failed after %d iterations. \n',N);
        break
    end
end
sol = x0;
end