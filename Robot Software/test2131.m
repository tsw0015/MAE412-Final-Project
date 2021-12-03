t = 0:0.01:4;
k = 0;
for i = 1:21
    a(i)=6.2-i*0.2;
    for j = 1:21
        b(j) =6.2-j*0.2;
        for h = 1:31;
            c(h) =12.2-h*0.2;
            num = [0 2*a(i)*c(h) (a(i)^2+b(j)^2+2*a(i)*c(h)) (a(i)^2+b(j)^2)*c(h)];
            den = [1 2*a(i)*c(h) a(i)^2+b(j)^2+2*a(i)*c(h) (a(i)^2+b(j)^2)*c(h)];
            y = step(num,den,t)
            m = max(y)
            s = 401; while y(s)>0.98 & y(s)<1.02;
            s = s-1; end;
        ts = (s-1)*0.01
        if m<1.2 & ts<1.0
           k=k+1
           table(k,:) = [a(i) b(j) c(h) m ts];
        end
        end
    end
end
