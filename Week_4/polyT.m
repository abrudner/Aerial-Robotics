function [T] = polyT(n,k,t)
% n  = polynomial number of coefficients
% k  = requested derivative
% t  = actual value of t (can be anything)

T = zeros(n,1);
D = zeros(n,1);

% init

for i = 1:n
    D(i) = i-1;
    T(i) = 1;
end
% assignin('base','T1',T);
% derivative

for j = 1:k
    for i = 1:n
        T(i) = T(i)*D(i);
        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
% assignin('base','T2',T);
% put t value

for i = 1:n
    T(i) = T(i)*t^D(i);
end
% assignin('base','T3',T);

T = T';

end
