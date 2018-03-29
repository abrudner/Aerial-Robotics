function [coeff, A, b] = getCoeff(waypoints)

% waypoints are assumed to be a Nx1 vector of one of the coordinates

n = size(waypoints,1) - 1; % number of segments P1...Pn
A = zeros(8*n,8*n);
b = zeros(1,8*n);

% b vector
for i = 1:n
    b(1,i) = waypoints(i,1);
    b(1,i+n) = waypoints(i+1,1);
end

row = 1;

%% Constraint 1: Pi(0) = Wi

for i=1:n
    A(row,(8*(i-1)+1):(i*8)) = polyT(8,0,0);
    row = row+1;
end

%% Constraint 2: Pi(1) = Wi+1

for i=1:n
    A(row,((8*(i-1))+1):(i*8)) = polyT(8,0,1);
    row = row+1;
end

%% Constraint 3: P1(k=1)(t=0)=0

for k=1:3
    A(row,(1:8)) = polyT(8,k,0);
    row = row+1;
end

%% Constraint 4: Pn(k=1)(t=1)=0

for k=1:3
    A(row,(8*(n-1)+1:8*n)) = polyT(8,k,1);
    row = row+1;
end

%% Constraint 5: Pi-1(k=1)(t=1) = Pi(k=1)(t=0)
%  Same as Pi-1(k=1)(t=1) - Pi(k=1)(t=0) = 0

for i=2:n
    for k=1:6
        A(row,((i-2)*8)+1:i*8) = [polyT(8,k,1) -polyT(8,k,0)];
        row = row+1;
    end
end

%% Bring it all together

% DEBUG
% b=b';
assignin('base','A',A);
assignin('base','b',b);

coeff = inv(A)*b';
% DEBUG
assignin('base','coeff',coeff);

end

%% Old code
% %% Constraint 1: Pi(0) = Wi
% 
% A(row,1:8) = polyT(8,0,0);
% row = row+1;
% for i=2:n
%     A(row,(8*(i-1)+1:8*i)) = polyT(8,0,0);
%     row = row+1;
% end
% 
% %% Constraint 2: Pi(1) = Wi+1
% 
% A(row,1:8) = ones(1,8);
% row = row+1;
% for i=2:n
%     A(row,(8*(i-1)+1:8*i)) = ones(1,8);
%     row = row+1;
% end
% 
% %% Constraint 3: P1(k=1)(t=0)=0
% 
% A(row,1:8) = [0 1 0 0 0 0 0 0];
% row = row+1;
% for i=2:n
%     A(row,(8*(i-1)+1:8*i)) = [0 1 0 0 0 0 0 0];
%     row = row+1;
% end

