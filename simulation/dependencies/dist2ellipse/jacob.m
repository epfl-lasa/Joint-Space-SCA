clear all
clc

syms u real;

p = sym('p',[3,1],'real');
q = sym('q',[3,1],'real');
dim = sym('dim',[3,1],'real');

f = sum((q./dim).^(2*u)) - 1; 
n = [diff(f,'q1'); diff(f,'q2'); diff(f,'q3')];
F = [f; cross(p-q, n)];

J = jacobian(F,[q(1), q(2), q(3)]);

for i=1:length(F)
    fprintf(['F[' num2str(i-1) '] = %s; \n'], char(F(i)));
end

for i=1:length(F)
    for j=1:3
        fprintf(['J(' num2str(i-1) ', ' num2str(j-1) ') = %s; \n'], char(J(i,j)));
    end
end
