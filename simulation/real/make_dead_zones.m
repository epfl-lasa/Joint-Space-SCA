x = importdata('data.txt');

phase = x(:,1);
x = x(:,2:end);
maskl = phase < 0.5;
maskr = phase > 0.5;

for i=1:4
disp(sprintf('elx[%d] = deadzone(elx[%d]-(%3.3f), %3.3f);',mod(i-1,4),mod(i-1,4),mean(x(maskl,i)), std(x(maskl,i))));
end
for i=5:8
disp(sprintf('erx[%d] = deadzone(erx[%d]-(%3.3f), %3.3f);',mod(i-1,4),mod(i-1,4),mean(x(maskr,i)), std(x(maskr,i))));
end
for i=9:12
disp(sprintf('ely[%d] = deadzone(ely[%d]-(%3.3f), %3.3f);',mod(i-1,4),mod(i-1,4),mean(x(maskl,i)), std(x(maskl,i))));
end
for i=13:16
disp(sprintf('ery[%d] = deadzone(ery[%d]-(%3.3f), %3.3f);',mod(i-1,4),mod(i-1,4),mean(x(maskr,i)), std(x(maskr,i))));
end