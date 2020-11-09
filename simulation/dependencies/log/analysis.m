clf;

x = importdata('log_calibration2.txt');
x = x(1:2:end,:);

time = x(:,1);
index = time>20 & time<40;
phase = x(:,2);

elx = x(index & phase<0.5, 3:6);
erx = x(index & phase>0.5, 7:10);
ely = x(index & phase<0.5, 11:14);
ery = x(index & phase>0.5, 15:18);

x = {elx, erx, ely, ery};
names = {'elx', 'erx', 'ely', 'ery'};
for i=1:4
    for j=1:4
        subplot(4,4,i*4-4+j); hold on;
        X = x{i}(:,j);
        hist(X,100);
%         if j<=2
%             %xlim([-0.05 0.05]);
%         else
%             %xlim([-0.4 0.4]);
%         end
        [m,s] = normfit(X);
%         disp([m s]);
        Y = normpdf(X,m,s);
        plot(X,Y,'.');
        disp(sprintf('%s[%d] = deadzone(%s[%d]-(%3.3f), %3.3f);', ...
            names{i},j-1,names{i},j-1,m,s));
    end
end