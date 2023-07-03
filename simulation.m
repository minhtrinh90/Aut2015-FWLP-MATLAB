%% Navigating in the plane using only bearing angle information
% 2023.04.19
% The FWLP with moving beacons
clear all
clc
global P w
w = [1,1,1,1,1,1];
pA = [0;0;0];  % In this simulation, the beacons are stationary
pB = [3;-1;0]; 
pC = [6;1;0];
pD = [5;5;0];
pE = [1;4;0];
pF = [-3;2;0];
P = [pA pB pC pD pE pF];
pc = sum(P,2)/6;
P = P - kron([1,1,1,1,1,1],pc);
pA = pA - pc;
pB = pB - pc;
pC = pC - pc;
pD = pD - pc;
pE = pE - pc;
pF = pF - pc;


p0 = [-4;5;0];
tsim = 0:0.01:10;

[t,p] = ode45(@fwlp_static,tsim,p0);
t_end = length(t);
FWpoint = p(end,:);
p=p';
objFuncP = zeros(1,length(t));
for i=1:t_end
    for j=1:6
        objFuncP(1,i)=objFuncP(1,i)+w(j)*norm(p(:,i)-P(:,j),2);
    end
end

q0 = pA;
[t,q] = ode45(@fwlp_static,tsim,q0);
t_end = length(t);
% FWpoint = q(end,:);
q=q';
objFuncQ = zeros(1,length(t));
for i=1:t_end
    for j=1:6
        objFuncQ(1,i)=objFuncQ(1,i)+w(j)*norm(q(:,i)-P(:,j),2);
    end
end

r0 = [5;3;5];
[t,r] = ode45(@fwlp_static,tsim,r0);
t_end = length(t);
% FWpoint = q(end,:);
r=r';
objFuncR = zeros(1,length(t));
for i=1:t_end
    for j=1:6
        objFuncR(1,i)=objFuncR(1,i)+w(j)*norm(r(:,i)-P(:,j),2);
    end
end

m0 = [5;-5;0];
[t,m] = ode45(@fwlp_static,tsim,m0);
t_end = length(t);
% FWpoint = q(end,:);
m=m';
objFuncM = zeros(1,length(t));
for i=1:t_end
    for j=1:6
        objFuncM(1,i)=objFuncM(1,i)+w(j)*norm(m(:,i)-P(:,j),2);
    end
end

% figure(1)
% hold on
% plot(p(1,1), p(2,1), 'bx','LineWidth', 1);
% plot(p(1,t_end), p(2,t_end), 'bo','LineWidth', 1);
% plot(p(1,1:t_end), p(2,1:t_end), 'b-','LineWidth', 1);
% plot(pA(1,1), pA(2,1), 'ok','LineWidth', 1);
% plot(pB(1,1), pB(2,1), 'ok','LineWidth', 1);
% plot(pC(1,1), pC(2,1), 'ok','LineWidth', 1);
% plot(pD(1,1), pD(2,1), 'ok','LineWidth', 1);
% plot(pE(1,1), pE(2,1), 'ok','LineWidth', 1);
% plot(pF(1,1), pF(2,1), 'ok','LineWidth', 1);
% grid on
% 
% line([pA(1,1), pB(1,1)],[pA(2,1), pB(2,1)],'LineStyle','-.','Color','k');
% line([pC(1,1), pB(1,1)],[pC(2,1), pB(2,1)],'LineStyle','-.','Color','k');
% line([pC(1,1), pD(1,1)],[pC(2,1), pD(2,1)],'LineStyle','-.','Color','k');
% line([pE(1,1), pD(1,1)],[pE(2,1), pD(2,1)],'LineStyle','-.','Color','k');
% line([pF(1,1), pE(1,1)],[pF(2,1), pE(2,1)],'LineStyle','-.','Color','k');
% line([pA(1,1), pF(1,1)],[pA(2,1), pF(2,1)],'LineStyle','-.','Color','k');
% box on
% axis equal
% xlabel x
% ylabel y
% title('Convergence to the Fermat-Weber point')
% 
% figure(2)
% hold on
% plot(t, objFunc, 'b-','LineWidth', 2);

%% Simulation animation
F(t_end) = struct('cdata',[],'colormap',[]);
v = VideoWriter('simFWLP.avi');
v.FrameRate = 20;
open(v);

figure(1); clf
hold on
% t_end = N;
% subplot(2,2,1)
% hold on
%     plot3(pA(1,1), pA(2,1), pA(3,1), 'ok','LineWidth', 1);
%     plot3(pB(1,1), pB(2,1), pB(3,1), 'ok','LineWidth', 1);
%     plot3(pC(1,1), pC(2,1), pC(3,1), 'ok','LineWidth', 1);
%     plot3(pD(1,1), pD(2,1), pD(3,1), 'ok','LineWidth', 1);
%     plot3(pE(1,1), pE(2,1), pE(3,1), 'ok','LineWidth', 1);
%     plot3(pF(1,1), pF(2,1), pF(3,1), 'ok','LineWidth', 1);
%     view(3)
%     %axis equal
%     xlabel x
%     ylabel y
%     zlabel z
%     ax = gca;
%     ax.BoxStyle ='full';
% % subplot(1,2,2)
% subplot(2,2,2)
% hold on
%     plot3(pA(1,1), pA(2,1), pA(3,1), 'ok','LineWidth', 1);
%     plot3(pB(1,1), pB(2,1), pB(3,1), 'ok','LineWidth', 1);
%     plot3(pC(1,1), pC(2,1), pC(3,1), 'ok','LineWidth', 1);
%     plot3(pD(1,1), pD(2,1), pD(3,1), 'ok','LineWidth', 1);
%     plot3(pE(1,1), pE(2,1), pE(3,1), 'ok','LineWidth', 1);
%     plot3(pF(1,1), pF(2,1), pF(3,1), 'ok','LineWidth', 1);
%     view(3)
%     %axis equal
%     xlabel x
%     ylabel y
%     zlabel z
%     ax = gca;
%     ax.BoxStyle ='full';

for i=1:find(t==5)
    figure(1);
    delete(subplot(1,2,1))
    subplot(1,2,1)
    hold on
    line([pA(1,1), pB(1,1)],[pA(2,1), pB(2,1)],[pA(3,1), pB(3,1)],'LineStyle','--','Color','k');
    line([pC(1,1), pB(1,1)],[pC(2,1), pB(2,1)],[pC(3,1), pB(3,1)],'LineStyle','--','Color','k');
    line([pC(1,1), pD(1,1)],[pC(2,1), pD(2,1)],[pC(3,1), pD(3,1)],'LineStyle','--','Color','k');
    line([pE(1,1), pD(1,1)],[pE(2,1), pD(2,1)],[pE(3,1), pD(3,1)],'LineStyle','--','Color','k');
    line([pF(1,1), pE(1,1)],[pF(2,1), pE(2,1)],[pF(3,1), pE(3,1)],'LineStyle','--','Color','k');
    line([pA(1,1), pF(1,1)],[pA(2,1), pF(2,1)],[pA(3,1), pF(3,1)],'LineStyle','--','Color','k');
    step = num2str(t(i));
    plot3(p(1,1), p(2,1), p(3,1), 'gx','LineWidth', 1);
    plot3(p(1,i), p(2,i), p(3,i), 'go','LineWidth', 1);
    plot3(p(1,1:i), p(2,1:i), p(3,1:i), 'g-','LineWidth', 1);

    plot3(q(1,1), q(2,1), q(3,1), 'rx','LineWidth', 1);
    plot3(q(1,i), q(2,i), q(3,i), 'ro','LineWidth', 1);
    plot3(q(1,1:i), q(2,1:i), q(3,1:i), 'r-','LineWidth', 1);

    plot3(r(1,1), r(2,1), r(3,1), 'bx','LineWidth', 1);
    plot3(r(1,i), r(2,i), r(3,i), 'bo','LineWidth', 1);
    plot3(r(1,1:i), r(2,1:i), r(3,1:i), 'b-','LineWidth', 1);

    plot3(m(1,1), m(2,1), m(3,1), 'mx','LineWidth', 1);
    plot3(m(1,i), m(2,i), m(3,i), 'mo','LineWidth', 1);
    plot3(m(1,1:i), m(2,1:i), m(3,1:i), 'm-','LineWidth', 1);

    plot3(pA(1,1), pA(2,1), pA(3,1), 'ok','LineWidth', 1);
    plot3(pB(1,1), pB(2,1), pB(3,1), 'ok','LineWidth', 1);
    plot3(pC(1,1), pC(2,1), pC(3,1), 'ok','LineWidth', 1);
    plot3(pD(1,1), pD(2,1), pD(3,1), 'ok','LineWidth', 1);
    plot3(pE(1,1), pE(2,1), pE(3,1), 'ok','LineWidth', 1);
    plot3(pF(1,1), pF(2,1), pF(3,1), 'ok','LineWidth', 1);
    % grid on
   
    %
    xlabel x
    ylabel y
    zlabel z
    box on
    ax = gca;
    ax.BoxStyle ='full';
    view(3)
    axis equal
    title(strcat('Position at t = ',step,' s'));
    % drawnow 

    % drawnow limitrate

    %nexttile
    delete(subplot(1,2,2))
    subplot(1,2,2)
    
    hold on
    plot(t(1:i), objFuncP(1,1:i), 'g-','LineWidth', 2);
    plot(t(1:i), objFuncQ(1,1:i), 'r-','LineWidth', 2);
    plot(t(1:i), objFuncR(1,1:i), 'b-','LineWidth', 2);
    plot(t(1:i), objFuncM(1,1:i), 'm-','LineWidth', 2);
    xlim([0,5]);
    legend({'$f(p)=\sum_{i=1}^n\omega_i d_i$'},'Interpreter','latex','FontSize',16)
    xlabel('$t (s)$','Interpreter','latex','FontSize',16)
    ylabel('$f(p(t))$','Interpreter','latex','FontSize',16)
    box on
    grid on
    %axis equal
    set(gcf, 'color', 'white')
    drawnow % limitrate
    

    F = getframe(gcf);
    writeVideo(v,F);
end

F = getframe(gcf);
writeVideo(v,F);
close(v);




