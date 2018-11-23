

close all
clear all
clc

%% 2D XYplaine ZVC Earth
mu=.012;
stepX=.002;
stepY=.002
x=-1.5:stepX:1.5;
y=0:stepY:1.3;
holderXY = NaN(length(x),length(y));
for c=1:length(x)
    for d=1:length(y)
        X=[x(c),y(d),0,0,0,0];
        jv=(jacobiValue3D(X,mu));
        if (jv<3.5 && jv>3)
            holderXY(c,d)=jv;
        end
    end
    c/length(x)
end
xyPlot=figure()
contour(x,y,holderXY','DisplayName','Jacobi Contour')
hold on
contour(x,-y,holderXY','DisplayName','Jacobi Contour')
scatter(-mu,0,100,'filled','DisplayName','Earth')
scatter(1-mu,0,100,'filled','DisplayName','Moon')
legend
colorbar
axis([-1.5,1.5,-1.3,1.3])

daspect([1,1,1])
title('Jacobi Contour Earth-Moon System (X-Y View)')
xlabel('X-Axis (ND)')
ylabel('Y-Axis (ND)')


%% 2D XZplaine ZVC Earth
mu=.012;
stepX=.002;
stepY=.002;
x=-1.5:stepX:1.5;
z=0:stepY:0.7;
holderXZ = NaN(length(x),length(z));
for c=1:length(x)
    for d=1:length(z)
        X=[x(c),0,z(d),0,0,0];
        jv=(jacobiValue3D(X,mu));
        if (jv<3.5 && jv>3)
            holderXZ(c,d)=jv;
        end
    end
    1+c/length(x)
end
xzPlot=figure();
contour(x,z,holderXZ','DisplayName','Jacobi Contour')
hold on
contour(x,-z,holderXZ','DisplayName','Jacobi Contour')

xzPlot.Position=[0.0055    0.4105    1.0090    0.4200]*(1e3);
scatter(-mu,0,100,'filled','DisplayName','Earth')
scatter(1-mu,0,100,'filled','DisplayName','Moon')
legend
colorbar
axis([-1.5,1.5,-0.7,0.7])
daspect([1,1,1])
title('Jacobi Contour Earth-Moon System (X-Z View)')
xlabel('X-Axis (ND)')
ylabel('Z-Axis (ND)')

%% Plotting both together (Didn't work)
%{
figure()
subplot(2,1,1)
contour(x,y,holderXY','DisplayName','Jacobi Contour')
hold on
contour(x,-y,holderXY','DisplayName','Jacobi Contour')

scatter(-mu,0,100,'filled','DisplayName','Earth')
scatter(1-mu,0,100,'filled','DisplayName','Moon')
legend
colorbar
axis([-1.5,1.5,-1.3,1.3])

title('Jacobi Contour Earth-Moon System')
daspect([1,1,1])

ylabel('Y-Axis (ND)')
subplot(2,1,2)

contour(x,z,holderXZ','DisplayName','Jacobi Contour')
hold on
contour(x,-z,holderXZ','DisplayName','Jacobi Contour')

scatter(-mu,0,100,'filled','DisplayName','Earth')
scatter(1-mu,0,100,'filled','DisplayName','Moon')
% legend
colorbar
axis([-1.5,1.5,-0.7,0.7])
daspect([1,1,1])
xlabel('X-Axis (ND)')
ylabel('Z-Axis (ND)')
%}
h=figure()
contourf(x,y,holderXY','DisplayName','Jacobi Contour X-Y View')
hold on
h.Position=[4  434  703  400];

contourf(x,-z,holderXZ','DisplayName','Jacobi Contour X-Z View')
plot([-1.5,1.5],[0,0],'r','LineWidth',4)
scatter(-mu,0,100,'filled','DisplayName','Earth')
scatter(1-mu,0,100,'filled','DisplayName','Moon')
colorbar
legend
axis([-1.5,1.5,-0.7,1.3])
title('Combined X-Y X-Z view')
xlabel('X-Axis (ND)')
ylabel('Z-Axis (ND)                     Y-Axis(ND)            ')
daspect([1,1,1])
%% 2D XY plane ZVC Varying Mu
TOG=6
fps = 20

ent=fps*TOG;
muH = linspace(0,1,ent);
holderXYU = NaN(length(x),length(y),ent);
holderXZU = NaN(length(x),length(z),ent);
for e=1:ent
    mu=muH(e);
    for c=1:length(x)
        for d=1:length(y)
            X=[x(c),y(d),0,0,0,0];
            jv=(jacobiValue3D(X,mu));
            if (jv<4 && jv>3)
                holderXYU(c,d,e)=jv;
            end
        end
        
    end
    2+e/ent
end
for e=1:ent
    mu=muH(e);
    for c=1:length(x)
        for d=1:length(z)
            X=[x(c),0,z(d),0,0,0];
            jv=(jacobiValue3D(X,mu));
            if (jv<4 && jv>3)
                holderXZU(c,d,e)=jv;
            end
        end
        
    end
    3+e/ent
end

h = figure; % this ensures that getframe() returns a consistent size
filename = 'xyMuE.gif';
for n = 1:ent
    mu=muH(n);
    contour(x,y,holderXYU(:,:,n)','DisplayName','Jacobi Contour')
    hold on
    contour(x,-y,holderXYU(:,:,n)','DisplayName','Jacobi Contour')
    scatter(-mu,0,70*(1-mu+eps),'b','filled','DisplayName','Primary')
    scatter(1-mu,0,70*(mu+eps),'b','filled','DisplayName','Secondary')
    %legend
    colorbar
    axis([-1.5,1.5,-1.3,1.3])

    daspect([1,1,1])
    title(strcat('Jacobi Contour \mu=',num2str(round(muH(n),2)),' XY-View'))
    xlabel('X-Axis (ND)')
    ylabel('Y-Axis (ND)')
    hold off
    drawnow 
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if n == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/fps); 
      end 
end
  
h = figure; % this ensures that getframe() returns a consistent size

filename = 'xzMuE.gif';
for n = 1:ent
    mu=muH(n);
    h.Position = [404   390   560   420];
    contour(x,z,holderXZU(:,:,n)','DisplayName','Jacobi Contour')
    hold on
    contour(x,-z,holderXZU(:,:,n)','DisplayName','Jacobi Contour')
    scatter(-mu,0,70*(1-mu+eps),'b','filled','DisplayName','Primary')
    scatter(1-mu,0,70*(mu+eps),'b','filled','DisplayName','Secondary')
    %legend
    colorbar
    axis([-1.5,1.5,-0.7,0.7])
    h.Position = [404 387.5000 561 266];
    daspect([1,1,1])
    title(strcat('Jacobi Contour \mu=',num2str(round(muH(n),2)),' XZ-View'))
    xlabel('X-Axis (ND)')
    ylabel('Z-Axis (ND)')
    hold off
    drawnow 
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if n == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/fps); 
      end 
end
  
h = figure; % this ensures that getframe() returns a consistent size

filename = 'xyzMuE.gif';
for n = 1:ent
    mu=muH(n);
    
    contourf(x,y,holderXYU(:,:,n)','DisplayName','Jacobi Contour')
    hold on
    contourf(x,-z,holderXZU(:,:,n)','DisplayName','Jacobi Contour')
    plot([-1.5,1.5],[0,0],'r','LineWidth',4)
    scatter(-mu,0,70*(1-mu+eps),'b','filled','DisplayName','Earth')
    scatter(1-mu,0,70*(mu+eps),'b','filled','DisplayName','Moon')
    %legend
    colorbar
    h.Position = [4 434 703 400];
    
    axis([-1.5,1.5,-0.7,1.3])
    title(strcat('Combined X-Y X-Z View \mu=',num2str(round(muH(n),2))))
    xlabel('X-Axis (ND)')
    ylabel('Z-Axis (ND)                     Y-Axis(ND)            ')
    daspect([1,1,1])
    hold off
    drawnow 
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if n == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/fps); 
      end 
  end