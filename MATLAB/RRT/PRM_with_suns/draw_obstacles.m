function [ obst, obst_ptsx, obst_ptsy ] = draw_obstacles( num_obst, course, obst )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% close all


 if nargin <3
     % % Uncomment here for random obstacles, and  comment out below.
% obst = [];
% nsides = round(rand(1, num_obst)*5 + 1); %number of sides in each obstacle
% lsides = rand(1, num_obst)*5; %length of each side.
% points = rand(2, num_obst);%starting points

% x = points(1, :) .*(course(3) - course(1)) + course(1);
% y = points(2, :) .*(course(4) - course(2)) + course(2);


% % Comment here for random obstacles
num_obst = 4;
obst(1, :) = [5 25 15 25];
nsides = [6 4 3 6];
lsides = [4 6 5 5];
points = [30  25  35 10; 
          20  10  5  5 ];

x = points(1, :) ;
y = points(2, :) ;       
obst_ptsx = 15;     %initialize the variable to the point that is missed when creating list of obstacles.
obst_ptsy = 25;
for v = 1:num_obst
    x0 = x(v);
    y0 = y(v);
    for w = 1: nsides(v)
        phi = (180 - 180*(nsides(v) - 2)/nsides(v))*(w-1);
        xn = x0 + lsides(v)*cosd(phi);
        yn = y0 + lsides(v)*sind(phi);
        obst(end +1, :) = [x0 y0 xn yn];
        x0 = xn;
        y0 = yn;
      obst_ptsx(v, end +1) = xn;
      obst_ptsy(v, end +1) = yn;
    end
end

obst(end +1, :) = [course(1) course(2) course(3) course(2)];
obst(end +1, :) = [course(3) course(2) course(3) course(4)];
obst(end +1, :) = [course(3) course(4) course(1) course(4)];
obst(end +1, :) = [course(1) course(4) course(1) course(2)];

 end
%  hold on %uncomment for testing purposes.
    for z = 1:length(obst)
    plot([obst(z,1); obst(z,3)],[obst(z,2);obst(z,4)], 'k', 'LineWidth', 3)
    end
    
end

