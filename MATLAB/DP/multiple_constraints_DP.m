%Testing multi-objective optimization
clear
figure(1)
clf

%build adjacency matrix

% %sample graph from drawing
% coords = [1,1; 3,5; 4,0; 8,4; 8,1.5; 10,0; 12,4; 13,1; 16,3];
% V = 9; %number of vertices
% %E(1,:) = [v_start, v_end]
% E = [1,2; 1,3; 2,3; 3,4; 3,5; 4,5; 4,6; 4,7; 5,6; 6,7; 7,8; 7,9; 8,9];  
% P_tr = sqrt(rand(V,1)); %Prob. of traverse associated with each node. sqrt is to bias towards higher values.
% adj = zeros(V);
% d = zeros(V,2); %d(1) is minimum cost, d(2) is total P_tr
% par_point = V*ones(V,1);
% for i=1:V
%     for j=1:size(E,1)
%        if E(j,1) == i
%           adj(i, E(j,2)) = abs(10*randn(1));
%        end
%     end
% end
% gplot(adj, coords, '*-')
% % wgPlot(adj, coords)
% %

% This creates the 'background' axes
ha = axes('units','normalized','position',[0 0 1 1]);

% Move the background axes to the bottom
uistack(ha,'bottom');

% Load in a background image and display it using the correct colors
% The image used below, is in the Image Processing Toolbox.  If you do not have %access to this toolbox, you can use another image file instead.
I=imread('RCP_google_maps.png');
hi = imagesc(I);
colormap gray

% Turn the handlevisibility off so that we don't inadvertently plot into the axes again
% Also, make the axes invisible
set(ha,'handlevisibility','off','visible','off')

% Now we can use the figure, as required.
% For example, we can put a plot in an axes
axes('position',[0.03,0.075,1,0.7])
% plot(rand(10))

%square grid, connected right and down
V = 100;
adj = zeros(V);
for i = 1:V-1
   if (mod(i,sqrt(V)) >0)
    adj(i, i+1)=1;
   end
   if (i<=(V- sqrt(V)))
    adj(i, i+sqrt(V))=1;
   end
end
d = zeros(V,2); %d(1) is minimum cost, d(2) is total P_tr

% adj = abs(10*randn(V)).*adj;
P_tr_thresh = 0.25;
P_tr = sqrt(rand(V,1)); %Prob. of traverse associated with each node. sqrt is to bias towards higher values.
costs = [.2 .7 .8 .7 .6 .2 .2 .3 .2 .1
       .5 .9 .7 .7 .3 .4 .5 .3 .1 .1
       .7 .6 .3 .2 .7 .5 .3 .1 .1 .1
       .2 .4 .9 .8 .5 .2 .2 .2 .1 .1
       .8 .9 .7 .4 .2 .1 .2 .3 .1 .8
       .2 .1 .1 .2 .8 .5 .4 .4 .1 .8
       .1 .6 .5 .6 .5 .4 .4 .2 .1 .1
       .3 .2 .1 .1 .1 .1 .2 .3 .1 .1
       .2 .2 .1 .1 .1 .2 .1 .1 .1 .1
       .1 .2 .2 .2 .4 .2 .1 .1 .1 .1];
costs = reshape(costs, 1, 100);       
for i=1:size(adj,1)
   adj(i,:) = costs.*adj(i,:); 
end 


%create coordinates
coords = [1 sqrt(V)];
for i = 1:V-1
   coords(i+1,:) = [floor(i/sqrt(V))+1, sqrt(V) - mod(i,sqrt(V))];
end

 gplot(adj, coords, '*-')

P_tr(V) = 1;
d(V,2) = 1;
for i=V-1:-1:1
    conns = find(adj(i,:)~=0);  %find non-zero entries in adj
    options = d(conns,1) + adj(i,conns)'; %list all options, find cost
    options(:,2) = P_tr(i)*d(conns,2);  %calculate P_tr for options
    for k=1:size(options,1) %if option violates P_tr, set penalty cost
        if options(k,2) < P_tr_thresh
            options(k,1) = 1000;         
        end
    end
    [d(i,1), ind] = min(options(:,1));  %choose minimum option
    par_point(i) = conns(ind);          %set parent for current node
    d(i,2) = P_tr(par_point(i))*P_tr(i);%calculate cost for current node.
end

t = 1;
path = [1];
while (t<V)
    path(end+1) = par_point(t);
    t = path(end);
end
path
cost = d(1,1)
Prob_traverse = d(1,2)
d;
hold on
for i=1:length(path)-1
    plot(coords(path(i),1), coords(path(i),2), 'r*')
    plot([coords(path(i),1), coords(path(i+1),1)],[coords(path(i),2), coords(path(i+1),2)], 'r-')    
end
axis off
axis equal

axis off
axis equal
figure(2)
clf
%  wgPlot(adj, coords)
% wgPlot(adj, coords,'vertexScale',100);

% m = max(max(adj));
% hold on
% for i=1:V
%     for j=1:V
%         if adj(i, j)>0
%             red = adj(i,j)/m;
%             blue = (P_tr(i) + P_tr(j))/2;
%             plot([coords(i,1), coords(j, 1)], [coords(i,2), coords(j,2)], 'color', [red, 0, blue])
%      
%         end
%         val = num2str(costs(i));
%         text(coords(i,1), coords(i,2), val)
%     end
% end
% axis off
% axis equal

figure(2)
clf
subplot(1,2,1)
for i = 1:V
    v = [ coords(i,2)-0.5 coords(i,1)-0.5; coords(i,2)-0.5 coords(i,1)+0.5; coords(i,2)+0.5 coords(i,1)+0.5; coords(i,2)+0.5  coords(i,1)-0.5 ];
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [0 0 1 ], 'FaceAlpha', 1 - costs(i))
    
end

axis off
axis equal
axis([0 11 0 11])
title('Cost Map')
% figure(4)
% clf
subplot(1,2,2)
for i = 1:V
    v = [ coords(i,2)-0.5 coords(i,1)-0.5; coords(i,2)-0.5 coords(i,1)+0.5; coords(i,2)+0.5 coords(i,1)+0.5; coords(i,2)+0.5  coords(i,1)-0.5 ];
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [1 0 0 ], 'FaceAlpha', 1 - P_tr(i))
end


axis off
axis equal
axis([0 11 0 11])
title('Map of P_{tr}(x)')
