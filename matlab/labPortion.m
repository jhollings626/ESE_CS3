%% Ray Propagation Through Free Space - Pt. 1

N = 8; %number of rays to propagate

pos1 = [0,0,0]; %xyz position vector of a point on an object
pos2 = [0.01,0,0];

angles = linspace(-pi/20,pi/20,N); %create vector storing the angles

d = 0.2; %propagation of each ray

M = [1 d 0 0;
     0 1 0 0;
     0 0 1 d;
     0 0 0 1;
     ];

raysIn1 = zeros(4,N); %create empty collection of first object state vecs
raysin2 = zeros(4,N); %create empty collection of second object state vecs

for i = 1:N
    raysIn1(1,i) = pos1(1); %initial x pos
    raysIn1(2,i) = angles(i);
    raysIn1(3,i) = pos1(2);
    raysIn1(4,i) = angles(i);
end

raysOut1 = zeros(4,N);

for i = 1:N
    raysOut1(:,i) = M*raysIn1(:,i);
end

ray_z = [zeros(1,size(raysIn1,2)); d*ones(1,size(raysIn1,2))];

for i = 1:N
    raysIn2(1,i) = pos2(1); % updated x pos for second set of rays
    raysIn2(2,i) = angles(i);
    raysIn2(3,i) = pos2(2); % y position remains 0
    raysIn2(4,i) = angles(i); 
end

raysOut2 = zeros(4,N);
for i = 1:N
    raysOut2(:,i) = M*raysIn2(:,i);
end

hold on; 
plot(ray_z, [raysIn1(1,:); raysOut1(1,:)], 'Color', '#DE4929'); % first set of rays
plot(ray_z, [raysIn2(1,:); raysOut2(1,:)], 'Color', '#56B4E9'); % second set of rays
hold off;

title('Ray Propagation through Free Space');
xlabel('Z-Position');
ylabel('X-Position');
legend('Ray 1', 'Ray 2');

%% Ray Tracing Through a Finite-Sized Lens - Pt. 2

f = 0.15; % focal length in meters (negative for a converging lens)
lensRadius = 0.02; % lens radius in meters
lensPosition = 0.2; % position of lens along the z-axis in meters

lensMatrix = [1 0 0 0; %build the matrix for passing through the lens (basically just adjust the slopes of the rays that pass through)
            -1/f 1 0 0;
              0 0 1 0;
              0 0 -1/f 1];

% Update the rays' positions and directions after passing through the lens
for i = 1:N
    if abs(raysOut1(1,i)) <= lensRadius %rays outside of the lens radius don't need to continue
        % Update ray's direction based on its position and the lens's focal length
        raysAfterLens1(:,i) = lensMatrix * [raysOut1(1,i); raysOut1(2,i); raysOut1(3,i); 1];
        % The y-component remains the same as rays are only propagating in the xz-plane
        raysAfterLens1(3,i) = raysOut1(3,i); %yz plane angle is unchanged
    else
        raysAfterLens1(:,i) = [NaN; NaN; NaN; NaN]; %for rays that go past the lens, they don't get more values...
    end
    if abs(raysOut2(1,i)) <= lensRadius %repeat same process for the second ray
        raysAfterLens2(:,i) = lensMatrix * [raysOut2(1,i); raysOut2(2,i); raysOut2(3,i); 1];
        raysAfterLens2(3,i) = raysOut2(3,i);
    else
        raysAfterLens2(:,i) = [NaN; NaN; NaN; NaN];
    end
end

% Propagate the rays after the lens for a distance d2
d2 = 0.65; % further propagation distance in meters, adjust as needed
M2 = [1 d2 0 0;
      0 1 0 0;
      0 0 1 d2;
      0 0 0 1];

%for continued propagation of rays that pass through, following same scheme
%as part 1
for i = 1:N
    if ~isnan(raysAfterLens1(1,i))
        raysAfterLens1(:,i) = M2 * raysAfterLens1(:,i);
    end
    if ~isnan(raysAfterLens2(1,i))
        raysAfterLens2(:,i) = M2 * raysAfterLens2(:,i);
    end
end


figure;
hold on;


viscircles([lensPosition, 0], lensRadius, 'Color','k', 'LineWidth', 1); %draw the lens

plot([0,lensPosition+d2], [0,0], 'Color','k','LineWidth',1); %draw a line that's level with the center of the lens


plot(ray_z, [raysIn1(1,:); raysOut1(1,:)], 'Color', '#DE4929'); %rays from Point 1 before lens
plot(ray_z, [raysIn2(1,:); raysOut2(1,:)], 'Color', '#56B4E9'); %rays from Point 2 before lens

%plot rays that passed through the lens
for i = 1:N
    if ~isnan(raysAfterLens1(1,i)) %if post-lens coordinates exist, plot them (darker than original rays though)
        plot([lensPosition, lensPosition + d2], [raysOut1(1,i), raysAfterLens1(1,i)], 'Color', '#B84329');
    end
    if ~isnan(raysAfterLens2(1,i)) %if post-lens coordinates exist, plot them (darker than originals again)
        plot([lensPosition, lensPosition + d2], [raysOut2(1,i), raysAfterLens2(1,i)], 'Color', ' #56b4e9');
    end
end

title('Ray Propagation Before & After Passing Through a Finite-Sized Lens');
xlabel('Z-Position');
ylabel('X-Position');
legend('Lens', 'Pre-lens Point 1 Rays', 'Pre-lens Point 2 Rays', 'Post-lens Point 1 Rays', 'Post-lens Point 2 Rays');
hold off;
