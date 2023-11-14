N = 8; %number of rays to propagate

pos1 = [0,0,0]; %xyz position vector of a point on an object
pos2 = [10,0,0];

angles = linspace(-pi/20,pi/20,N); %create vector storing the angles

d = 0.005; %propagation of each ray

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
plot(ray_z, [raysIn1(1,:); raysOut1(1,:)],'Color','#DE4929');