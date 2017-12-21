trajectory = importdata('trajectory.txt');
maxVelocity = importdata('maxVelocity.txt');
plot(maxVelocity(:,1), maxVelocity(:,2), maxVelocity(:,1), maxVelocity(:,3), trajectory(:,1), trajectory(:,2), '.')
