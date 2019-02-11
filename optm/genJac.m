
syms u0 v0 au av sk real
syms tx ty tz wx wy wz real
syms X Y um vm real
syms k1 k2

% the intrinsic parameter matrix
K=[au sk u0;
   0 av v0;
   0 0 1];
% Expression for the rotation matrix based on the Rodrigues formula
theta=sqrt(wx^2+wy^2+wz^2);
omega=[0 -wz wy;
       wz 0 -wx;
       -wy wx 0;];
R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
>> % Expression for the translation vector
t=[tx;ty;tz];

% perspective projection of the model point (X,Y)
uvs=K*[R(:,1) R(:,2) t]*[X; Y; 1];
u=uvs(1)/uvs(3);
v=uvs(2)/uvs(3);

% calculate the geometric distance in x and y direction
% um,vm =the x and y positions of an extracted corner
% u,v = the x and y positions of the projection of the corresponding model point

%radial distortion
tpU = (u - u0);
tpV = (v - v0);
%r = (tpU / au)*(tpU / au) + (tpV / av)*(tpV / av);
r = u*u+v*v;

ud = u+tpU*(k1*r + k2*r*r);
vd = v+tpV*(k1*r + k2*r*r);

%dx=um-u;
%dy=vm-v;

dy=vm-vd;
dx = um-ud;

Jx=jacobian(dx,[au,av,u0,v0,sk,wx wy wz tx ty tz k1 k2]);
Jy=jacobian(dy,[au,av,u0,v0,sk,wx wy wz tx ty tz k1 k2]);
