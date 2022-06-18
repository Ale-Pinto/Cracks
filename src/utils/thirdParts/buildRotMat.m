function R = buildRotMat(angles)

% buildRotMat build rotation matrix from
% 
% xyz=d3trafo(XYZ,p,O,dir,FileOut)
%  Inputs:  angles  3x1-vector containing rotation angles [ex ey ez] in rad        
% 
% Output:  R  3x3 Rotation Matrix
%
%  R =  cos(ey) cos(ez)   cos(ex) sin(ez) + cos(ez) sin(ex) sin(ey)   sin(ex) sin(ez) − cos(ex) cos(ez) sin(ey) 
%       −cos(ey) sin(ez)  cos(ex) cos(ez) − sin(ex) sin(ey) sin(ez)   cos(ez) sin(ex) + cos(ex) sin(ey) sin(ez)
%       sin(ey)        −cos(ey) sin(ex)                   cos(ex) cos(ey)
% 
% Francesco Ioli
% 2022.03.09

ex = angles(1);
ey = angles(2);
ez = angles(3);

R =  [ cos(ey)*cos(ez)     cos(ex)*sin(ez)+cos(ez)*sin(ex)*sin(ey)     sin(ex)*sin(ez)-cos(ex)*cos(ez)*sin(ey);
    -cos(ey)*sin(ez)    cos(ex)*cos(ez)-sin(ex)*sin(ey)*sin(ez)     cos(ez)*sin(ex)+cos(ex)*sin(ey)*sin(ez);
    sin(ey)             -cos(ey)*sin(ex)                            cos(ex)*cos(ey) ];

end