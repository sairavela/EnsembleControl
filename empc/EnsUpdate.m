function [uout] = EnsUpdate(xens,uens,xg,gprec)

nens = size(xens,2);
sel = [1:12];selsz = length(sel);
gprec = gprec(sel,sel);
xg = xg(sel); xens = xens(sel,:);

dxg = xg - xens; dxg(isnan(dxg))=0;
%mean(dxg,2)'

muens = mean(uens,2); duens = uens - muens;
dxens = xens - mean(xens,2);
[ux,sx,vx]= svd(dxens);
%diag(sx)
sqnens = sqrt(nens);
sx = diag(diag(sx))/sqnens;
cux = duens*vx(:,sel)*sx/sqnens;
mdl = cux*pinv(sx.*sx + gprec*eye(selsz))*ux';

% smdl = sign(mdl); amdl = abs(mdl);
% c1 = (amdl(1,1)+amdl(3,1))/2;
% c2 = (amdl(2,2)+amdl(4,2))/2;
% c3 = mean(amdl(:,3));
% c4 = (amdl(2,4)+amdl(4,4))/2;
% c5 = (amdl(1,5)+amdl(3,5))/2;
% c6 = mean(amdl(:,6));
% c7 = (amdl(1,7)+amdl(3,7))/2;
% c8 = (amdl(2,8)+amdl(4,8))/2;
% c9 = mean(amdl(:,9));
% 
% cmdl = [c1 0 c3 0 c5 c6 c7 0 c9;...
%        0 c2 c3 c4 0 c6 0 c8 c9;...
%        c1 0 c3 0 c5 c6 c7 0 c9;...
%        0 c2 c3 c4 0 c6 0 c8 c9];
%    
% fmdl = smdl.*cmdl;

du = mdl(:,sel)*dxg(sel,:);
uout = uens+du;%uout  =muens+cux*icx*dxg;
uout = min(max(uout,0),12); 
 % sdxg = sum(abs(dxg(1:3,:)));
% uout = uens(:,sdxg == min(sdxg));
% xout = xens(:,sdxg == min(sdxg));