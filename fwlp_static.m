function dpdt = fwlp_static(~,p)
global P w
dpdt = [0;0;0];
for i=1:6
    if norm(P(:,i)-p,2)~=0
        dpdt = dpdt + w(i)*(P(:,i)-p)/norm(P(:,i)-p,2);
    end
end
end