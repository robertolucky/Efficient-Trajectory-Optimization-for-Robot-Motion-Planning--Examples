function [ ret ] = conIneq( Xc,Uc,D,scale,prob )
% inequality constraint
nS = 12; %numero di stati
nU = 6;
npts = numel(Xc)/nS; %numero di nodi
Xc = reshape(Xc,nS,npts); %prime 6 posizioni, seconde 6 velocità
Uc = reshape(Uc,nU,npts);
Acc = (D*(Xc(7:12,:).')/scale).'; %ecco come fa la derivata
Jerk = (D*(Acc.')/scale).';       %usando la matrice D
Ucd = (D*(Uc.')/scale).';      
% pos, velocity, torque constraints, considering motor feedforward
temp = [Xc;...   % temp sta per temporary variable -.-
    Uc;...
    Ucd];


ret = [temp(:);...
    Xc(1:6,1);... % posizini
    Xc(7:12,1);... %velocità
    Acc(:,1);...% accelerazioni
    Jerk(:,1);... %jerk
    prob.rob.tcpPos(Xc(1:6,end));...
    Xc(7:12,end);...
    Acc(:,end);...
    Jerk(:,end);...
    scale*2];

selfcoli=[];
wallcoli=[];
obscoli=[];
selfmap=prob.selfmap;
wallmap=prob.wallmap;
obsmap=prob.obsmap;
for i=1:npts
    prob.rob.jnt_pos=Xc(1:6,i);
    [jntT]=prob.rob.kfwd_rob_full();
    for j=[1,2,3,4,5]
        temp=jntT{j}*[prob.c{j}.';ones(1,length(prob.r{j}))];
        c{j}=temp(1:3,:).';
    end
    
    for k=1:size(selfmap,2)
        lLind=selfmap(k).l(1);
        lBind=selfmap(k).l(2);
        rLind=selfmap(k).r(1);
        rBind=selfmap(k).r(2);
        selfcoli=[selfcoli;norm(c{lLind}(lBind,:)-c{rLind}(rBind,:))-(prob.r{lLind}(lBind)+prob.r{rLind}(rBind))];
    end
    for k=1:size(wallmap,2)
        Lind=wallmap(k).ind(1);
        Bind=wallmap(k).ind(2);
        wallcoli=[wallcoli;c{Lind}(Bind,3).']; % only consider z cood for ground
    end
    for k=1:size(obsmap,2)
        lLind=obsmap(k).l(1);
        lBind=obsmap(k).l(2);
        rBind=obsmap(k).r;
        obscoli=[obscoli;norm(c{lLind}(lBind,:)-prob.obs.c(rBind,:))-(prob.r{lLind}(lBind)+prob.obs.r(rBind))];
    end
end

ret=[ret;selfcoli];
ret=[ret;wallcoli];
ret=[ret;obscoli];


end

