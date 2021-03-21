function [ defects ] = conEq( Xc,Uc,D,nS,nU,scale,P )
% equality constraint
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);

defects = (D*(Xc.')/scale).' - robKin(Xc,Uc); % penso sia il difetto di uguaglianza
                    %che ci dovrebbe essere tra la dinamica e la cinematica del robot
defects = defects(:);

end

