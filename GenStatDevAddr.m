function [AdvA_bin, AdvA_hex, AdvA_List] = GenStatDevAddr(AdvA_List) %generate Static Device Address
rng('shuffle');
A=round(rand(1,46));
AdvA_bin=[1 1 A]';
AdvA_hex=dec2hex(bin2dec(num2str(AdvA_bin')),12);

while 1>0
    
    flag=0;
    
    for k=1:size(AdvA_List,1)
        if strcmp(AdvA_hex,AdvA_List(k,:))
            flag=1;
            break
        end
    end
    
    if sum(A)==0 || sum(A)==46 || flag==1
        A=round(rand(1,46));
        AdvA_bin=[1 1 A]';
        AdvA_hex=dec2hex(bin2dec(num2str(AdvA_bin')),12);
    else
        AdvA_List=[AdvA_List; AdvA_hex];
        break;
    end
end

end