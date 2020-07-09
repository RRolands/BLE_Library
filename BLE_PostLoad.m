AdvA_List=[];
XY=[];
Names=find_system(bdroot,'regexp','on','Name','BLE master');
if ~isempty(Names)
    for k=1:length(Names)
        
        add=get_param(Names{k},'UserData');
        if ~isempty(add)
            AdvA_List=[AdvA_List; add.hex];
        else
            [RandA_bin,RandA_hex, AdvA_List] = GenStatDevAddr(AdvA_List);
            set_param(Names{k},'UserData',struct('bin',RandA_bin,'hex',RandA_hex));
            set_param(Names{k},'InitA_R',RandA_hex);
        end
        
        xy=get_param(Names{k},'Position');
        XY=[XY;xy(1:2)];
    end
end

Names=find_system(bdroot,'regexp','on','Name','BLE slave');
if ~isempty(Names)
    for k=1:length(Names)
        
        add=get_param(Names{k},'UserData');
        if ~isempty(add)
            AdvA_List=[AdvA_List; add.hex];
        else
            [RandA_bin,RandA_hex, AdvA_List] = GenStatDevAddr(AdvA_List);
            set_param(Names{k},'UserData',struct('bin',RandA_bin,'hex',RandA_hex));
            set_param(Names{k},'AdvA_R',RandA_hex);
        end
        
        xy=get_param(Names{k},'Position');
        XY=[XY;xy(1:2)];
    end
end

Names=find_system(bdroot,'regexp','on','Name','interferer');
if ~isempty(Names)
    for k=1:length(Names)
        
        xy=get_param(Names{k},'Position');
        XY=[XY;xy(1:2)];
    end
end

assignin('base', 'AdvA_List', AdvA_List);

if ~isempty(XY)
    N_BLE=size(XY,1)-1;
    D=zeros(N_BLE,N_BLE+1);
    for n=1:size(XY,1)
        for k=1:size(XY,1)
            D(n,k)=sqrt((XY(n,1)-XY(k,1))^2+(XY(n,2)-XY(k,2))^2);
        end
    end
    ScaleCoefficient=0;
    varList=whos(get_param(bdroot,'ModelWorkspace'));
    if ~isempty(varList)
        for m=1:length(varList)
            if strcmp(varList(m).name,'ScaleCoefficient')
                ScaleCoefficient=getVariable(get_param(bdroot,'ModelWorkspace'),'ScaleCoefficient');
                break;
            end
        end
    end
    if ScaleCoefficient==0
        ScaleCoefficient=1;
    end
    D=D/ScaleCoefficient;
    
    % calculate the Free Space Path Loss
    D_path_loss=1./(4*pi*D*2.4e9/299792458);
    D_path_loss(D_path_loss==Inf)=0;
    assignin('base', 'D', D');
    assignin('base', 'D_path_loss_I_on', D_path_loss');
    D_path_loss(:,end)=0;
    assignin('base', 'D_path_loss_I_off', D_path_loss');
end