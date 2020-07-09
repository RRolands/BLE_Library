function GenAccA(block)

% function generates an Access Address,
% given the list of AccA_List (workspace)

setup(block)

function setup(block)

% Register number of ports
block.NumInputPorts = 0;
block.NumOutputPorts = 1;

block.SetPreCompOutPortInfoToDynamic;
block.SimStateCompliance = 'DefaultSimState';

% Specify inport properties
% block.InputPort(1).Dimensions = [32,1];
% block.InputPort(1).DatatypeID = 0; %double
% block.InputPort(1).Complexity = 'Real';
% block.InputPort(1).DirectFeedthrough = false;

% Specify outport properties
block.OutputPort(1).Dimensions = [32,1];
block.OutputPort(1).DatatypeID = 8; %boolean
% block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

% Register the parameters
block.NumDialogPrms = 1;
block.DialogPrmsTunable = {'Tunable'};
%block.NumDialogPrms = 3;
%block.DialogPrmsTunable = {'Tunable','Nontunable','SimOnlyTunable'};

% Register sample times to Discrete sample time
block.SampleTimes = [block.DialogPrm(1).Data 0];

% Register all methods to be used in this file
block.RegBlockMethod('Outputs', @outputs);

function outputs(block)

%AccA_List=evalin('base',block.DialogPrm(1).Data);

h=get_param(get_param(get_param(get_param(gcb,'Parent'),'Parent'),'Parent'),'Parent');
uData=get_param(h,'UserData');
if isfield(uData,'AccA_List')
    AccA_List=uData.AccA_List;
else
    AccA_List=[];
end

AdvAccA=dec2bin(hex2dec('8E89BED6'),32); %Advertising channel Access Address

rng('shuffle');
A=round(rand(32,1));

AccA_hex=dec2hex(bin2dec(num2str(A')),8);
AccA_bin=dec2bin(hex2dec(AccA_hex),32);

while 1>0
    
    flag=0;
    
    for k=1:size(AccA_List,1)
        if strcmp(AccA_hex,AccA_List(k,:))
            flag=1;
            break
        end
    end
    
    if flag==0
        for k=1:27
            if sum(AccA_bin(k:k+5)=='000000')==6 || sum(AccA_bin(k:k+5)=='111111')==6
                flag=1;
                break
            end
        end
    end
    
    if flag==0
        if strcmp(AccA_bin(1:8),AccA_bin(9:16))+strcmp(AccA_bin(1:8),AccA_bin(17:24))+strcmp(AccA_bin(1:8),AccA_bin(25:32))==3
            flag=1;
        end
    end
    
    if flag==0
        N_transitions=0;
        for k=1:31
            if AccA_bin(k)~=AccA_bin(k+1)
                N_transitions=N_transitions+1;
            end
        end
        if N_transitions>24
            flag=1;
        end
    end
    
    if flag==0
        N_transitions=0;
        for k=1:5
            if AccA_bin(k)~=AccA_bin(k+1)
                N_transitions=N_transitions+1;
            end
        end
        if N_transitions<2
            flag=1;
        end
    end
    
    if flag==0
        if sum(AccA_bin==AdvAccA)>30
            flag=1;
        end
    end   
    
    if flag==1
        A=round(rand(32,1));
        AccA_hex=dec2hex(bin2dec(num2str(A')),8);
        AccA_bin=dec2bin(hex2dec(AccA_hex),32);
    else
        block.OutputPort(1).Data=false(32,1);
        block.OutputPort(1).Data(A>0)=true;
        break;
    end
end

uData.AccA_List=[AccA_List; AccA_hex]; %update Access Address list
set_param(h,'UserData',uData)

%assignin('base','AccA_List',[AccA_List; AccA_hex]); %update Access Address list