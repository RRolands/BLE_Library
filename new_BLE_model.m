function new_BLE_model(modelname,xy_BLE_master,xy_BLE_slave,xy_Interf) 
% NEW_BLE_MODEL Create a new, empty Simulink model
%    NEW_BLE_MODEL('MODELNAME') creates a new model with
%    the name 'MODELNAME'. Without the 'MODELNAME'
%    argument, the new model is named 'BLE_untitled'.

if nargin == 0 
     modelname = 'BLE_untitled';
end 


% create and open the model
open_system(new_system(modelname));

set_param(modelname,'StopTime','50e-3');

set_param(modelname,'Solver','FixedStepDiscrete');

set_param(bdroot,'FixedStep','Slot_Ts');

set_param(modelname,'PreloadFcn','BLE_PreLoad','PostloadFcn','BLE_PostLoad','StartFcn','BLE_start');


% calculate the distances D between all devices
XY=[xy_BLE_master; xy_BLE_slave; xy_Interf(1,:)];
N_BLE=size(XY,1)-1;
D=zeros(N_BLE,N_BLE+1);
for n=1:size(XY,1)
    for k=1:size(XY,1)
        D(n,k)=sqrt((XY(n,1)-XY(k,1))^2+(XY(n,2)-XY(k,2))^2);
    end
end

% calculate the Free Space Path Loss
D_path_loss=1./(4*pi*D*2.4e9/299792458);
D_path_loss(D_path_loss==Inf)=0;
assignin('base', 'D', D');
assignin('base', 'D_path_loss_I_on', D_path_loss');
D_path_loss(:,end)=0;
assignin('base', 'D_path_loss_I_off', D_path_loss');


% number of BLE master devices
N_BLE_M=size(xy_BLE_master,1);


% find the minimum distance D_min for scaling purpose
D_min=D(:);
D_min(D_min==0)=[];
D_min=min(D_min);


% scale the distances for placing the devices in the model
ScaleCoefficient=4*100/D_min; 
% save ScaleCoefficient to model workspace
assignin(get_param(modelname,'ModelWorkspace'),'ScaleCoefficient',ScaleCoefficient);
xy_BLE_master_hat=round(xy_BLE_master*ScaleCoefficient);
xy_BLE_slave_hat=round(xy_BLE_slave*ScaleCoefficient);
xy_Interf_hat=round(xy_Interf(1,:)*ScaleCoefficient);


% define the sizes of the elements in the model
Size_BLE=[100,134];
Size_GotoTx=[60,16];
Size_GotoDiagn=[34,20];
Size_Switch=[30,36];
Size_VectConcat=[5,40];
Size_Sensor=[50,16];
Size_BatCharg=[45,24];
Size_Channel=[100,40*N_BLE];
Size_In=[30,16];
Size_Out=[30,16];
Size_Selector=[50,16];
Size_Gain=[40,40];
Size_AWGN=[50,30];
Size_Interf=[50,50];
Size_ChannelMux=[5,70*(N_BLE+1)];
Size_ChannelMatrix=[90,16];
Size_ChannelProduct=[60,150];
Size_ChannelDemux=[5,70*N_BLE];
Size_SwitchI=[50,70];
Size_GenParam=[100,40];
Size_Diagnostics=[290,50];
Size_DDisplay=[60,55];


% place the channel block in the model

%%% find the coordinates for placing the floating blocks (optional)
% XY_hat=[xy_BLE_master_hat; xy_BLE_slave_hat; xy_Interf_hat];
% bound_min=min(XY_hat);
% bound_max=max(XY_hat);
% map=zeros(bound_max-bound_min+[0,Size_BLE(2)])';
% offset=-bound_min+1;
% XY_hath=XY_hat+repmat(offset,size(XY_hat,1),1);
% for k=1:N_BLE_M
%     map(XY_hath(k,2):min(size(map,1),XY_hath(k,2)+Size_BLE(2)),max(1,XY_hath(k,1)-20-Size_GotoTx(1)):min(size(map,2),XY_hath(k,1)+Size_BLE(1)+20+Size_GotoTx(1)))=1;
% end
% for k=N_BLE_M+1:N_BLE_M+size(xy_BLE_slave_hat,1)
%     map(XY_hath(k,2):min(size(map,1),XY_hath(k,2)+Size_BLE(2)),max(1,XY_hath(k,1)-40-Size_BatCharg(1)-Size_Switch(1)):min(size(map,2),XY_hath(k,1)+Size_BLE(1)+20+Size_GotoTx(1)))=1;
% end
% map(XY_hath(k+1,2):min(size(map,1),XY_hath(k+1,2)+Size_Interf(2)),XY_hath(k+1,1):min(size(map,2),XY_hath(k+1,1)+Size_Interf(1)+20+Size_GotoTx(1)))=1;
% kk=conv2(map,ones(Size_Channel(2)+100,490+2*Size_GotoTx(1)+Size_Diagnostics(1)+Size_Channel(1)+100),'valid');
% [y,x]=find(kk==0);
% xy_Channel=[x(1),y(1)]-offset+[470+Size_GotoTx(1)+Size_Diagnostics(1),0]+[50,50];

xy_Channel=min([xy_BLE_master_hat;xy_BLE_slave_hat;xy_Interf_hat])+[370+Size_GotoTx(1)+Size_Diagnostics(1)+50,-Size_Channel(2)-150];
add_block('BLE_Library/Channel',[modelname,'/Channel'],'Position',[xy_Channel, xy_Channel+Size_Channel],'LinkStatus','none');

xy_ChannelProduct=[300,300];
add_block('simulink/Math Operations/Product',[modelname,'/Channel/Channel Product'],'Multiplication','Matrix(*)','position',[xy_ChannelProduct, xy_ChannelProduct+Size_ChannelProduct]);
ChannelProduct_PortHandles=get_param([modelname,'/Channel/Channel Product'],'PortHandles');
xy_ChannelProduct_Inport1=get_param(ChannelProduct_PortHandles.Inport(1),'Position');
xy_ChannelProduct_Inport2=get_param(ChannelProduct_PortHandles.Inport(2),'Position');
xy_ChannelProduct_Outport=get_param(ChannelProduct_PortHandles.Outport,'Position');

xy_SwitchI=[xy_ChannelProduct_Inport2(1)-40-Size_SwitchI(1), xy_ChannelProduct_Inport2(2)-Size_SwitchI(2)/2];
add_block('simulink/Signal Routing/Switch',[modelname,'/Channel/SwitchI'],'ShowName','off');
set_param([modelname,'/Channel/SwitchI'],'position',[xy_SwitchI, xy_SwitchI+Size_SwitchI]);
SwitchI_PortHandles=get_param([modelname,'/Channel/SwitchI'],'PortHandles');
xy_SwitchI_Inport1=get_param(SwitchI_PortHandles.Inport(1),'Position');
xy_SwitchI_Inport2=get_param(SwitchI_PortHandles.Inport(2),'Position');
xy_SwitchI_Inport3=get_param(SwitchI_PortHandles.Inport(3),'Position');
add_line([modelname,'/Channel'], [xy_ChannelProduct_Inport2(1)-40,xy_ChannelProduct_Inport2(2); xy_ChannelProduct_Inport2]);

xy_ChannelMatrix_Ion=[xy_SwitchI_Inport1(1)-40-Size_ChannelMatrix(1), xy_SwitchI_Inport1(2)-Size_ChannelMatrix(2)/2];
add_block('simulink/Sources/Constant',[modelname,'/Channel/Channel Matrix I on'],'ShowName','off','Value','D_path_loss_I_on','SampleTime','Slot_Ts','Position',[xy_ChannelMatrix_Ion, xy_ChannelMatrix_Ion+Size_ChannelMatrix]);
add_line([modelname,'/Channel'], [xy_SwitchI_Inport1(1)-40,xy_SwitchI_Inport1(2); xy_SwitchI_Inport1]);

xy_Ion=[xy_SwitchI_Inport2(1)-40-Size_ChannelMatrix(1), xy_SwitchI_Inport2(2)-Size_ChannelMatrix(2)/2];
add_block('simulink/Sources/Constant',[modelname,'/Channel/Ionoff'],'ShowName','off','Value','InterfOn','SampleTime','Slot_Ts','Position',[xy_Ion, xy_Ion+Size_ChannelMatrix]);
add_line([modelname,'/Channel'], [xy_SwitchI_Inport2(1)-40,xy_SwitchI_Inport2(2); xy_SwitchI_Inport2]);

xy_ChannelMatrix_Ioff=[xy_SwitchI_Inport3(1)-40-Size_ChannelMatrix(1), xy_SwitchI_Inport3(2)-Size_ChannelMatrix(2)/2];
add_block('simulink/Sources/Constant',[modelname,'/Channel/Channel Matrix I off'],'ShowName','off','Value','D_path_loss_I_off','SampleTime','Slot_Ts','Position',[xy_ChannelMatrix_Ioff, xy_ChannelMatrix_Ioff+Size_ChannelMatrix]);
add_line([modelname,'/Channel'], [xy_SwitchI_Inport3(1)-40,xy_SwitchI_Inport3(2); xy_SwitchI_Inport3]);

xy_ChannelMux=[xy_ChannelProduct_Inport1(1)-300, xy_ChannelProduct_Inport1(2)-Size_ChannelMux(2)/2];
add_block('simulink/Signal Routing/Vector Concatenate',[modelname,'/Channel/ChannelMux'],'ShowName','off','NumInputs',num2str(N_BLE+1),'Mode','Multidimensional array','ConcatenateDimension','2','Position',[xy_ChannelMux, xy_ChannelMux+Size_ChannelMux]);
ChannelMux_PortHandles=get_param([modelname,'/Channel/ChannelMux'],'PortHandles');
xy_ChannelMux_Outport=get_param(ChannelMux_PortHandles.Outport,'Position');
add_line([modelname,'/Channel'], [xy_ChannelMux_Outport; xy_ChannelProduct_Inport1]);

xy_ChannelDemux=[xy_ChannelProduct_Outport(1)+150, xy_ChannelProduct_Outport(2)-Size_ChannelDemux(2)/2];
add_block('simulink/Signal Routing/Demux',[modelname,'/Channel/ChannelDemux'],'Outputs',num2str(N_BLE),'Position',[xy_ChannelDemux, xy_ChannelDemux+Size_ChannelDemux]);
ChannelDemux_PortHandles=get_param([modelname,'/Channel/ChannelDemux'],'PortHandles');
add_line([modelname,'/Channel'], [xy_ChannelProduct_Outport; xy_ChannelProduct_Outport(1)+100,xy_ChannelProduct_Outport(2)]);


% place the master devices in the model
for k=1:size(xy_BLE_master,1)
    xy_BLE=xy_BLE_master_hat(k,:);
    add_block('BLE_Library/BLE master',[modelname,'/BLE master',num2str(k)],'Position',[xy_BLE,xy_BLE+Size_BLE],'LinkStatus','none');
    
    BLE_PortHandles=get_param([modelname,'/BLE master',num2str(k)],'PortHandles');
    xy_BLE_Inport1=get_param(BLE_PortHandles.Inport(1),'Position');
    xy_BLE_Outport1=get_param(BLE_PortHandles.Outport(1),'Position');
    xy_BLE_Outport2=get_param(BLE_PortHandles.Outport(2),'Position');
    
    xy_FromRx=[xy_BLE_Inport1(1)-20-Size_GotoTx(1), xy_BLE_Inport1(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/From',[modelname,'/Rx_signal_M',num2str(k)],'ShowName','off','GotoTag',['Rx_M',num2str(k)]);
    set_param([modelname,'/Rx_signal_M',num2str(k)],'position',[xy_FromRx, xy_FromRx+Size_GotoTx]);
    add_line(modelname, [xy_FromRx(1)+Size_GotoTx(1),xy_BLE_Inport1(2); xy_BLE_Inport1]);
    
    xy_GotoTx=[xy_BLE_Outport1(1)+20, xy_BLE_Outport1(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Tx_signal_M',num2str(k)],'ShowName','off','GotoTag',['Tx_M',num2str(k)]);
    set_param([modelname,'/Tx_signal_M',num2str(k)],'position',[xy_GotoTx, xy_GotoTx+Size_GotoTx]);
    add_line(modelname, [xy_BLE_Outport1; xy_GotoTx(1),xy_BLE_Outport1(2)]);

    xy_GotoDiagn=[xy_BLE_Outport2(1)+20, xy_BLE_Outport2(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Diagnostics_M',num2str(k)],'ShowName','off','GotoTag',['D_M',num2str(k)]);
    set_param([modelname,'/Diagnostics_M',num2str(k)],'position',[xy_GotoDiagn, xy_GotoDiagn+Size_GotoTx]);
    add_line(modelname, [xy_BLE_Outport2; xy_GotoDiagn(1),xy_BLE_Outport2(2)]);
    
    xy_ChannelMux_Inport=get_param(ChannelMux_PortHandles.Inport(k),'Position');
    
    xy_In=[xy_ChannelMux_Inport(1)-150-Size_In(1), xy_ChannelMux_Inport(2)-Size_In(2)/2];
    add_block('simulink/Sources/In1',[modelname,'/Channel/Tx_M',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/Tx_M',num2str(k)],'position',[xy_In, xy_In+Size_In]);
    
    xy_ChannelDemux_Outport=get_param(ChannelDemux_PortHandles.Outport(k),'Position');
    
    xy_Out=[xy_ChannelDemux_Outport(1)+200, xy_ChannelDemux_Outport(2)-Size_Out(2)/2];
    add_block('simulink/Sinks/Out1',[modelname,'/Channel/Rx_M',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/Rx_M',num2str(k)],'position',[xy_Out, xy_Out+Size_Out]);
    
    xy_Selector=[xy_ChannelDemux_Outport(1), xy_ChannelDemux_Outport(2)-Size_Selector(2)/2];
    add_block('simulink/Signal Routing/Selector',[modelname,'/Channel/Selector_M',num2str(k)],'ShowName','off','NumberOfDimensions','2','IndexOptions','Select all,Index vector (dialog)','Indices',['[],',num2str(k)]);
    set_param([modelname,'/Channel/Selector_M',num2str(k)],'position',[xy_Selector, xy_Selector+Size_Selector]);

    xy_AWGN=[xy_ChannelDemux_Outport(1)+100, xy_ChannelDemux_Outport(2)-Size_AWGN(2)/2];
    add_block('commchan3/AWGN Channel',[modelname,'/Channel/AWGN_M',num2str(k)],'ShowName','on','seed',num2str(k),'noiseMode','Signal to noise ratio  (SNR)','SNRdB','-Channel_SNR','Ps','1e-3');
    set_param([modelname,'/Channel/AWGN_M',num2str(k)],'position',[xy_AWGN, xy_AWGN+Size_AWGN]);
    %set_param([modelname,'/Channel/AWGN_M',num2str(k)],'MaskEnables',{'on';'on';'on';'off';'off';'on';'off';'off';'off';'off'});
    
    xy_Gain=[xy_ChannelMux_Inport(1)-50-Size_Gain(1), xy_ChannelMux_Inport(2)-Size_Gain(2)/2];
    add_block('simulink/Math Operations/Gain',[modelname,'/Channel/TxGain_M',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/TxGain_M',num2str(k)],'position',[xy_Gain, xy_Gain+Size_Gain]);
    set_param([modelname,'/Channel/TxGain_M',num2str(k)],'Gain','sqrt(10^(Tx_power/10)*1e-3)');
    
    add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-150,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport(1)-50-Size_Gain(1),xy_ChannelMux_Inport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-50,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport]);
    add_line([modelname,'/Channel'], [xy_ChannelProduct_Outport(1)+100,xy_ChannelProduct_Outport(2); xy_ChannelProduct_Outport(1)+100,xy_ChannelDemux_Outport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelProduct_Outport(1)+100,xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport]);
    add_line([modelname,'/Channel'], [xy_ChannelDemux_Outport(1)+Size_Selector(1),xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport(1)+100,xy_ChannelDemux_Outport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelDemux_Outport(1)+100+Size_AWGN(1),xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport(1)+200,xy_ChannelDemux_Outport(2)]);
end


% place the slave devices in the model
for k=1:size(xy_BLE_slave,1)
    xy_BLE=xy_BLE_slave_hat(k,:);
    add_block('BLE_Library/BLE slave',[modelname,'/BLE slave',num2str(k)],'Position',[xy_BLE,xy_BLE+Size_BLE],'LinkStatus','none');
    
    BLE_PortHandles=get_param([modelname,'/BLE slave',num2str(k)],'PortHandles');
    xy_BLE_Inport1=get_param(BLE_PortHandles.Inport(1),'Position');
    xy_BLE_Inport2=get_param(BLE_PortHandles.Inport(2),'Position');
    xy_BLE_Outport1=get_param(BLE_PortHandles.Outport(1),'Position');
    xy_BLE_Outport2=get_param(BLE_PortHandles.Outport(2),'Position');
    
    xy_VectConcat=[xy_BLE_Inport1(1)-15-Size_VectConcat(1), xy_BLE_Inport1(2)-Size_VectConcat(2)/2];
    add_block('simulink/Signal Routing/Vector Concatenate',[modelname,'/VectConcat_S',num2str(k)],'ShowName','off');
    set_param([modelname,'/VectConcat_S',num2str(k)],'position',[xy_VectConcat, xy_VectConcat+Size_VectConcat]);
    add_line(modelname, [xy_VectConcat(1)+Size_VectConcat(1),xy_BLE_Inport1(2); xy_BLE_Inport1]);
    
    VectConcat_PortHandles=get_param([modelname,'/VectConcat_S',num2str(k)],'PortHandles');
    xy_VectConcat_Inport1=get_param(VectConcat_PortHandles.Inport(1),'Position');
    xy_VectConcat_Inport2=get_param(VectConcat_PortHandles.Inport(2),'Position');
    
    xy_Sensor1=[xy_VectConcat_Inport1(1)-14-Size_Sensor(1), xy_VectConcat_Inport1(2)-Size_Sensor(2)/2];
    add_block('BLE_Library/Sensor',[modelname,'/Sensor_S',num2str(k),'1'],'ShowName','off');
    set_param([modelname,'/Sensor_S',num2str(k),'1'],'position',[xy_Sensor1, xy_Sensor1+Size_Sensor]);
    add_line(modelname, [xy_Sensor1(1)+Size_Sensor(1),xy_VectConcat_Inport1(2); xy_VectConcat_Inport1]);
    
    xy_Sensor2=[xy_VectConcat_Inport2(1)-14-Size_Sensor(1), xy_VectConcat_Inport2(2)-Size_Sensor(2)/2];
    add_block('BLE_Library/Sensor',[modelname,'/Sensor_S',num2str(k),'2'],'ShowName','off');
    set_param([modelname,'/Sensor_S',num2str(k),'2'],'position',[xy_Sensor2, xy_Sensor2+Size_Sensor]);
    add_line(modelname, [xy_Sensor2(1)+Size_Sensor(1),xy_VectConcat_Inport2(2); xy_VectConcat_Inport2]);
    
    xy_FromRx=[xy_BLE_Inport2(1)-30-Size_GotoTx(1), xy_BLE_Inport2(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/From',[modelname,'/Rx_signal_S',num2str(k)],'ShowName','off','GotoTag',['Rx_S',num2str(k)]);
    set_param([modelname,'/Rx_signal_S',num2str(k)],'position',[xy_FromRx, xy_FromRx+Size_GotoTx]);
    add_line(modelname, [xy_FromRx(1)+Size_GotoTx(1),xy_BLE_Inport2(2); xy_BLE_Inport2]);
    
    xy_GotoTx=[xy_BLE_Outport1(1)+20, xy_BLE_Outport1(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Tx_signal_S',num2str(k)],'ShowName','off','GotoTag',['Tx_S',num2str(k)]);
    set_param([modelname,'/Tx_signal_S',num2str(k)],'position',[xy_GotoTx, xy_GotoTx+Size_GotoTx]);
    add_line(modelname, [xy_BLE_Outport1; xy_GotoTx(1),xy_BLE_Outport1(2)]);
    
    xy_GotoDiagn=[xy_BLE_Outport2(1)+20, xy_BLE_Outport2(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Diagnostics_S',num2str(k)],'ShowName','off','GotoTag',['D_S',num2str(k)]);
    set_param([modelname,'/Diagnostics_S',num2str(k)],'position',[xy_GotoDiagn, xy_GotoDiagn+Size_GotoTx]);
    add_line(modelname, [xy_BLE_Outport2; xy_GotoDiagn(1),xy_BLE_Outport2(2)]);
    
    xy_ChannelMux_Inport=get_param(ChannelMux_PortHandles.Inport(k+N_BLE_M),'Position');
    
    xy_In=[xy_ChannelMux_Inport(1)-150-Size_In(1), xy_ChannelMux_Inport(2)-Size_In(2)/2];
    add_block('simulink/Sources/In1',[modelname,'/Channel/Tx_S',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/Tx_S',num2str(k)],'position',[xy_In, xy_In+Size_In]);
    
    xy_ChannelDemux_Outport=get_param(ChannelDemux_PortHandles.Outport(k+N_BLE_M),'Position');
    
    xy_Out=[xy_ChannelDemux_Outport(1)+200, xy_ChannelDemux_Outport(2)-Size_Out(2)/2];
    add_block('simulink/Sinks/Out1',[modelname,'/Channel/Rx_S',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/Rx_S',num2str(k)],'position',[xy_Out, xy_Out+Size_Out]);
    
    xy_Selector=[xy_ChannelDemux_Outport(1), xy_ChannelDemux_Outport(2)-Size_Selector(2)/2];
    add_block('simulink/Signal Routing/Selector',[modelname,'/Channel/Selector_S',num2str(k)],'ShowName','off','NumberOfDimensions','2','IndexOptions','Select all,Index vector (dialog)','Indices',['[],',num2str(k+N_BLE_M)]);
    set_param([modelname,'/Channel/Selector_S',num2str(k)],'position',[xy_Selector, xy_Selector+Size_Selector]);
    
    xy_AWGN=[xy_ChannelDemux_Outport(1)+100, xy_ChannelDemux_Outport(2)-Size_AWGN(2)/2];
    add_block('commchan3/AWGN Channel',[modelname,'/Channel/AWGN_S',num2str(k)],'ShowName','on','seed',num2str(k+N_BLE_M),'noiseMode','Signal to noise ratio  (SNR)','SNRdB','-Channel_SNR','Ps','1e-3');
    set_param([modelname,'/Channel/AWGN_S',num2str(k)],'position',[xy_AWGN, xy_AWGN+Size_AWGN]);
    %set_param([modelname,'/Channel/AWGN_S',num2str(k)],'MaskEnables',{'on';'on';'on';'off';'off';'on';'off';'off';'off';'off'});
    
    xy_Gain=[xy_ChannelMux_Inport(1)-50-Size_Gain(1), xy_ChannelMux_Inport(2)-Size_Gain(2)/2];
    add_block('simulink/Math Operations/Gain',[modelname,'/Channel/TxGain_S',num2str(k)],'ShowName','on');
    set_param([modelname,'/Channel/TxGain_S',num2str(k)],'position',[xy_Gain, xy_Gain+Size_Gain]);
    set_param([modelname,'/Channel/TxGain_S',num2str(k)],'Gain','sqrt(10^(Tx_power/10)*1e-3)');
    
    add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-150,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport(1)-50-Size_Gain(1),xy_ChannelMux_Inport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-50,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport]);
    add_line([modelname,'/Channel'], [xy_ChannelProduct_Outport(1)+100,xy_ChannelProduct_Outport(2); xy_ChannelProduct_Outport(1)+100,xy_ChannelDemux_Outport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelProduct_Outport(1)+100,xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport]);
    add_line([modelname,'/Channel'], [xy_ChannelDemux_Outport(1)+Size_Selector(1),xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport(1)+100,xy_ChannelDemux_Outport(2)]);
    add_line([modelname,'/Channel'], [xy_ChannelDemux_Outport(1)+100+Size_AWGN(1),xy_ChannelDemux_Outport(2); xy_ChannelDemux_Outport(1)+200,xy_ChannelDemux_Outport(2)]);
end

delete_block([modelname,'/Channel/ChannelDemux']);

xy_ChannelMux_Inport=get_param(ChannelMux_PortHandles.Inport(end),'Position');
xy_In=[xy_ChannelMux_Inport(1)-150-Size_In(1), xy_ChannelMux_Inport(2)-Size_In(2)/2];
add_block('simulink/Sources/In1',[modelname,'/Channel/Tx_I'],'ShowName','on');
set_param([modelname,'/Channel/Tx_I'],'position',[xy_In, xy_In+Size_In]);

xy_Gain=[xy_ChannelMux_Inport(1)-50-Size_Gain(1), xy_ChannelMux_Inport(2)-Size_Gain(2)/2];
add_block('simulink/Math Operations/Gain',[modelname,'/Channel/TxGain_I'],'ShowName','on');
set_param([modelname,'/Channel/TxGain_I'],'position',[xy_Gain, xy_Gain+Size_Gain],'Gain','sqrt(10^(Interf_power/10)*1e-3)');

add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-150,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport(1)-50-Size_Gain(1),xy_ChannelMux_Inport(2)]);
add_line([modelname,'/Channel'], [xy_ChannelMux_Inport(1)-50,xy_ChannelMux_Inport(2);xy_ChannelMux_Inport]);


% place From and Goto at the channel inputs and outputs
Channel_PortHandles=get_param([modelname,'/Channel'],'PortHandles');
for k=1:size(xy_BLE_master,1)
    xy_Channel_Inport=get_param(Channel_PortHandles.Inport(k),'Position');
    xy_Channel_Outport=get_param(Channel_PortHandles.Outport(k),'Position');
    
    xy_FromTx=[xy_Channel_Inport(1)-20-Size_GotoTx(1), xy_Channel_Inport(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/From',[modelname,'/Tx_signal_M_ch',num2str(k)],'ShowName','off','GotoTag',['Tx_M',num2str(k)]);
    set_param([modelname,'/Tx_signal_M_ch',num2str(k)],'position',[xy_FromTx, xy_FromTx+Size_GotoTx]);
    add_line(modelname, [xy_FromTx(1)+Size_GotoTx(1),xy_Channel_Inport(2); xy_Channel_Inport]);
    
    xy_GotoRx=[xy_Channel_Outport(1)+20, xy_Channel_Outport(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Rx_signal_M_ch',num2str(k)],'ShowName','off','GotoTag',['Rx_M',num2str(k)]);
    set_param([modelname,'/Rx_signal_M_ch',num2str(k)],'position',[xy_GotoRx, xy_GotoRx+Size_GotoTx]);
    add_line(modelname, [xy_Channel_Outport; xy_GotoRx(1),xy_Channel_Outport(2)]);
end

for k=1:size(xy_BLE_slave,1)
    xy_Channel_Inport=get_param(Channel_PortHandles.Inport(k+N_BLE_M),'Position');
    xy_Channel_Outport=get_param(Channel_PortHandles.Outport(k+N_BLE_M),'Position');
    
    xy_FromTx=[xy_Channel_Inport(1)-20-Size_GotoTx(1), xy_Channel_Inport(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/From',[modelname,'/Tx_signal_S_ch',num2str(k)],'ShowName','off','GotoTag',['Tx_S',num2str(k)]);
    set_param([modelname,'/Tx_signal_S_ch',num2str(k)],'position',[xy_FromTx, xy_FromTx+Size_GotoTx]);
    add_line(modelname, [xy_FromTx(1)+Size_GotoTx(1),xy_Channel_Inport(2); xy_Channel_Inport]);
    
    xy_GotoRx=[xy_Channel_Outport(1)+20, xy_Channel_Outport(2)-Size_GotoTx(2)/2];
    add_block('simulink/Signal Routing/Goto',[modelname,'/Rx_signal_S_ch',num2str(k)],'ShowName','off','GotoTag',['Rx_S',num2str(k)]);
    set_param([modelname,'/Rx_signal_S_ch',num2str(k)],'position',[xy_GotoRx, xy_GotoRx+Size_GotoTx]);
    add_line(modelname, [xy_Channel_Outport; xy_GotoRx(1),xy_Channel_Outport(2)]);
end

xy_Channel_Inport=get_param(Channel_PortHandles.Inport(end),'Position');
xy_FromTx=[xy_Channel_Inport(1)-20-Size_GotoTx(1), xy_Channel_Inport(2)-Size_GotoTx(2)/2];
add_block('simulink/Signal Routing/From',[modelname,'/Tx_signal_I_ch'],'ShowName','off','GotoTag','Tx_I');
set_param([modelname,'/Tx_signal_I_ch'],'position',[xy_FromTx, xy_FromTx+Size_GotoTx]);
add_line(modelname, [xy_FromTx(1)+Size_GotoTx(1),xy_Channel_Inport(2); xy_Channel_Inport]);


% place the Interferer block in the model
add_block('BLE_Library/802.11b interferer',[modelname,'/802.11b interferer'],'Position',[xy_Interf_hat, xy_Interf_hat+Size_Interf],'LinkStatus','none');
Interferer_PortHandles=get_param([modelname,'/802.11b interferer'],'PortHandles');
xy_Interferer_Outport1=get_param(Interferer_PortHandles.Outport(1),'Position');
xy_Interferer_Outport2=get_param(Interferer_PortHandles.Outport(2),'Position');

xy_GotoTx=[xy_Interferer_Outport1(1)+20, xy_Interferer_Outport1(2)-Size_GotoTx(2)/2];
add_block('simulink/Signal Routing/Goto',[modelname,'/Tx_signal_I'],'ShowName','off','GotoTag','Tx_I');
set_param([modelname,'/Tx_signal_I'],'position',[xy_GotoTx, xy_GotoTx+Size_GotoTx]);
add_line(modelname, [xy_Interferer_Outport1; xy_GotoTx(1),xy_Interferer_Outport1(2)]);

xy_GotoTx=[xy_Interferer_Outport2(1)+20, xy_Interferer_Outport2(2)-Size_GotoTx(2)/2];
add_block('simulink/Signal Routing/Goto',[modelname,'/Diagnostics_I'],'ShowName','off','GotoTag','D_I');
set_param([modelname,'/Diagnostics_I'],'position',[xy_GotoTx, xy_GotoTx+Size_GotoTx]);
add_line(modelname, [xy_Interferer_Outport2; xy_GotoTx(1),xy_Interferer_Outport2(2)]);


% place the "Export parameters" block in the model
xy_GenParam=xy_Channel-[20+Size_GotoTx(1)+50+Size_Diagnostics(1)+350,0];
add_block('BLE_Library/Export parameters',[modelname,'/Export parameters'],'Position',[xy_GenParam, xy_GenParam+Size_GenParam],'LinkStatus','none');


% place the Diagnostics block in the model
xy_Diagnostics=xy_Channel-[20+Size_GotoTx(1)+Size_Diagnostics(1)+160,0];
add_block('BLE_Library/Diagnostics',[modelname,'/Diagnostics'],'Position',[xy_Diagnostics, xy_Diagnostics+Size_Diagnostics],'Orientation','down','LinkStatus','none');
Diagnostics_PortHandles=get_param([modelname,'/Diagnostics'],'PortHandles');
xy_Diagnostics_Inport1=get_param(Diagnostics_PortHandles.Inport(1),'Position');
xy_Diagnostics_Inport2=get_param(Diagnostics_PortHandles.Inport(2),'Position');
xy_Diagnostics_Inport3=get_param(Diagnostics_PortHandles.Inport(3),'Position');
xy_Diagnostics_Outport1=get_param(Diagnostics_PortHandles.Outport(1),'Position');
xy_Diagnostics_Outport2=get_param(Diagnostics_PortHandles.Outport(2),'Position');
xy_Diagnostics_Outport3=get_param(Diagnostics_PortHandles.Outport(3),'Position');
xy_Diagnostics_Outport4=get_param(Diagnostics_PortHandles.Outport(4),'Position');

xy_FromDiagn=[xy_Diagnostics_Inport1(1)-Size_GotoDiagn(1)/2, xy_Diagnostics_Inport1(2)-20-Size_GotoDiagn(2)];
add_block('simulink/Signal Routing/From',[modelname,'/Diagnostics_S1_'],'position',[xy_FromDiagn, xy_FromDiagn+Size_GotoDiagn],'Orientation','down','ShowName','off','GotoTag','D_S1');
add_line(modelname, [xy_Diagnostics_Inport1(1),xy_FromDiagn(2)+Size_GotoDiagn(2); xy_Diagnostics_Inport1]);
 
xy_FromDiagn=[xy_Diagnostics_Inport2(1)-Size_GotoDiagn(1)/2, xy_Diagnostics_Inport2(2)-20-Size_GotoDiagn(2)];
add_block('simulink/Signal Routing/From',[modelname,'/Diagnostics_M1_'],'position',[xy_FromDiagn, xy_FromDiagn+Size_GotoDiagn],'Orientation','down','ShowName','off','GotoTag','D_M1');
add_line(modelname, [xy_Diagnostics_Inport2(1),xy_FromDiagn(2)+Size_GotoDiagn(2); xy_Diagnostics_Inport2]);
 
xy_FromDiagn=[xy_Diagnostics_Inport3(1)-Size_GotoDiagn(1)/2, xy_Diagnostics_Inport3(2)-20-Size_GotoDiagn(2)];
add_block('simulink/Signal Routing/From',[modelname,'/Diagnostics_I_'],'position',[xy_FromDiagn, xy_FromDiagn+Size_GotoDiagn],'Orientation','down','ShowName','off','GotoTag','D_I');
add_line(modelname, [xy_Diagnostics_Inport3(1),xy_FromDiagn(2)+Size_GotoDiagn(2); xy_Diagnostics_Inport3]);

xy_DDisplay=[xy_Diagnostics_Outport1(1)-Size_DDisplay(1)/2, xy_Diagnostics_Outport1(2)+20];
add_block('simulink/Sinks/Display',[modelname,'/Display Battery status'],'ShowName','off','Position',[xy_DDisplay, xy_DDisplay+Size_DDisplay],'Orientation','down');
add_line(modelname, [xy_Diagnostics_Outport1; xy_Diagnostics_Outport1(1), xy_Diagnostics_Outport1(2)+20]);
 
xy_DDisplay=[xy_Diagnostics_Outport2(1)-Size_DDisplay(1)/2, xy_Diagnostics_Outport2(2)+20];
add_block('simulink/Sinks/Display',[modelname,'/Display BER'],'ShowName','off','Position',[xy_DDisplay, xy_DDisplay+Size_DDisplay],'Orientation','down');
add_line(modelname, [xy_Diagnostics_Outport2; xy_Diagnostics_Outport2(1), xy_Diagnostics_Outport2(2)+20]);

xy_DDisplay=[xy_Diagnostics_Outport3(1)-Size_DDisplay(1)/2, xy_Diagnostics_Outport3(2)+20];
add_block('simulink/Sinks/Display',[modelname,'/Display PER'],'ShowName','off','Position',[xy_DDisplay, xy_DDisplay+Size_DDisplay],'Orientation','down');
add_line(modelname, [xy_Diagnostics_Outport3; xy_Diagnostics_Outport3(1), xy_Diagnostics_Outport3(2)+20]);

xy_DDisplay=[xy_Diagnostics_Outport4(1)-Size_DDisplay(1)/2, xy_Diagnostics_Outport4(2)+20];
add_block('simulink/Sinks/Display',[modelname,'/Display PRR'],'ShowName','off','Position',[xy_DDisplay, xy_DDisplay+Size_DDisplay],'Orientation','down');
add_line(modelname, [xy_Diagnostics_Outport4; xy_Diagnostics_Outport4(1), xy_Diagnostics_Outport4(2)+20]);

save_system(modelname);

run('BLE_PreLoad') 