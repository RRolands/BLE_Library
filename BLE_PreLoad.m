assignin('base', 'T_IFS', 150); %Inter Frame Space (us)

assignin('base', 'ADV_IND_id', 10); %packet type ID
assignin('base', 'CONNECT_IND_id', 30);
assignin('base', 'LL_DATA_id', 40);

assignin('base', 'CONNECT_IND_length', 352); %packet length
assignin('base', 'LL_DATA_EMPTY_length', 80); 

% Access Address for advertising channel packets
assignin('base', 'ADV_IND_AccA', boolean(reshape(dec2bin(hex2dec('8E89BED6')).',1,[])'-'0'));

assignin('base', 'Slot_Ts', 1e-6); %Sample rate

% GMSK Modulator and hopping variables
assignin('base', 'BT_Hop_Separation', 2e6); 
assignin('base', 'BT_Freq_Deviation', 2*250e3);
assignin('base', 'Num_Hop_Frequencies', 40);
assignin('base', 'BT_Samples_Per_Symbol', 88);