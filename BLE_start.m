Names=find_system(bdroot,'regexp','on','Name','Channel');

if ~isempty(Names)
    for k=1:length(Names)
        if isfield(get_param(Names{k},'DialogParameters'),'InterfOn')
            if strcmp(get_param(Names{k},'InterfOn'),'off')
                
                figHandles=findall(groot, 'Type', 'Axes');
                
                if ~isempty(figHandles)
                    for n=1:length(figHandles)
                        
                        if strcmp(figHandles(n).Title.String,'Frequencies')
                            figHandles(n).Children(1).Visible='off';
                            figHandles(n).Legend.String(end)=[];
                        end
                        
                        if strcmp(figHandles(n).Title.String,'Slave/Master signals')
                            figHandles(n).Children(1).Visible='off';
                            figHandles(n).Legend.String(end)=[];
                        end
                        
                    end
                end
            end
        end
    end
end