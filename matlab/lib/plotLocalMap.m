function [] = plotLocalMap(local_map)
    s = scatter(local_map(:,2),local_map(:,3));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('ID',local_map(:,1));
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Type',local_map(:,4));
end
