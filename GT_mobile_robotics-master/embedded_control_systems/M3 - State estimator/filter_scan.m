function scan_filtered = filter_scan(scan)
% Function that filters zero elements from laser scan

r = scan.Ranges;
th = scan.Angles;

% get index of non-zero elements
index = find(r);

scan_filtered.Ranges = r(index);
scan_filtered.Angles  = th(index);

end