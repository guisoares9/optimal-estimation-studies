function risk = map_risk(pdf_x_z)

global delta_map
[max_val, ~] = max(pdf_x_z);
risk = 1 - max_val*delta_map;

return