function overall_z = pz(x, pdf_x, pdf_z_x)

% calculate over all possible z given the pz_x and pz
% its more a normalize term for the bayes px_z = pz_x * px / overall_z
% overall_z = int{pz_x*px*dx}
overall_z = trapz(x, pdf_z_x.*pdf_x);

return

