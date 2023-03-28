function pdf_z_x = pz_x(z, x_fixed, P0, P1, sigma)

pdf_z_x = P0*normpdf(z, x_fixed, sigma) + P1*normpdf(z, 2*x_fixed, sigma);

return