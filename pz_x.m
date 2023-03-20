function pdf_z_x = pz_x(z, mu)

P0 = 0.95;
P1 = 0.05;

sigma = 0.1;


pdf_z_x = P0*normpdf(z, mu, sigma) + P1*normpdf(z, 2*mu, sigma);

return