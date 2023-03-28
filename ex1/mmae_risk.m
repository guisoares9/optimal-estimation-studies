function risk = mmae_risk(pdf_x_z, x, x_hat)

dif_vec = x_hat - x;
risk = trapz(x, dif_vec.*pdf_x_z);

return