function x_hat = ml(x, pdf_z_x)

[~ ,index] = max(pdf_z_x);
x_hat = x(index);

return