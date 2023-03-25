function x_hat = map(x, pdf_x_z)

[~ ,index] = max(pdf_x_z);
x_hat = x(index);

return