function x_hat = mmse(x, pdf_x_z)


x_hat = trapz(x, pdf_x_z.*x);

return