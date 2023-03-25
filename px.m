function pdf_x = px(x, beta, x_max, x_min)

u = (x_max + x_min)./2;
alpha = (x_max - x_min)./2;

pdf_x = (beta*exp(-1*(abs(x-u)/alpha).^beta))/(2*alpha*gamma(1./beta));

return



