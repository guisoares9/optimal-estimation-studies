function pdf = px(x)

beta = 20;
x_min = 1;
x_max = 3;

u = (x_max + x_min)./2;
alpha = (x_max - x_min)./2;

pdf = (beta*exp(-1*(abs(x-u)/alpha).^beta))/(2*alpha*gamma(1./beta));

return



