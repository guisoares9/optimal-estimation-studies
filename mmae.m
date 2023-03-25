function x_hat = mmae(x, pdf_x_z)

dx = (x(end) - x(1))./length(x);
sum = 0;
for i = 1:length(x)
    sum = sum + pdf_x_z(i)*dx;
    if sum > 1/2
        x_hat = x(i);
        break
    end
end

return