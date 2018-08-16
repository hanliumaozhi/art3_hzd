function B = bezier_vec(t, P, T)
    B = 0.0;
    for i = 0:4
        B = B + (P(1, i+2)-P(1, i+1))*(factorial(5)/(factorial(i)*factorial(5-i-1)))*((t^i)*(1-t)^(5-i-1));
    end
    B = B/(T);
end