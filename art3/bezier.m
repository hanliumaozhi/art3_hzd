function B = bezier(t, P)

    B = 0.0;
    for i = 0:5
        B = B + P(1, i+1)*factorial(5)/(factorial(i)*factorial(5-i))*((t^i)*(1-t)^(5-i));
    end
end