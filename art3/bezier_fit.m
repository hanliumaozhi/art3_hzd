function y = bezier_fit(gait_x, a, b, c, d, e, f)

    y = zeros(size(gait_x));
    P = [a b c d e f];
    for i = 0:5
        y = y + (P(1, i+1)*factorial(5)/(factorial(i)*factorial(5-i))).*((gait_x.^i).*(ones(size(gait_x))-gait_x).^(5-i));
    end
end