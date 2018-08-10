function B = bezier(t, P)
    %Bazier curve
    % Parameters
    % ----------
    % - t: double
    %   Time between 0 and 1
    % - C: 2-by-n double matrix
    %   Control points
    %
    % Returns
    % -------
    % - B: 2-by-1 vector
    %   Output point

    B = 0.0;
    for i = 0:5
        B = B + P(1, i+1)*factorial(5)/(factorial(i)*factorial(5-i))*((t^i)*(1-t)^(5-i));
    end
end