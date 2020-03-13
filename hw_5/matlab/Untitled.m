clc;
clear;
n_order = 7
Q_k = zeros(n_order+1, n_order+1)
for i = 1:n_order+1
    for l = 1:n_order+1
        if i >= 4
            if l >= 4
                i
                l
                a = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*1^(i+l-7)/(i+l-7)
                Q_k(i,l) = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*1^(i+l-7)/(i+l-7);
            end
        end
    end
end
Q_k