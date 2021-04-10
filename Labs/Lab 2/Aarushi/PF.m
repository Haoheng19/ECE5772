function [X_t, tier5, big1] = PF(X_0, u, z, R, Q, measurement, prediction)

    X_t_bar = [];
    %lets move each particle
    for i=1:length(X_0)
        X_t_bar = [X_t_bar; prediction(X_0(i,:)', u)' + (randn(length(R))*diag(R))'];
    end
    weights = ones([length(X_0),1]);
    for i=1:size(X_t_bar, 1)
        expected = measurement(X_t_bar(i, :)'); 
        new_e = [];
        new_z = [];
        for j=1:size(expected, 1)
            p = X_t_bar(i,:);
            if (expected(j) <= 10) && ~(...
                    (p(1) > 1.54) || (p(1) < -1.54) || (p(2) > 3) || (p(2) < -3) )
                new_e = [new_e; expected(j)];
                new_z = [new_z; z(j)];
            end
        end
        if isempty(new_e)
            weights(i) = 0;
        else
            new_Q = Q(1:length(new_z),1:length(new_z));
            weights(i) = mvnpdf(new_e, new_z, new_Q*5);
        end
    end
    if sum(weights) == 0
        weights = ones([length(X_0),1]);
    end
    X_t = datasample(X_t_bar,size(X_t_bar, 1),'Weights',weights);
    [~,I] = maxk(weights, 6);
    [~,I1] = maxk(weights, 1);
    tier5 = [];
    big1 = X_t(I1(1), :);
    for i=1:length(I)
        tier5 = [tier5; X_t(I(i), :)];
    end
end