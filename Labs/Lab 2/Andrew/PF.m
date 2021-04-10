function final_set = PF(set_prev, g, u, R, w, z, map)
    set_bar = zeros(size(set_prev));
    for m = 1:size(set_prev,1)
        current_particle = g(set_prev(m,1:end-1), u);
        for i = 1:length(current_particle)
            current_particle(i) = normrnd(current_particle(i),R(i,i));
        end
        set_bar(m,:) = [current_particle w(current_particle, z, map)];
    end
    if sum(set_bar(:,end)) == 0
        set_bar(:,end) = set_bar(:,end) + 1;
    end
    final_set = datasample(set_bar,size(set_prev,1),1,'Weights',set_bar(:,end));
end