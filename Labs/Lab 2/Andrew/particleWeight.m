function weight = particleWeight(x, z, map, tol)
    depth = depthPredict(x,map,[0.13 0],linspace(deg2rad(27),deg2rad(-27),length(z)));
    weight = 1;
    max_x = max(max(map(:,[1 3])));
    min_x = min(min(map(:,[1 3])));
    max_y = max(max(map(:,[2 4])));
    min_y = min(min(map(:,[2 4])));
    if (x(1)>=max_x || x(1)<= min_x || x(2) >= max_y || x(2) <= min_y)
        weight = 0;
    else
        for i = 1:length(z)
            weight = weight * normpdf(z(i),depth(i),tol);
        end
    end
end