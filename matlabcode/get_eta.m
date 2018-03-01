% get eta function
function eta = get_eta(k1, m, vmin, vmax, wmax, si_v, si_w, gamma0, gamma1, gamma2)
%2,1,0.1,20,1,5,0.5,0.5,0.5,0.5
syms alph bta si
% equation for calculating M3, M4 and M5
rw1 = (k1*bta+(alph/sqrt(alph^2+1))*si)*sin((alph - bta)/m)+ k1*gamma1*bta*sin(bta/m);
M3_2 = (alph^2 +1)*(rw1/alph^2);
rw2 = (abs(alph)/sqrt(alph^2 + 1))*(wmax - si_w) + ...
    (k1 - 0.5) * (vmin - (vmin + si_v)*cos((alph - bta)/m)) + ...
    gamma2*(k1 - 0.5)*((vmin+si_v)*cos(bta/m) - vmin);
rw3 = (abs(alph)/sqrt(alph^2 +1))*(wmax - si_w) + (k1 -0.5)*(gamma2 - 1)*si_v;
M4_2 = (alph^2 +1)*(rw2/alph^2);
M5_2 = (alph^2 +1)*(rw3/alph^2);

% limit to check the value of M3, M4 and M5
alph_sub = [-0.999 -0.001 0.001 0.999];
bta_sub = [-0.999 -0.001 0 0.001 0.999];
si_sub = [0.001 0.999 1];
alph_len = length(alph_sub);
bta_len = length(bta_sub);
si_len = length(si_sub);
sample_len = alph_len*bta_len*si_len;
M3_sub = zeros(sample_len,3);
M3_var = [alph bta si];
M3 = zeros(sample_len,1);

%finding all the value of alpha beta and si for substitution  in M3
for i = 1:alph_len
    for j = 1:bta_len
        for k = 1:si_len
            M3_sub(((i-1)*bta_len*si_len)+((j-1)*si_len)+k,:)...
            = [alph_sub(i) bta_sub(j) si_sub(k)];
        end
    end
end

%checking the value of M3 for all possible value of alpha beta and si
for i =1:sample_len
    M3(i) = double(subs(M3_2, M3_var, M3_sub(i,:)));
end
M4_var = [alph bta];
M4_sub = zeros(bta_len,2);

% finding all the value of alpha and beta for substitution  in M4
for i = 3:alph_len
    for j= 1:bta_len
        M4_sub(((i-3)*bta_len)+j,:) = [alph_sub(i) bta_sub(j)];
    end
end

%checking the value of M4 for all possible value of alpha and beta
sample_len2 = length(M4_sub);
M4 = zeros(sample_len2,1);
for i=1:sample_len2
    M4(i) = double(subs(M4_2, M4_var, M4_sub(i,:)));
end

%checking the value of M5 for all possible value of alpha
M5 = zeros(alph_len, 1);
for i = 1:alph_len
    M5(i) = subs(M5_2, alph, alph_sub(i));
end

% finding the value of M3, M4 and M5
M3_max = max(0, max(M3));
M4_max = max(0, max(M4));
M5_max = max(0, max(M5));

% calculating the value of kw and kv since m = 1 so componenet of 
% d3 and d4 are not used for calculating the value of kw and kv
d3 = M3_max*(vmax - si_v) + gamma0 + 0.5*(wmax - si_w);
d4 = M3_max*(vmax - si_v) + gamma0 + max(M4_max, M5_max);
kw = (2*wmax - si_w);
kv = ((wmax - si_w)/(2*k1 - 1)) + gamma2*si_v;

% since eta0>kw and eta1>kv so adding 0.5 in both to find eta0 and eta1
eta0 = kw+ 0.5;
eta1 = kv + 0.5;
eta = [eta0; eta1];
eta

