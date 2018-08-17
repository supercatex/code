% 清空畫面
clear ; close all; clc

X = [];                       % 樣本數據
n = floor(rand()*9) + 2;      % 群集數目 2 ~ 10
m = floor(rand(n)*450) + 50;  % 群集數據量 50 ~ 499
r = floor(rand(n)*15) + 5;    % 數據集中度 5 ~ 19
R = floor(rand()*50) + 50;    % 數據邊界 50 ~ 99
d = 3;                        % 維度 3
marker_style = 'o';
marker_size = 5;

% 隨機生生群集樣本數據
for i = 1:n
  X(end + 1:end + m(i), :) = rand(1, d) * (R - r(i)) + rand(m(i), d) * r(i);
  y(end + 1:end + m(i)) = i;
end

% 顯示原樣本數據
figure(1, 'position', [0, 550, 600, 450]);
drawPoint(X(1,:), marker_style, marker_size);
for i = 1:n
  indices = find(y == i);
  drawPoint(X(indices, :), marker_style, marker_size, false);
end
pause(1);

% 顯示無差別樣本數據
figure(2, 'position', [600, 550, 600, 450]);
drawPoint(X, marker_style, marker_size);

Landmark = [];
k = 1;
grad = [];
while true
  printf("clustering (k=%d)...\n", k);
  
  % 生成標記點
  if k == 1
    % 只有一個群集時為全部樣本重心位置
    Landmark(k, :) = mean(X);
    [Landmark, error] = optimizer(X, Landmark, k);
    grad = [grad; [k, sum(error)]];
    k += 1;
    pause(0.1);
    continue;
  endif
  
  % 在最大的群集旁邊加一個新群集
  me = inf;
  mi = ones(1, d);
  for i = 1:50
    Landmark = [Landmark; rand(1, d) * R];
    [Landmark, error] = optimizer(X, Landmark, k, R, 0);

    %printf('k=%.0f, i=%.0f, error=%.2f\n', k, i, sum(error)(2));
    if sum(error)(2) < me
      me = sum(error)(2);
      mi = Landmark(end, :);
    endif
 
    Landmark(k, :) = [];
    [Landmark, error] = optimizer(X, Landmark, k - 1, R, 0);
  end
  Landmark = [Landmark; mi];
  [Landmark, error] = optimizer(X, Landmark, k, R, 0);
  grad = [grad; [k, sum(error)(2)]];
  
  if k > 1
    e1 = grad(k, 2) / grad(1, 2);
    e2 = grad(k - 1, 2) / grad(1, 2);
    e = abs(e2 - e1);
    
    printf('total error: ek = %.4f, ek-1 = %.4f\n', e1, e2);
    e
    
    if e < 0.001
      printf('error no more changed. %.4f\n', e);
      break;
    endif
  endif
  
  k += 1;
  %pause(0.1);
end

figure(4, 'position', [600, 50, 600, 450]);
drawPoint(grad, '-', 5);
printf('n: %d, k: %d\n\n', n, k);

% 由尾向前找波幅的轉折點
i = k - 1;
while i >= 1
  e1 = grad(i + 1, 2);
  e2 = grad(i, 2);
  e = abs(e1 / e2)
  if e < 0.8
    break;
  endif
  i -= 1;
end

k_star = i + 1
while k_star > 0 && k_star < k - 1
  Landmark(k, :) = [];
  [Landmark, error] = optimizer(X, Landmark, k - 1, R, 0);
  k -= 1;
end
Landmark(k, :) = [];
[Landmark, error] = optimizer(X, Landmark, k - 1, R, 0.1);
printf('END');