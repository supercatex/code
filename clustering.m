% 清空畫面
clear ; close all; clc

X = [];   % 樣本數據
m = 100;  % 每群集數據量
n = 5;    % 群集數目
r = 4;    % 數據邊界
d = 3;    % 維度
marker_style = 'o';
marker_size = 5;

% 隨機生生群集樣本數據
for i = 1:n
  X(m * (i - 1) + 1:m * i, :) = rand(1, d) * (r - 1) + rand(m, d);
  y(m * (i - 1) + 1:m * i) = i;
end

% 顯示原樣本數據
figure(1);
plot3(X(1, 1), X(1, 2), X(1, 3), marker_style, 'MarkerSize', marker_size);
hold on;
for i = 1:n
  indices = find(y == i);
  %plot(X(indices, 1), X(indices, 2), marker_style, 'MarkerSize', marker_size);
  plot3(X(indices, 1), X(indices, 2), X(indices, 3), marker_style, 'MarkerSize', marker_size);
end
hold off;
pause(1);

% 顯示無差別樣本數據
figure(2);
%plot(X(:, 1), X(:, 2), marker_style, 'MarkerSize', marker_size);
plot3(X(:, 1), X(:, 2), X(:, 3), marker_style, 'MarkerSize', marker_size);
pause(1);

for k = 1:n*2
  printf("clustering (k=%d)...\n", k);
  
  % 隨機生成標記點
  Landmark = rand(k, d) * r;

  prev_error = 0;
  curr_error = 1;
  while (curr_error - prev_error) .^ 2 >= 0.1
    % 計算樣本數據到標記點誤差, 及所屬群集
    for i = 1:k
      ERROR(:, i) = sum((X - Landmark(i, :)) .^ 2, 2);
    end
    prev_error = curr_error;
    curr_error = sum(sum(ERROR, 1), 2);
    [v, Y] = min(ERROR, [], 2);

    % 顯示標記點及分類結果
    figure(3);
    %plot(Landmark(:, 1), Landmark(:, 2), 'r.', 'MarkerSize', 30);
    plot3(Landmark(:, 1), Landmark(:, 2), Landmark(:, 3), 'r.', 'MarkerSize', 30);
    
    hold on;
    for i = 1:k
      indices = find(Y == i);
      %plot(X(indices, 1), X(indices, 2), marker_style, 'MarkerSize', marker_size);
      plot3(X(indices, 1), X(indices, 2), X(indices, 3), marker_style, 'MarkerSize', marker_size);
    end
    hold off;
    
    % 更新標記點
    for i = 1:k
      indices = find(Y == i);
      if size(indices) > 0
        Landmark(i, :) = mean(X(indices, :));
      else
        Landmark(i, :) = rand(1, d) * r;
      endif
    end
    
    pause(0.1);
  end
  pause(1);
end

printf("END");
