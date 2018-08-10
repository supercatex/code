% 清空畫面
clear ; close all; clc

styles = {'b+'; 'c+'; 'm+'; 'y+'; 'g+'; 'k+'; 'r+'; 'bs'; 'cs'; 'gs'};

% 隨機生生群集樣本數據
X = [];   % 樣本數據
m = 100;  % 每群集數據量
n = 5;   % 群集數目
r = 3;    % 數據邊界
for i = 1:n
  X(m*(i-1)+1:m*i,:) = rand(1, 2) * (r-1) + rand(m, 2);
  y(m*(i-1)+1:m*i) = i;
end

% 顯示原樣本數據
figure(1);
for i = 1:n
  index = find(y == i);
  hold on;
  plot(X(index,1), X(index,2), styles{i}, 'MarkerSize', 5);
  hold off;
end
printf("samples...(enter to next)...\n");
pause;

figure(2);
plot(X(:,1), X(:,2), styles{1}, 'MarkerSize', 5);
printf("for reference...(enter to next)...\n");
pause;

for k = 1:size(styles)
  printf("clustering (k=%d)...\n", k);
  
  % 隨機生成標記點
  Landmark = rand(k, 2) * r;
  figure(3);
  hold on;
  plot(Landmark(:,1), Landmark(:,2), 'ko', 'MarkerSize', 10);
  hold off;

  prev_error = 0;
  curr_error = 1;
  while (curr_error - prev_error) .^ 2 >= 0.1
    % 計算樣本數據到標記點誤差, 及所屬群集
    for i = 1:k
      ERROR(:, i) = sum((X - Landmark(i,:)) .^ 2, 2);
    end
    prev_error = curr_error;
    curr_error = sum(sum(ERROR, 1), 2);
    [v, y] = min(ERROR, [], 2);

    % 顯示分類結果
    figure(3);
    plot(Landmark(:,1), Landmark(:,2), 'r.', 'MarkerSize', 30);

    for i = 1:k
      index = find(y == i);
      hold on;
      plot(X(index,1), X(index,2), styles{i}, 'MarkerSize', 5);
      hold off;
    end
    
    % 更新標記點
    for i = 1:k
      index = find(y == i);
      if size(index) > 0
        Landmark(i,:) = mean(X(index,:));
      else
        Landmark(i,:) = rand(1, 2) * r;
      endif
    end
    
    pause(0.1);
  end
  printf("next...");
  pause;
end
printf("END");