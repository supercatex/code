function [Landmark, error] = optimizer(X, Landmark, k, R, delay = 0)
  % 不斷修正標記點位置直到誤差小於1e-4
  prev_error = 0;
  curr_error = 1;
  error = [];
  for i = 1:k
    error(i, :) = [i, 0];
  end
  d = size(X)(2);
  
  while (curr_error - prev_error) .^ 2 >= 1e-2
    % 計算樣本數據到標記點誤差, 及所屬群集
    for i = 1:k
      ERROR(:, i) = sum((X - Landmark(i, :)) .^ 2, 2);
    end
    prev_error = curr_error;
    curr_error = sum(sum(ERROR, 1), 2);
    [v, Y] = min(ERROR, [], 2);

    % 顯示標記點及分類結果
    if delay > 0
      figure(3, 'position', [0, 50, 600, 450]);
      drawPoint(Landmark, 'r.', 40);
    endif
    
    for i = 1:k
      % 計算每個群集最遠的一點的距離
      indices = find(Y == i);
      dist = sum(sum((X(indices, :) - Landmark(i, :)) .^ 2));
      error(i, 2) = sum(dist) ./ size(dist)(2);
      
      if delay > 0
        drawPoint(X(indices, :), 'o', 5, false);
      endif
    end
    
    % 更新標記點
    for i = 1:k
      indices = find(Y == i);
      if size(indices) > 0
        Landmark(i, :) = mean(X(indices, :));
      else
        Landmark(i, :) = rand(1, d) * R;
      endif
    end

    if delay > 0
      pause(delay);
    endif
  end
endfunction