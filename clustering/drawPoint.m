function drawPoint(data, mkstyle, mksize, redraw = true)
  if redraw == false
    hold on;
  endif
  if size(data)(2) == 2
    plot(data(:, 1), data(:, 2), mkstyle, 'MarkerSize', mksize);
  elseif size(data)(2) == 3
    plot3(data(:, 1), data(:, 2), data(:, 3), mkstyle, 'MarkerSize', mksize);
  endif
  if redraw == false
    hold off;
  endif
endfunction