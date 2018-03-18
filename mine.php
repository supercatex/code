<?php
	$_rows = 10;
	$_cols = 10;
	$_boxSize = 50;
	$_mineCount = 15;
	
	function getIdBy($class, $x, $y) {
		return $class . '_' . $x . '_' . $y;
	}
	
	function getDivBy($class, $x, $y, $size) {
		$id = getIdBy($class, $x, $y);
		$left = $x * $size . 'px';
		$top = $y * $size . 'px';
		return "<div id=\"$id\" class=\"$class\" style=\"left:$left; top:$top;\" x=\"$x\" y=\"$y\"></div>";
	}
?>
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Minesweeper</title>
<style type="text/css">
#container { width: <?= $_boxSize * $_cols ?>px; }
#main { 
	position: relative; 
	height: <?= $_boxSize * $_rows ?>px;
}
h1 { position: relative; }
h1 span {
	position: absolute;
	right: 0;
	bottom: 0;
	font-size: 16px;
}
p { float: right; }
.box {
	width: <?= $_boxSize ?>px;
	height: <?= $_boxSize ?>px;
	background-color: #CCCCCC;
	border: solid 1px #333333;
	position: absolute;
	z-index: 10;
	line-height: <?= $_boxSize ?>px;
	color: #000;
	font-size: 24px;
	text-align: center;
}
.cover {
	width: <?= $_boxSize ?>px;
	height: <?= $_boxSize ?>px;
	background-color: #336699;
	border: solid 1px #333333;
	position: absolute;
	z-index: 100;
}
.cover:hover { background-color: #4477AA; }
.num1 { color: #0000FF; }
.num2 { color: #009900; }
.num3 { color: #990000; }
.num4, .num5, .num6, .num7, .num8 { color: #FF0000; }
</style>
</head>
<body>
<div id="container">
<h1>Minesweeper<span><?= $_mineCount ?> mines</span></h1>
<div id="main">
<?php
	for ($y = 0; $y < $_rows; $y++) {
		for ($x = 0; $x < $_cols; $x++) {
			echo getDivBy('box', $x, $y, $_boxSize);
			echo getDivBy('cover', $x, $y, $_boxSize);
		}
	}
?>
</div>
<p>&copy;2018 Powered by Kinda.</p>
</div>
<script type="text/javascript">
	var _cols = <?= $_cols ?>;
	var _mineCount = <?= $_mineCount ?>;
	var _mineChar = "@";
	var _blankChar = "";
	var _isGameOver = false;
	
	function setMines() {
		var boxes = Array.from(document.getElementsByClassName("box"));
		
		for (var i = 0; i < boxes.length; i++) {
			var randomIndex = Math.floor(Math.random() * boxes.length);
			var temp = boxes[i];
			boxes[i] = boxes[randomIndex];
			boxes[randomIndex] = temp;
		}
		
		for (var i = 0; i < Math.min(_mineCount, boxes.length); i++) {
			boxes[i].innerHTML = _mineChar;
		}
	}
	
	function getIdBy(className, x, y) {
		return className + "_" + x + "_" + y;
	}
	
	function isValidRange(x, y) {
		if (x < 0 || x >= _cols || y < 0 || y >= _cols) 
			return false;
		return true;
	}
	
	function isEqual(value, x, y) {
		if (!isValidRange(x, y)) return false;
		
		var box = document.getElementById(getIdBy("box", x, y));
		return box.innerHTML == value;
	}
	function isMine(x, y) { return isEqual(_mineChar, x, y); }
	function isBlank(x, y) { return isEqual(_blankChar, x, y); }
	
	function setNumbers() {
		var boxes = document.getElementsByClassName("box");
		
		for (var i = 0; i < boxes.length; i++) {
			var x = i % _cols;
			var y = Math.floor(i / _cols);
				
			if (!isMine(x, y)) {
				count = 0;
				if (isMine(x - 1, y - 1)) count++;
				if (isMine(x, y - 1)) count++;
				if (isMine(x + 1, y - 1)) count++;
				if (isMine(x - 1, y + 1)) count++;
				if (isMine(x, y + 1)) count++;
				if (isMine(x + 1, y + 1)) count++;
				if (isMine(x - 1, y)) count++;
				if (isMine(x + 1, y)) count++;

				boxes[i].innerHTML = count > 0 ? count : "";
				boxes[i].classList.add("num" + count);
			}
		}
	}
	
	function isOpened(x, y) {
		var cover = document.getElementById(getIdBy("cover", x, y));
		return cover.style.display == "none";
	}
	
	function open(x, y) {
		if (!isValidRange(x, y) || isOpened(x, y)) return;

		var cover = document.getElementById(getIdBy("cover", x, y));
		cover.style.display = "none";
		
		if (isBlank(x, y)) {
			for (var i = -1; i <= 1; i++) {
				for (var j = -1; j <= 1; j++) {
					if (i == 0 && j == 0) continue;
					open(x + i, y + j);
				}
			}
		}
	}
	
	function isFinished() {
		var covers = document.getElementsByClassName("cover");
		var isMatch = true;
		for (var i = 0; i < covers.length; i++) {
			var x = i % _cols;
			var y = Math.floor(i / _cols);
			
			if (isOpened(x, y) == isMine(x, y)) {
				isMatch = false;
				break;
			}
		}
		return isMatch;
	}
	
	window.onload = function() {
		setMines();
		setNumbers();
		
		var covers = document.getElementsByClassName("cover");
		for (var i = 0; i < covers.length; i++) {
			covers[i].addEventListener("click", function() {
				if (_isGameOver == false) {
					var x = parseInt(this.getAttribute("x"));
					var y = parseInt(this.getAttribute("y"));
					
					open(x, y);
					
					if (isMine(x, y)) {
						_isGameOver = true;
						alert("Game Over!!!");
					} else if (isFinished()) {
						_isGameOver = true;
						setTimeout(function () { alert("Victory!!!"); }, 200);
					}
				}
			});
			
			covers[i].addEventListener("contextmenu", function(event) {
				event.preventDefault();
			});
		}
	};
</script>
</body>
</html>