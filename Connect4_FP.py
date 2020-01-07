# -*- coding: utf-8 -*-
"""
四連棋遊戲
函數式編程版本 - Functional Programming version.
Name: Kinda Lam
Date: 2020-01-08
"""
import numpy as np
import time


# 定義棋盤的狀態
_EMPTY: str = "　"              # 空白
_BLACK: str = "Ｘ"              # 黑棋 (預設黑棋先行)
_WHITE: str = "Ｏ"              # 白棋

# 定義棋盤的大小
_NUM_OF_ROW: int = 6            # 六行
_NUM_OF_COL: int = 7            # 七列

# 定義全局變量
_board: [[str]] = [[]]          # $棋盤$
_current_player: str = _BLACK   # $當前玩家$


def init_board():
    """
    初始化 $棋盤$
    :return: None
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL
    _board = []
    for _ in range(_NUM_OF_ROW):
        _board.append([])
        for _ in range(_NUM_OF_COL):
            _board[-1].append(_EMPTY)


def get_choices() -> [int]:
    """
    :return: 可選擇下棋的位置列表
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL, _EMPTY
    choices: [int] = []
    for j in range(_NUM_OF_COL):
        i = _NUM_OF_ROW - 1
        if _board[i][j] == _EMPTY:
            choices.append(j)
    np.random.shuffle(choices)
    return choices


def next_player():
    """
    $當前玩家$ 改變為下一回合的玩家
    :return: None
    """
    global _current_player, _BLACK, _WHITE
    if _current_player == _BLACK:
        _current_player = _WHITE
    else:
        _current_player = _BLACK


def prev_player():
    """
    $當前玩家$ 改變為上一回合的玩家
    :return: None
    """
    next_player()


def move(choice: int) -> bool:
    """
    根據 $choice$ 更改 $棋盤$ 對應位置的狀態為 $當前玩家$
    :param choice: 下棋的位置
    :return: 是否成功下棋
    """
    global _board, _NUM_OF_ROW, _EMPTY, _current_player
    if choice not in get_choices():
        return False
    for i in range(_NUM_OF_ROW):
        j = choice
        if _board[i][j] == _EMPTY:
            _board[i][j] = _current_player
            return True
    return False


def move_back(choice: int) -> bool:
    """
    根據 $choice$ 更改 $棋盤$ 對應位置的狀態為 $空白$
    :param choice: 需要移除的棋的位置
    :return: 是否成功移除
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL, _EMPTY
    if choice < 0 or choice >= _NUM_OF_COL:
        return False
    for i in range(_NUM_OF_ROW - 1, -1, -1):
        j = choice
        if _board[i][j] != _EMPTY:
            _board[i][j] = _EMPTY
            return True
    return False


def board_string() -> str:
    """
    把 $棋盤$ 變為字串形式輸出
    :return: 字串
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL
    s = ""
    for i in range(_NUM_OF_COL):
        s += "_＿"
    s += "_\n"
    for i in range(_NUM_OF_ROW - 1, -1, -1):
        for col in _board[i]:
            s += "|" + col
        s += "|\n"
    asc2 = ord("０")
    for i in range(_NUM_OF_COL):
        s += "*" + chr(asc2 + i)
    s += "*\n"
    s += "*********************"
    return s


def check_win() -> str:
    """
    檢查整個棋盤是否出現勝負局面
    :return: 勝方玩家 (空白表示未分勝負)
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL, _EMPTY, _BLACK, _WHITE
    for k in [_BLACK, _WHITE]:
        # Horizontal
        for i in range(_NUM_OF_ROW):
            length: int = 0
            for j in range(_NUM_OF_COL):
                if _board[i][j] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
        # Vertical
        for j in range(_NUM_OF_COL):
            length: int = 0
            for i in range(_NUM_OF_ROW):
                if _board[i][j] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
        # Top-Right
        for j in range(_NUM_OF_COL):
            length: int = 0
            for i in range(_NUM_OF_ROW):
                if j + i >= _NUM_OF_COL:
                    break
                if _board[i][j + i] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
        # Top-Left
        for j in range(_NUM_OF_COL):
            length: int = 0
            for i in range(_NUM_OF_ROW):
                if j - i < 0:
                    break
                if _board[i][j - i] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
        # Bottom-Left
        for i in range(_NUM_OF_ROW):
            length: int = 0
            for j in range(_NUM_OF_COL):
                if i + j >= _NUM_OF_ROW:
                    break
                if _board[i + j][j] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
        # Bottom-Right
        for i in range(_NUM_OF_ROW):
            length: int = 0
            for j in range(_NUM_OF_COL - 1, -1, -1):
                if i + _NUM_OF_COL - 1 - j >= _NUM_OF_ROW:
                    break
                if _board[i + _NUM_OF_COL - 1 - j][j] != k:
                    length = 0
                else:
                    length += 1
                if length == 4:
                    return k
    return _EMPTY


def check_draw() -> bool:
    """
    當沒有可行棋的位置為和棋
    :return: 是否和棋
    """
    return len(get_choices()) == 0


def log(message: str, is_print: bool = True):
    """
    輸出信息
    :param message: 信息內容
    :param is_print: 是否輸出
    :return: None
    """
    if is_print:
        print(message)


def human() -> (str, int):
    """
    人類玩家
    :return: 名稱, 行棋位置
    """
    global _current_player
    name: str = "Human"
    num_of_allowed_errors: int = 5
    choices = get_choices()
    for i in range(1, num_of_allowed_errors + 1):
        keyboard = input(f"({_current_player}) {choices}: ")
        if keyboard.isdigit():
            if int(keyboard) in choices:
                return name, int(keyboard)
        print(f"Wrong input ({i}/{num_of_allowed_errors}): {keyboard}")
    return name, None


def random_robot() -> (str, int):
    """
    隨機電腦
    :return: 名稱, 行棋位置
    """
    choices = get_choices()
    return "Random AI", np.random.choice(choices)


def minimax(depth: int, find_max: int = 1, alpha: int = -np.inf, beta: int = np.inf) -> (int, int):
    """
    Minimax algorithm and Alpha-beta pruning
    :param depth: 搜尋深度
    :param find_max: 由最大開始, 每層交替
    :param alpha: 最小邊界值
    :param beta: 最大邊界值
    :return: 最佳評分, 最佳行棋位置
    """
    global _board, _EMPTY

    if check_win() != _EMPTY:        # 因為檢查上一層，所以需要加負號
        return (100 + depth) * -find_max, None

    if check_draw():                 # 和局立即返回，評分為零
        return 0, None

    if depth == 0:                   # 已到最大深度立即返回，評分為零
        return 0, None

    best_value = -np.inf * find_max  # 找最大時由負無窮開始, 反之由正無窮開始
    best_choice = None

    choices = get_choices()
    for choice in choices:
        move(choice)        # 模擬下棋
        next_player()       # 變成下一位玩家
        next_best_value, _ = minimax(depth - 1, -find_max, alpha, beta)
        move_back(choice)   # 還原下棋
        prev_player()       # 變回上一位玩家

        # 乘 $find_max$ 相當於改變大小於符號
        if next_best_value * find_max > best_value * find_max:
            best_value = next_best_value
            best_choice = choice

        # Alpha-Beta剪技
        if find_max == 1:
            alpha = best_value
        else:
            beta = best_value
        if alpha >= beta:
            break

    return best_value, best_choice


def minimax_robot(depth: int = 5):
    """
    Minimax電腦
    :param depth: 搜尋深度
    :return: 名稱, 行棋選擇
    """
    _, choice = minimax(depth=depth)
    return f"Minimax Robot({depth})", choice


def start_game(black_player, white_player, is_print=True) -> (str, int):
    """
    一局遊戲的主流程
    :param black_player: 黑色玩家
    :param white_player: 白色玩家
    :param is_print: 是否輸出過程
    :return: 勝方, 回合數
    """
    global _board, _NUM_OF_ROW, _NUM_OF_COL, _EMPTY, _BLACK, _WHITE, _current_player

    # 棋盤初始化
    init_board()
    log(board_string(), is_print)

    # 預設黑棋先行
    _current_player = _BLACK

    # 玩家準備
    players = {
        _BLACK: black_player,
        _WHITE: white_player
    }

    # 第一回合
    turn: int = 1

    # 主循環
    while True:
        # 當前玩家開始決定下棋位置
        name, choice = players[_current_player]()

        # 如果選擇為 None = 投降, 跳出主循環
        if choice is None:
            log(f"TURN: {turn} -- {name}({_current_player}) => RESIGN", is_print)
            if _current_player == _BLACK:
                winner = _WHITE
            else:
                winner = _BLACK
            break

        # 下棋
        move(choice)
        log(f"TURN: {turn} -- {name}({_current_player}) => {choice}", is_print)

        # 輸出棋盤
        log(board_string(), is_print)

        # 是否分出勝負
        winner = check_win()
        if winner != _EMPTY:    # 已分勝負, 跳出主循環
            log(f"Winner: {name}({winner})", is_print)
            break

        # 是否和局
        if check_draw():        # 和局, 跳出主循環
            log("Draw", is_print)
            break

        # 否則遊戲繼續, 下一位玩家準備
        next_player()

        # 回合數 +1
        turn += 1

    return winner, turn


if __name__ == "__main__":
    # 勝率統計
    _records = {
        _BLACK: 0,
        _WHITE: 0,
        _EMPTY: 0
    }

    # 測試遊戲總局數
    _num_of_games: int = 100

    for _i in range(1, _num_of_games + 1):
        # 記錄開始時間
        _start_time = time.time()

        # 進行一局遊戲, 可更換不同的玩家, 如：human, random_robot, minimax_robot
        _winner, _turn = start_game(minimax_robot, minimax_robot, _i == 1)

        # 計算耗時
        _t = time.time() - _start_time

        # 記錄勝場
        _records[_winner] += 1

        # 輸出該局總結
        log(f"Game:{str(_i).zfill(len(str(_num_of_games)))}, "  # 遊戲場次
            f"Winner:({_winner}), "                             # 勝方玩家
            f"Turn:{str(_turn).zfill(2)}, "                     # 回合數
            f"Time:{np.round(_t, 6)}")                          # 耗時

    # 輸出統計數據
    log(f"Number of games: {_num_of_games}")                    # 遊戲總局數
    log(f"({_BLACK}): {_records[_BLACK] / _num_of_games}, "     # 黑方勝率
        f"({_WHITE}): {_records[_WHITE] / _num_of_games}, "     # 白方勝率
        f"DRAW: {_records[_EMPTY] / _num_of_games}")            # 和局概率
