# https://squares.org/
import pyautogui
import time

class Tire(object):
    def __init__(self):
        self.data = {}

    def load_file(self, path):
        with open(path, "a+") as f:
            pass
        with open(path, "r") as f:
            self.words = f.readlines()
        for word in self.words:
            self.insert(word.strip().lower())

    def insert(self, word):
        tree = self.data
        for c in word:
            if c not in tree:
                tree[c] = {}
            tree = tree[c]
        tree['#'] = '#'

    def start_with(self, prefix):
        tree = self.data
        for c in prefix:
            if c in tree:
                tree = tree[c]
            else:
                return {}
        return tree


dict_tree = Tire()
# dict_tree.load_file("ignore_words.txt")
dict_tree.load_file("words.txt")
dict_tree.load_file("words_alpha.txt")
dict_tree.load_file("google-10000-english.txt")
ignore_tree = Tire()
# ignore_tree.load_file("ignore_words.txt")
squares = [
    # ['V', 'S', 'I', 'W'],
    # ['L', 'O', 'N', 'A'],
    # ['M', 'O', 'C', 'L'],
    # ['N', 'S', 'I', 'N']

    # ['L', 'O', 'I', 'G'],
    # ['M', 'L', 'F', 'U'],
    # ['E', 'Y', 'E', 'T'],
    # ['D', 'N', 'J', 'W']

    ['N', 'I', 'S', 'N'],
    ['L', 'C', 'O', 'M'],
    ['A', 'N', 'O', 'L'],
    ['W', 'I', 'S', 'V']
]
highlight = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]
steps = []
answers = []
word_steps = {}

def dfs(x, y, s):
    global dict_tree, ignore_tree, squares, highlight, answers, steps, word_steps
    if y < 0 or x < 0: return
    if y >= len(squares) or x >= len(squares[y]): return
    if highlight[y][x] == 1: return
    ss = s + squares[y][x].lower()
    keys = dict_tree.start_with(ss).keys()
    if len(keys) == 0: return
    highlight[y][x] = 1
    steps.append((x, y))
    if '#' in keys:
        if len(ss) >= 4 and '#' not in ignore_tree.start_with(ss):
            answers.append(ss)
            word_steps[ss] = steps.copy()

    for i in range(-1, 1 + 1, 1):
        for j in range(-1, 1 + 1, 1):
            if i == 0 and j == 0: continue
            dfs(x + j, y + i, ss)
    highlight[y][x] = 0
    steps.pop()

for i in range(4):
    for j in range(4):
        dfs(i, j, "")

answers = list(set(answers))
answers.sort()
for word in answers:
    print(word)
print("found:", len(answers))
# exit(1)

# for word in answers:
#     print(word, word_steps[word])
# print(len(answers))
# ignore_words = answers.copy()
# with open("bingo_words.txt", "r") as f:
#     bingo_words = f.readlines()
#     for word in bingo_words:
#         if word.strip().lower() in ignore_words:
#             ignore_words.remove(word.strip().lower())
# with open("ignore_words.txt", "a+") as f:
#     for word in ignore_words:
#         f.write(word + "\n")

print("請移到左上角那一格的正中央。")
time.sleep(3)
pos1 = pyautogui.position()

print("請移到右下角那一格的正中央。")
time.sleep(3)
pos2 = pyautogui.position()

pos = []
for i in range(4):
    row = []
    for j in range(4):
        w = (pos2.x - pos1.x) // 3
        h = (pos2.y - pos1.y) // 3
        x = pos1.x + w * j
        y = pos1.y + h * i
        row.append((x, y))
    pos.append(row)

t = 0.01
for c, word in enumerate(answers):
    print("%d/%d:" % (c, len(answers)), word, word_steps[word])
    for k, (j, i) in enumerate(word_steps[word]):
        x = pos[i][j][0]
        y = pos[i][j][1]
        # pyautogui.moveTo(x, y, duration=t)
        # pyautogui.mouseDown(button="left")
        # pyautogui.mouseUp(button="left")
        if k == 0:
            pyautogui.moveTo(x, y, duration=t)
            pyautogui.mouseDown(button="left")
        else:
            pyautogui.moveTo(x, y, duration=t)
        time.sleep(t)
        # if k == len(word_steps[word]) - 1:
        #     pyautogui.moveTo(pos2.x, pos2.y + (pos2.y - pos1.y) // 3, duration=t)
        #     pyautogui.mouseDown(button="left")
        #     pyautogui.mouseUp(button="left")
        #     time.sleep(t)
    pyautogui.mouseUp(button="left")
    time.sleep(1)
