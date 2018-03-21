import random
import copy


_actions = [1, 3, 7, 8]
_solution = []
temp = [None] * 1000


def f(x, depth):

    actions = copy.copy(_actions)
    random.shuffle(actions)

    for action in actions:
        if x >= action:
            flag = False
            if  x - action < len(temp) and temp[x - action] != None:
                flag = temp[x - action]
            else:
                flag = f(x - action, depth + 1)
                temp[x - action] = flag
                
            if flag == False:
                if depth == 0:
                    _solution.append(action)
                return True

    if depth == 0:
        for action in actions:
            if x >= action:
                _solution.append(action)
                break
    
    return False


if __name__ == '__main__':

    while True:
        turn = 0
        count = random.randint(20, 40)
        
        print('\n' * 100)
        print('有', count, '塊石頭, 誰拿到最後一塊就贏!')
        print('遊戲開始!!!\n')

        s = -1
        while s not in(0, 1):
            print('請選擇先後權')
            print('[0] AI先拿')
            print('[1] 您先拿')
            try:
                s = int(input('>> '))
                first = s
            except:
                pass
        
        while count > 0:
            _solution = []
            print('\n現在還有', count, '塊')
            if turn % 2 == first:
                flag = f(count, 0)
                print('AI 拿了', _solution[0], '塊')
            else:
                s = 0
                while s not in(_actions) or s > count:
                    _solution = []
                    try:
                        msg = '每次只可拿' + str(_actions) + '塊: '
                        s = int(input(msg))
                        _solution.append(s)
                    except:
                        pass

            if len(_solution) > 0:
                count = count - _solution[0]
            turn = turn + 1

        if turn % 2 != first:
            print('AI 贏了!!!')
        else:
            print('您贏了!!!')

        input('按ENTER重新開始~')
