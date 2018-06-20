
digit = [ '0' , '1' , '2' , '3' , '4' , '5' , '6' , '7' , '8' , '9' , 'A' , 'B' , 'C' , 'D' , 'E', 'F' ]


def inputInteger ( message ):
    while True:
        try:
            str = input ( message )
            if str == 'q':
                exit(0)
            num = int ( str )
            if num < 0:
                 continue
            return num
        except Exception as e:
            pass


def getMaxPower ( number , base ):
    power = 0
    n = 1
    while n <= number:
        power = power + 1
        n = pow ( base , power )
    return power - 1


def calc ( number , base ):

    power = getMaxPower ( number , base )
    result = ''

    while power >= 0:
        tmp = pow ( base , power )
        bit = base - 1
        num = tmp * bit

        if number == 0:
            result = result + digit[0]
        else:
            while number - num < 0:
                num = num - tmp
                bit = bit - 1
            number = number - num
            result = result + str ( digit[bit] )

        power = power - 1

    return result


if __name__ == '__main__':
    while True:
        base = inputInteger ( '你要計算幾進制？' )
        if base < 2 or base > 16:
            print ( '不會計這個~' )
            continue
        number = inputInteger ( '數字是？' )
        print ( calc ( number , base ) )