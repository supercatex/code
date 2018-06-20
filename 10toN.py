
digit = [ '0' , '1' , '2' , '3' , '4' , '5' , '6' , '7' , '8' , '9' , 'A' , 'B' , 'C' , 'D' , 'E', 'F' ]


def inputInteger ( message ):
    while True:
        try:
            str = input ( message )
            if str == 'q':
                exit(0)
            num = int ( str )
            return num
        except Exception as e:
            pass


def getMaxPower ( number , base ):
    power = 0
    n = 0
    while n <= number:
        power = power + 1
        n = pow ( base , power )
    return power - 1


def calc ( number , base ):

    result = ''
    power = getMaxPower ( number , base )

    while power >= 0:
        tmp = pow ( base , power )
        bit = base - 1
        num = tmp * bit

        while number - num < 0:
            num = num - tmp
            bit = bit - 1
        number = number - num
        result = result + digit[bit]

        power = power - 1

    return result


if __name__ == '__main__':
    while True:

        base = inputInteger ( '你要計算幾進制？' )
        if base < 2 or base > 16:
            print ( '不會計這個~' )
            continue
            
        number = inputInteger ( '數字是？' )
        if number >= 0:
            print ( calc ( number , base ) )
        else:
            print ( '-' + calc ( -number , base ) )
