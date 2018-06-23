
def method_1 ( number ):
    i = 2
    while i < number:
        if number % i == 0:
            return False
        i = i + 1
    return True


def method_2 ( number ):
    i = 2
    while i * i <= number:
        if number % i == 0:
            return False
        i = i + 1
    return True


def calc ( number ):
    return method_2 ( number )


if __name__ == '__main__':
    n = 100

    i = 2
    while i <= 100:
        if calc ( i ):
            print ( i )
        i = i + 1