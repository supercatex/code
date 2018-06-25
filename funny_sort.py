n = [3, 2, 1, 5, 8, 4]

def findMax ( numbers ):
    index = 1
    m = numbers[0]
    while index < len ( numbers ):
        if numbers[index] > m:
            m = numbers[index]
        index = index + 1
    return m

def findMin ( numbers ):
    index = 1
    m = numbers[0]
    while index < len ( numbers ):
        if numbers[index] < m:
            m = numbers[index]
        index = index + 1
    return m

def searchNumber ( x , numbers ):
    i = 0
    while i < len ( numbers ):
        if x == numbers[i]:
            return True
        i = i + 1
    return False

i = findMin ( n )
maxinum = findMax ( n )
while i < maxinum:
    if searchNumber ( i , n ):
        print ( i )
    i = i + 1