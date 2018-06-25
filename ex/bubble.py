
numbers = [8, 3, 9, 2, 6, 7, 1, 4, 5]

def bubble ( n ):
    i = 0
    while i < len ( n ) - 1:
        j = i + 1
        while j < len ( n ):
            if n[i] > n[j]:
                m = n[i]
                n[i] = n[j]
                n[j] = m
            j = j + 1
        i = i + 1
    return n


print ( numbers )
print ( bubble ( numbers ) )
