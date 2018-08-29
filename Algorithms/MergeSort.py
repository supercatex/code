import math

def mergeSort(n):
    print('IN:', n)
    if len(n) <= 1:
        return n 
    
    m = math.floor(len(n) / 2)
    a = n[0:m]
    b = n[m:len(n)]
    print('BREAK TO:', a, b)
    
    a = mergeSort(a)
    b = mergeSort(b)
    print('SORTED:', a, b)
    
    result = []
    while len(a) > 0 or len(b) > 0:
        if len(a) > 0 and len(b) > 0:
            if a[0] < b[0]:
                result.append(a.pop(0))
            else:
                result.append(b.pop(0))
        elif len(a) == 0:
            result.append(b.pop(0))
        elif len(b) == 0:
            result.append(a.pop(0))
    print('MERGE:', result)
    
    return result
        
    
if __name__ == '__main__':
    print(mergeSort([9, 8, 7, 6, 5, 4, 3, 2, 1]))
