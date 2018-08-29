import math

def mergeSort(n):
    print('IN:', n)
    if len(n) <= 1:
        print('RETURN:', n)
        return n 
    
    m = math.floor(len(n) / 2)
    a = n[0:m]
    b = n[m:len(n)]
    print('BREAK TO:', a, b)
    
    a = mergeSort(a)
    print('Now a = ', a)
    
    b = mergeSort(b)
    print('Now b = ', b)
    
    print('READY TO SORT:', a, b)
    result = []
    while not(len(a) == 0 and len(b) == 0):
        if len(a) > 0 and len(b) > 0:
            if a[0] < b[0]:
                result.append(a.pop(0))
            else:
                result.append(b.pop(0))
        elif len(a) == 0:
            result.append(b.pop(0))
        elif len(b) == 0:
            result.append(a.pop(0))
            
    print('RETURN MERGE SORT:', result)
    return result
        
    
if __name__ == '__main__':
    print(mergeSort([9, 8, 7, 6, 5, 4, 3, 2, 1]))
