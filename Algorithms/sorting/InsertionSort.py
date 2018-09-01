def insertionSrot(n):
    # sorted count start from 1 to n
    c = 1
    while c < len(n):
        
        # compare the sorted list and swap to the right space
        i = c
        while i > 0:
            print(n, 'compare: S[%d] > S[%d]' % (i, i - 1))
            
            if n[i] < n[i - 1]:
                t = n[i] 
                n[i] = n[i - 1] 
                n[i - 1] = t
            
            i = i - 1
        
        # next iteration
        print()
        c = c + 1
    
    return n
    
if __name__ == '__main__':
    print(insertionSrot([5, 3, 4, 7, 2, 8, 6, 9, 1]))
