def bubbleSort(n):
    # sorted count from 0 to n - 1
    c = 0
    while c < len(n) - 1:
        
        # i from the end of list to sorted count
        i = len(n) - 1
        while i > c:
            print(n, 'compare: S[%d] < S[%d]' % (i, i - 1))
            
            # compare i & i - 1
            if n[i] < n[i - 1]:
                t = n[i]
                n[i] = n[i - 1]
                n[i - 1] = t
            
            # next step
            i = i - 1
            
        # next iteration
        c = c + 1
    
    return n

if __name__ == '__main__':
    print(bubbleSort([5, 9, 3, 1, 2, 8, 4, 7, 6]))
