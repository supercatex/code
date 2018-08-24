if __name__ == '__main__':
    # define a numeric list
    S = [5, 3, 4, 7, 2, 8, 6, 9, 1]
    
    # sorted count start from 1 to n
    c = 1
    while c < len(S):
        
        # compare the sorted list and swap to the right space
        i = c
        while i > 0:
            print(S, 'compare: S[%d] > S[%d]' % (i, i - 1))
            
            if S[i] < S[i - 1]:
                t = S[i] 
                S[i] = S[i - 1] 
                S[i - 1] = t
            
            i = i - 1
        
        # next iteration
        print()
        c = c + 1
        
    print(S)