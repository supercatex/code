if __name__ == '__main__':
    # define a numeric list
    S = [5, 9, 3, 1, 2, 8, 4, 7, 6]
    
    # sorted count from 0 to n - 1
    c = 0
    while c < len(S) - 1:
        
        # i from the end of list to sorted count
        i = len(S) - 1
        while i > c:
            print(S, 'compare: S[%d] < S[%d]' % (i, i - 1))
            
            # compare i & i - 1
            if S[i] < S[i - 1]:
                t = S[i]
                S[i] = S[i - 1]
                S[i - 1] = t
            
            # next step
            i = i - 1
            
        # next iteration
        c = c + 1
        print()
    
    # final output
    print(S)