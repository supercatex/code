if __name__ == '__main__':
    # define a numeric list
    S = [5, 9, 3, 1, 2, 8, 4, 7, 6]
    
    # sorted count from 0 to n - 1
    c = 0
    while c < len(S) - 1:
        
        # default minimize index is the first one
        min_index = c
        
        # i start from the second one to the end of the list
        i = c + 1
        while i < len(S):
            
            print(S, 'the minimize data is %d' % S[min_index])
            
            # update the minimize index
            if S[i] < S[min_index]:
                min_index = i 
            
            # next step
            i = i + 1
        
        print(S, 'the final minimize data is %d\n' % S[min_index])
        
        # swap the minimize one next to the sorted numeric.
        t = S[c]
        S[c] = S[min_index]
        S[min_index] = t 
        
        # next iteration
        c = c + 1
    
    print(S)