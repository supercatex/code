def selectionSort(n):
    # sorted count from 0 to n - 1
    c = 0
    while c < len(n) - 1:
        
        # default minimize index is the first one
        min_index = c
        
        # i start from the second one to the end of the list
        i = c + 1
        while i < len(n):
            
            print(n, 'the minimize data is %d' % n[min_index])
            
            # update the minimize index
            if n[i] < n[min_index]:
                min_index = i 
            
            # next step
            i = i + 1
        
        print(n, 'the final minimize data is %d\n' % n[min_index])
        
        # swap the minimize one next to the sorted numeric.
        t = n[c]
        n[c] = n[min_index]
        n[min_index] = t 
        
        # next iteration
        c = c + 1
    
    return n
    
if __name__ == '__main__':
    print(selectionSort([5, 9, 3, 1, 2, 8, 4, 7, 6]))
