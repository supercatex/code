
if __name__ == '__main__':
    # Tree contains 12 nodes.
    tree_names = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L']
    T = []
    for i in range(len(tree_names)):
        T.append(i)
    
    # Relationship between nodes.
    R = [
        [1, 2, 3],      # 0     A
        [4, 5],         # 1     B
        [7],            # 2     C
        [8, 9],         # 3     D
        [10],           # 4     E
        [],             # 5     F
        [],             # 6     G
        [6],            # 7     H
        [],             # 8     I
        [11],           # 9     J
        [],             # 10    K
        []              # 11    L
        ]
    
    # Using data structure Queue in BFS.
    Q = []
    
    # Adding the first node into the Queue.
    Q.append(T[0])
    
    # Loop until Queue is empty.
    while len(Q) > 0:
        # Get one node from the Queue.
        node = Q.pop(0)
        
        # do something here...
        print(tree_names[node])
        
        # Put all child nodes into the Queue.
        for i in R[node]:
            Q.append(i)
