# Group 7 Members
| Student ID | Full Name |
| ---------- | --------- |
| 5025241012 | Dewey Sutjiadi |
| 5025241082 | Isabel Hayaaulia Ismail |
| 5025241166 | Krisna Widhi Wijaya |
| 5999251114 | Gharbi Yassine |

<br><br>

# Knight's Tour Problem
### Definition
In a chessboard, the knight moves in a capital L shape(two tiles in a direction then one tile perpendicular). The knight piece's movement is the most unique compared to the other pieces whose moves are simple and straightforward. This brings up the question : can the knight move to all tiles of the chessboard without stepping on a tile more than once? Hence, the challenge of this problem is to create a Hamiltonian Path for our knight piece. Keep in mind that a solution for this problem only exists if the chessboard's dimensions is at least 5 rows and columns.

### Algorithm
Due to Python's reputation as a simple programming language, we chose this language to solve the Knight's Tour Problem. There are two algorithms that can be used to solve the Knight's Tour Problem : Backtracking(DFS) and Warnsdorff's Algorithm(Heuristic Search). The time complexity of Backtracking is O(8^n) where 'n' is the dimension of the chessboard. This will take a huge amount of time on the classic 8x8 chessboard, but is doable on a 5x5 chessboard, which is the minimum dimension required for a Knight's Tour Problem. On the other hand, Warnsdorff's Algorithm has a time complexity of O(n^2), where 'n' is the dimension of the chessboard. The algorithm created by Warnsdorff has a significantly smaller time complexity compared to Backtracking. Hence, we will be using Warnsdorff's Algorithm.

So, how does Warnsdorff's Algorithm work? His algorithm uses a heuristic function that checks for the next-hop tile with the least possible moves onwards. By stepping onto tiles with the least possible paths, we can eliminate risky tiles that can lead to a dead-end if left for last. In our code, we first ask for the user's input to initialize our chessboard by asking the chessboard's dimension and the knight's starting tile. Then, we check which tiles can the knight go to and call a function that checks how many tiles can the next-tile go to. The value of the function is returned and added into an array, which will be sorted ascendingly so we can get the next-hop tile with the least possible moves. After finding the next-hop tile with the least possible moves, we set the current tile as explored and set the current tile to the next-hop tile. This process repeats for n^2-1 times, n being the dimension of the chessboard.

The output's print format is essentially the coordinate of the tiles the knight steps onto. Each line holds the coordinate of the knight's current tile. The first number in each line represents the row and the second number in each line represents the column.

### Code
```
moves = [(1,-2),(2,-1),(2,1),(1,2),(-1,2),(-2,1),(-2,-1),(-1,-2)]

def lineSeperator():
    print("=============================================")

def canGoThere(currentRow, currentColumn, boardSize, explored):
    return (0 <= currentRow < boardSize and 0 <= currentColumn < boardSize and explored[currentRow][currentColumn] == 0)

def checkTiles(currentRow, currentColumn, boardSize, explored):
    available = 0
    for moveRow,moveColumn in moves:
        newRow, newColumn = currentRow + moveRow, currentColumn + moveColumn
        if canGoThere(newRow, newColumn, boardSize, explored):
            available = available + 1
    return available

print("WARNSDORFF'S ALGORITHM : INITIALIZATION")
lineSeperator()
print("What is your chessboard's size?")
boardSize = int(input())
print("What is the row-coordinate of your starting point? Keep it in your chessboard's size.")
startRow = int(input())
print("What is the column-coordinate of your starting point? Keep it in your chessboard's size.")
startColumn = int(input())
lineSeperator()

print("WARNSDORFF'S ALGORITHM : RUNNING...")
print("Your chessboard's size is", boardSize)
print("The coordinate of your starting point is", startRow, startColumn)
explored = [[0 for _ in range(boardSize)] for _ in range(boardSize)]
currentRow, currentColumn = startRow, startColumn
explored[currentRow][currentColumn] = 1
print(currentRow, currentColumn)

for iteration in range(2, boardSize*boardSize+1):
    possibleNextTile = []
    for moveRow,moveColumn in moves:
        nextRow, nextColumn = currentRow + moveRow, currentColumn + moveColumn
        if canGoThere(nextRow, nextColumn, boardSize, explored):
            availableTiles = checkTiles(nextRow, nextColumn, boardSize, explored)
            possibleNextTile.append((nextRow, nextColumn, availableTiles))
    
    if not possibleNextTile:
        print("No available steps left... Knight's tour is terminated.")
        break
    
    possibleNextTile.sort(key=lambda x: x[2])
    newRow, newColumn, _ = possibleNextTile[0]
    explored[newRow][newColumn] = 1
    print(newRow, newColumn)
    currentRow, currentColumn = newRow, newColumn
```

### Test Case : Input
```
5
2
2
```

### Test Case : Output
```
2 2
3 0
4 2
3 4
1 3
0 1
2 0
4 1
3 3
1 4
0 2
1 0
3 1
4 3
2 4
0 3
1 1
3 2
4 0
2 1
0 0
1 2
0 4
2 3
4 4
```

<br><br>

# Travelling Salesman Problem
### Definition
The Travelling Salesman Problem (TSP) is a classic optimization problem in computer science and mathematics.In other words, a salesman needs to visit several cities once and return home, and he wants to minimize the total travel cost or distance.
Formal Definition

Given:

A set of n vertices (cities).
A distance (or cost) matrix C[i][j] representing the distance from city i to city j.

Find:

A Hamiltonian cycle (a cycle that visits every city exactly once and returns to the start)
with minimum total cost

### Algorithm
What programming language and algorithm are we using? Why do we choose that programming language/algorithm? How does the algorithm work?

### Code
```
Insert code here.
```

### Test Case : Input
```
Insert input example here.
```

### Test Case : Output
```
Insert output of the example here.
Include screenshot if possible.
```

<br><br>

# Chinese Postman Problem
### Definition
What is this problem about?

### Algorithm
What programming language and algorithm are we using? Why do we choose that programming language/algorithm? How does the algorithm work?

### Code
```
Insert code here.
```

### Test Case : Input
```
Insert input example here.
```

### Test Case : Output
```
Insert output of the example here.
Include screenshot if possible.
```
