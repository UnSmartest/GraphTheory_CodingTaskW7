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