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
In a chessboard, the knight moves in a capital L shape(two tiles in a direction then one tile perpendicular). The knight piece's movement is the most unique compared to the other pieces whose moves are simple and straightforward. This brings up the question : can the knight move to all tiles of the chessboard without stepping on a tile more than once? Hence, the challenge of this problem is to create a Hamiltonian Path for our knight piece. Keep in mind that a solution for this problem only exists if the chessboard's dimension is at least 5 rows and columns.

### Algorithm
Due to Python's reputation as a simple programming language, we chose this language to solve the Knight's Tour Problem. There are two algorithms that can be used to solve the Knight's Tour Problem : Backtracking(DFS) and Warnsdorff's Algorithm(Heuristic Search). The time complexity of Backtracking is O(8^n) where 'n' is the dimension of the chessboard. This will take a huge amount of time on the classic 8x8 chessboard, but is doable on a 5x5 chessboard, which is the minimum dimension required for a Knight's Tour Problem. On the other hand, Warnsdorff's Algorithm has a time complexity of O(n^2), where 'n' is the dimension of the chessboard. The algorithm created by Warnsdorff has a significantly smaller time complexity compared to Backtracking. Hence, we will be using Warnsdorff's Algorithm.

So, how does Warnsdorff's Algorithm work? His algorithm uses a heuristic function that checks for the next-hop tile with the least possible moves onwards. By stepping onto tiles with the least possible paths, we can eliminate risky tiles that can lead to a dead-end if left for last. In our code, we first ask for the user's input to initialize our chessboard by asking the chessboard's dimension and the knight's starting tile. Then, we check which tiles can the knight go to and call a function that checks how many tiles can the next-tile go to. The value of the function is returned and added into an array, which will be sorted ascendingly so we can get the next-hop tile with the least possible moves. After finding the next-hop tile with the least possible moves, we set the current tile as explored and set the current tile to the next-hop tile. This process repeats for n^2-1 times, n being the dimension of the chessboard.

The output's print format is essentially the coordinate of the tiles the knight steps onto. Each line holds the coordinate of the knight's current tile. The first number in each line represents the row and the second number in each line represents the column.

### Code
```python
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

# Traveling Salesman Problem:


## Problem:
A salesman is traveling to different cities with different costs. How can we find the path with the most minimum cost that the salesman can take to visit all cities exactly once and come back to the starting point?
(we must start in a certain city, visit each city exactly once, and return to the starting point)

This is also known as a Hamiltonian cycle. In other words, we're asked to find the Hamiltonian cycle (path that visits every node/vertices once) of minimum cost.


## Input and output format:

### Input:

```
3 
4 
0 1 2 10 
1 2 3 5
2 3 1 7 
3 3 1 2 
1
```

- first two lines defines the number of nodes n and number of edges e respectively
- next e lines consists of set of number denoting: name of edge, two edges that it connects, and its cost
- last line denotes the starting point (and finishing point) of the TSP/CPP solution.
- it is illustrated as below:
(graph example png)

### Ouptut:

```
Cost: 17
Route: 0, 1, 3
```

- basically, the output of the route will consist of the edges to take that results in the path with the most minimum cost


## Solution + explanation:
In solving TSP, there are several methods we can use, such as:

### 1. Exact methods:
- Brute force (trying all possible routes, O(n!))
- Dynamic programming (Held–Karp algorithm, O(n²·2ⁿ))
- Branch and bound

### 2. Approximation or heuristic methods:
- Nearest Neighbor
- Genetic algorithms
- Simulated annealing
- Ant colony optimization


Something to note for this problem:
- has to start and end at the same node, therefore we can't use approaches such as the Nearest Neighbor method, since the results don't align with the given expected output.


Since the Brute-Force method isn't a viable solution for large inputs, and given the expected output, we're going to implement the Branch and Bound algorithm, which is a more optimized version of the Brute-Force algorithm.


## Code: 

The algorithm starts from the specified starting city and explores all possible routes to visit the remaining cities. As it builds each path, it keeps track of the total cost so far. If the current path already exceeds the best solution found, it prunes that path, meaning it stops exploring it further, because it cannot lead to a better solution. This pruning is what makes Branch and Bound more efficient than checking every possible route blindly.

Whenever the algorithm reaches a path that has visited all cities, it checks if it can return to the starting city to form a complete cycle. If the total cost of this cycle is lower than any previously found path, the algorithm updates the best solution. By recursively branching into new paths and bounding the non-promising ones, the algorithm eventually finds the optimal route with the minimum cost.

```python
from math import inf

def tsp_branch_bound_edges(n, e, edges, start):
    # adjacency list of edge names
    graph = {i: [] for i in range(1, n + 1)}
    for edge_name, u, v, cost in edges:
        graph[u].append((edge_name, v, cost))
        graph[v].append((edge_name, u, cost))  # undirected
    
    min_cost = [inf]
    best_route = [[]]

    def dfs(current_node, visited_nodes, total_cost, route):
        # if all nodes are visited and we can return to start
        if len(visited_nodes) == n:
            for edge_name, neighbor, cost in graph[current_node]:
                if neighbor == start:
                    final_cost = total_cost + cost
                    if final_cost < min_cost[0]:
                        min_cost[0] = final_cost
                        best_route[0] = route + [edge_name]
            return
        
        # pursue path that already exceed best cost
        if total_cost >= min_cost[0]:
            return
        
        for edge_name, neighbor, cost in graph[current_node]:
            if neighbor not in visited_nodes:
                dfs(
                    neighbor,
                    visited_nodes | {neighbor},
                    total_cost + cost,
                    route + [edge_name]
                )
    
    dfs(start, {start}, 0, [])
    return min_cost[0], best_route[0]


n = int(input().strip()) 
e = int(input().strip())  

edges = []
for _ in range(e):
    edge_data = list(map(int, input().split()))
    edges.append(tuple(edge_data))  # (edge_name, u, v, cost)

start = int(input().strip()) 

cost, route = tsp_branch_bound_edges(n, e, edges, start)

print("Cost:", cost)
print("Route:", ", ".join(map(str, route)))
```

<br><br>

# Chinese Postman Problem
### Definition
The Chinese Postman Problem, is graph theory problem that aims to find the shortest distance by traversing every edge of the graph at least once and coming back to the starting point. This problem can be applied to model real-life situations, such as postal delivery, garbage collection, or road maintenance, where a worker or vehicle must efficiently cover all streets and return to the depot or starting position with minimal travel cost.

### Algorithm
Generally, in order to solve the Chinese Postman Problem, we use Dijkstra's algorithm and Hierholzer's algorithm. The first step is to determine whether the given graph is Eulerian or not. A graph is called Eulerian if every vertex has an even degree, meaning that each vertex is connected to an even number of edges. When this condition is satisfied, the graph contains an Eulerian circuit, which is a closed path that visits every edge exactly once and returns to the starting vertex. In this case, the optimal route can be directly obtained, and the total cost will be the sum of all edge weights.

However, if the graph has one or more odd-degree vertices, it is not Eulerian, meaning it does not have an Eulerian circuit. To make it Eulerian, we first identify all the odd-degree vertices and then use Dijkstra’s algorithm to find the shortest paths between each pair of these vertices. These shortest paths help determine which edges should be duplicated with the least additional cost so that all vertices have even degrees. After modifying the graph in this way, it becomes Eulerian.

Once the graph has been balanced, we apply Hierholzer’s algorithm to find the Eulerian circuit by traversing all edges and backtracking when necessary. This ensures that every edge is visited exactly once, resulting in the shortest possible closed route that covers every edge in the network at least once and returns to the starting point
### Code
```
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <stack>
#include <map>
using namespace std;

struct Edge {
    int u, v, weight;
};

class ChinesePostman {
private:
    int V;
    vector<Edge> edges;
    vector<vector<pair<int, int>>> adj;
    vector<int> degree;
    
public:
    ChinesePostman(int vertices) : V(vertices) {
        adj.resize(V);
        degree.resize(V, 0);
    }
    
    void addEdge(int u, int v, int weight) {
        edges.push_back({u, v, weight});
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight});
        degree[u]++;
        degree[v]++;
    }
    
    int dijkstra(int src, int dest, vector<int>& parent) {
        vector<int> dist(V, INT_MAX);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        
        dist[src] = 0;
        parent[src] = -1;
        pq.push({0, src});
        
        while (!pq.empty()) {
            int u = pq.top().second;
            int d = pq.top().first;
            pq.pop();
            
            if (d > dist[u]) continue;
            
            for (auto& edge : adj[u]) {
                int v = edge.first;
                int weight = edge.second;
                
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
        
        return dist[dest];
    }
    
    vector<int> findOddVertices() {
        vector<int> oddVertices;
        for (int i = 0; i < V; i++) {
            if (degree[i] % 2 == 1) {
                oddVertices.push_back(i);
            }
        }
        return oddVertices;
    }
    
    pair<int, vector<pair<int, int>>> findMinMatching(vector<int>& oddVertices) {
        int n = oddVertices.size();
        if (n == 0) return {0, {}};
        
        vector<vector<int>> shortestPaths(n, vector<int>(n, 0));
        vector<vector<vector<int>>> paths(n, vector<vector<int>>(n));
        
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                vector<int> parent(V);
                int dist = dijkstra(oddVertices[i], oddVertices[j], parent);
                shortestPaths[i][j] = shortestPaths[j][i] = dist;
                
                vector<int> path;
                int curr = oddVertices[j];
                while (curr != -1) {
                    path.push_back(curr);
                    curr = parent[curr];
                }
                reverse(path.begin(), path.end());
                paths[i][j] = paths[j][i] = path;
            }
        }
        
        int minCost = INT_MAX;
        vector<pair<int, int>> bestMatching;
        
        vector<bool> used(n, false);
        vector<pair<int, int>> currentMatching;
        findMinMatchingHelper(oddVertices, shortestPaths, paths, used, 0, 0, 
                             currentMatching, minCost, bestMatching);
        
        return {minCost, bestMatching};
    }
    
    void findMinMatchingHelper(vector<int>& oddVertices, 
                               vector<vector<int>>& shortestPaths,
                               vector<vector<vector<int>>>& paths,
                               vector<bool>& used, int idx, int currentCost,
                               vector<pair<int, int>>& currentMatching,
                               int& minCost, vector<pair<int, int>>& bestMatching) {
        int n = oddVertices.size();
        
        if (idx == n) {
            if (currentCost < minCost) {
                minCost = currentCost;
                bestMatching = currentMatching;
            }
            return;
        }
        
        if (used[idx]) {
            findMinMatchingHelper(oddVertices, shortestPaths, paths, used, 
                                 idx + 1, currentCost, currentMatching, 
                                 minCost, bestMatching);
            return;
        }
        
        used[idx] = true;
        for (int j = idx + 1; j < n; j++) {
            if (!used[j]) {
                used[j] = true;
                currentMatching.push_back({oddVertices[idx], oddVertices[j]});
                
                findMinMatchingHelper(oddVertices, shortestPaths, paths, used,
                                     idx + 1, currentCost + shortestPaths[idx][j],
                                     currentMatching, minCost, bestMatching);
                
                currentMatching.pop_back();
                used[j] = false;
            }
        }
        used[idx] = false;
    }
    
    vector<int> findEulerianCircuit() {
        map<pair<int,int>, int> edgeCount;
        for (auto& e : edges) {
            edgeCount[{min(e.u, e.v), max(e.u, e.v)}]++;
        }
        
        vector<vector<int>> adjList(V);
        for (auto& p : edgeCount) {
            int u = p.first.first;
            int v = p.first.second;
            int count = p.second;
            for (int i = 0; i < count; i++) {
                adjList[u].push_back(v);
                adjList[v].push_back(u);
            }
        }
        
        stack<int> currPath;
        vector<int> circuit;
        
        currPath.push(0);
        int curr = 0;
        
        while (!currPath.empty()) {
            if (!adjList[curr].empty()) {
                currPath.push(curr);
                int next = adjList[curr].back();
                adjList[curr].pop_back();
                
                auto it = find(adjList[next].begin(), adjList[next].end(), curr);
                if (it != adjList[next].end()) {
                    adjList[next].erase(it);
                }
                
                curr = next;
            } else {
                circuit.push_back(curr);
                curr = currPath.top();
                currPath.pop();
            }
        }
        
        reverse(circuit.begin(), circuit.end());
        return circuit;
    }
    
    void solve() {
        // Find odd degree vertices
        vector<int> oddVertices = findOddVertices();
        
        int totalOriginalWeight = 0;
        for (auto& e : edges) {
            totalOriginalWeight += e.weight;
        }
        
        int totalCost = totalOriginalWeight;
        
        if (!oddVertices.empty()) {
            auto result = findMinMatching(oddVertices);
            int matchingCost = result.first;
            vector<pair<int, int>> matching = result.second;
            
            // Add duplicate edges
            for (auto& p : matching) {
                vector<int> parent(V);
                dijkstra(p.first, p.second, parent);
                
                // Reconstruct and add path
                int curr = p.second;
                while (parent[curr] != -1) {
                    int prev = parent[curr];
                    // Find edge weight
                    int weight = 0;
                    for (auto& neighbor : adj[prev]) {
                        if (neighbor.first == curr) {
                            weight = neighbor.second;
                            break;
                        }
                    }
                    addEdge(prev, curr, weight);
                    curr = prev;
                }
            }
            
            totalCost = totalOriginalWeight + matchingCost;
        }
        
        vector<int> circuit = findEulerianCircuit();
        
        cout << "Minimum total cost: " << totalCost << endl;
        cout << "Chinese Postman route: ";
        for (int v : circuit) {
            cout << v << " ";
        }
        cout << endl;
    }
};

int main() {
    int V, E;
    
    cout << "Enter number of vertices: ";
    cin >> V;
    
    cout << "Enter number of edges: ";
    cin >> E;
    
    ChinesePostman cpp(V);
    
    cout << "Enter edges" << endl;
    for (int i = 0; i < E; i++) {
        int u, v, weight;
        cout << "Enter edge:" << (i + 1);
        cin >> u >> v >> weight;
        cpp.addEdge(u, v, weight);
    }
    
    cpp.solve();
    
    return 0;
}
```

### Test Case : Input
```
Enter number of vertices: 4
Enter number of edges: 5
Enter edge 1: 0 1 1
Enter edge 2: 1 2 3
Enter edge 3: 2 3 1
Enter edge 4: 3 0 2
Enter edge 5: 1 3 4
```

### Test Case : Output
```
Minimum total cost: 14
Chinese Postman route: 0 3 2 1 3 0 1 0
```
