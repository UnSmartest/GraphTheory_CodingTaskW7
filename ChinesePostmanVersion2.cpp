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
    int V;  //how many vertices
    vector<Edge> edges;  //listing all edges
    vector<vector<pair<int, int>>> adj;  //adjacency list(vertex, weight)
    vector<int> degree;  //degree of each vertex

public:
    ChinesePostman(int vertices) : V(vertices) {
        adj.resize(V);
        degree.resize(V, 0);
    }

    //add edge information (from where to who and whats their weight)
    void addEdge(int u, int v, int weight) {
        edges.push_back({u, v, weight});
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight});
        degree[u]++;
        degree[v]++;
    }

    //dijkstra’s algorithm (finding shortest path between two vertices)
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

    //find the minimum cost between odd vertices
    pair<int, vector<pair<int, int>>> findMinMatching(vector<int>& oddVertices) {
        int n = oddVertices.size();
        if (n == 0) return {0, {}};

        //calculate the shortest path between all vertices with odd degrees
        vector<vector<int>> shortestPaths(n, vector<int>(n, 0));
        vector<vector<vector<int>>> paths(n, vector<vector<int>>(n));

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                vector<int> parent(V);
                int dist = dijkstra(oddVertices[i], oddVertices[j], parent);
                shortestPaths[i][j] = shortestPaths[j][i] = dist;

                //store path
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

        //recursion to find minimum perfect matching
        findMinMatchingHelper(oddVertices, shortestPaths, paths, used, 0, 0,
                              currentMatching, minCost, bestMatching);

        return {minCost, bestMatching};
    }

    // Helper function (recursive) to compute minimum-weight perfect matching
    void findMinMatchingHelper(vector<int>& oddVertices, vector<vector<int>>& shortestPaths, vector<vector<vector<int>>>& paths, vector<bool>& used, int idx, int currentCost, vector<pair<int, int>>& currentMatching, int& minCost, vector<pair<int, int>>& bestMatching) {
        int n = oddVertices.size();

        //if all vertices are paired
        if (idx == n) {
            if (currentCost < minCost) {
                minCost = currentCost;
                bestMatching = currentMatching;
            }
            return;
        }

        //ignore already used vertices
        if (used[idx]) {
            findMinMatchingHelper(oddVertices, shortestPaths, paths, used, idx + 1, currentCost, currentMatching, minCost, bestMatching);
            return;
        }

        used[idx] = true;
        for (int j = idx + 1; j < n; j++) {
            if (!used[j]) {
                used[j] = true;
                currentMatching.push_back({oddVertices[idx], oddVertices[j]});

                //recurse to next pairing
                findMinMatchingHelper(oddVertices, shortestPaths, paths, used,
                                      idx + 1, currentCost + shortestPaths[idx][j],
                                      currentMatching, minCost, bestMatching);

                currentMatching.pop_back();
                used[j] = false;
            }
        }
        used[idx] = false;
    }

    //hierholzer’s algorithm to find the Eulerian circuit
    vector<int> findEulerianCircuit() {
        map<pair<int,int>, int> edgeCount;

        for (auto& e : edges) {
            edgeCount[{min(e.u, e.v), max(e.u, e.v)}]++;
        }

        //build adjacency list for the Eulerian traversal
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

        //perform Eulerian traversal using Hierholzer’s method
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

    //solve function (combine all algorithms)
    void solve() {
        vector<int> oddVertices = findOddVertices();

        //calculate total weight of the original graph
        int totalOriginalWeight = 0;
        for (auto& e : edges) {
            totalOriginalWeight += e.weight;
        }

        int totalCost = totalOriginalWeight;

        //if there are odd-degree vertices, make the graph Eulerian
        if (!oddVertices.empty()) {
            auto result = findMinMatching(oddVertices);
            int matchingCost = result.first;
            vector<pair<int, int>> matching = result.second;

            //duplicate the shortest paths for each matched pair
            for (auto& p : matching) {
                vector<int> parent(V);
                dijkstra(p.first, p.second, parent);

                int curr = p.second;
                while (parent[curr] != -1) {
                    int prev = parent[curr];
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

        //using Hierholzer’s algorithm to find the Eulerian circuit
        vector<int> circuit = findEulerianCircuit();

        //display the final result
        cout << "Minimum total cost: " << totalCost << endl;
        cout << "Route: ";
        for (int v : circuit) {
            cout << v << " ";
        }
        cout << endl;
    }
};

/*
Example Input for Testing
4
5
0 1 1
1 2 3
2 3 1
3 0 2
1 3 4
*/

int main() {
    int V, E;

    cout << "Enter number of vertices: ";
    cin >> V;

    cout << "Enter number of edges: ";
    cin >> E;

    ChinesePostman cpp(V);

    cout << "Enter edges:" << endl;
    for (int i = 0; i < E; i++) {
        int u, v, weight;
        cin >> u >> v >> weight;
        cpp.addEdge(u, v, weight);
    }

    cpp.solve();

    return 0;
}
