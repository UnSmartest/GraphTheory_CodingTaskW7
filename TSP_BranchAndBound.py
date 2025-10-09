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
