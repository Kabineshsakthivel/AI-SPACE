#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <unordered_map>

using namespace std;

// Structure to represent a weighted edge in a graph
struct Edge {
    int destination, weight;
};

// Function to find the shortest paths from the source using Dijkstra's Algorithm
void dijkstra(int source, vector<vector<Edge>>& graph, vector<int>& distances, vector<int>& parent) {
    int n = graph.size();
    
    // Priority queue to store (distance, node) pairs (min-heap)
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Initialize distances to INF and set parent nodes for path tracking
    distances.assign(n, INT_MAX);
    parent.assign(n, -1);
    distances[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int current_dist = pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        if (current_dist > distances[current_node]) continue;

        // Traverse all neighbors of the current node
        for (Edge edge : graph[current_node]) {
            int next_node = edge.destination;
            int weight = edge.weight;
            int new_dist = distances[current_node] + weight;

            // If a shorter path is found, update distance and push to queue
            if (new_dist < distances[next_node]) {
                distances[next_node] = new_dist;
                parent[next_node] = current_node;
                pq.push({new_dist, next_node});
            }
        }
    }
}

// Function to print the shortest path from source to a given destination
void printPath(int destination, vector<int>& parent) {
    if (parent[destination] == -1) {
        cout << destination;
        return;
    }
    printPath(parent[destination], parent);
    cout << " -> " << destination;
}

// Driver function
int main() {
    int V, E, source;
    cout << "Enter number of locations (nodes) and roads (edges): ";
    cin >> V >> E;

    // Graph representation (Adjacency List)
    vector<vector<Edge>> graph(V);

    // Store location names for better readability
    unordered_map<int, string> locationNames;
    for (int i = 0; i < V; i++) {
        cout << "Enter name for location " << i << ": ";
        cin >> locationNames[i];
    }

    cout << "Enter roads in the format (source destination weight):" << endl;
    for (int i = 0; i < E; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        graph[u].push_back({v, w});
        graph[v].push_back({u, w}); // Assuming an undirected graph
    }

    cout << "Enter the warehouse location (source node): ";
    cin >> source;

    vector<int> distances, parent;
    dijkstra(source, graph, distances, parent);

    // Output the shortest distances and paths from the warehouse to all locations
    cout << "\nOptimized Delivery Routes from Warehouse (" << locationNames[source] << "):\n";
    for (int i = 0; i < V; i++) {
        if (i == source) continue;
        cout << "To " << locationNames[i] << " -> Shortest Time: " 
             << (distances[i] == INT_MAX ? -1 : distances[i]) << " mins, Path: ";
        if (distances[i] == INT_MAX) {
            cout << "No path available";
        } else {
            printPath(i, parent);
        }
        cout << endl;
    }

    return 0;
}
