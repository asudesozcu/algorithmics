package org.example;

import java.util.*;

class Edge {
    //road
    int source;
    int destination;
    int weight;

    public Edge(int source, int destination, int weight) {
        this.source = source;
        this.destination = destination;
        this.weight = weight;
    }
}


public class Graph {

    int vertices; //number of cities
    List<Edge>[] adjacencyList;

    public Graph(int vertices) {
        this.vertices = vertices;
        adjacencyList = new ArrayList[vertices]; //left part of adjacenylist

        for (int i = 0; i < vertices; i++) {
            adjacencyList[i] = new ArrayList<>(); //right part of adjList. just created
        }
    }

    public void addEdge(int source, int destination, int weight) {
        Edge sourceToDestinationEdge = new Edge(source, destination, weight);
        Edge destinationToSourceEdge = new Edge(destination, source, weight);

        adjacencyList[source].add(sourceToDestinationEdge);
        adjacencyList[destination].add(destinationToSourceEdge); //since its not directional graph
    }


    public List<Edge>[] getVertices() {
        return adjacencyList;
    }

    public List<Integer> bfs(int start, int target) {
        Queue<Integer> queue = new LinkedList<>();
        boolean[] visited = new boolean[vertices]; // Track visited nodes
        Map<Integer, Integer> parentMap = new HashMap<>(); // To track paths

        // Mark the start node as visited and enqueue it
        visited[start] = true;
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.remove(); // Dequeue the front node

            // If we reach the target node, reconstruct the path
            if (current == target) {
                return reconstructPath(parentMap, start, target);
            }

            // Explore neighbors using the adjacency list
            for (Edge edge : adjacencyList[current]) {
                System.out.println("current:"+ current);

                System.out.println("edge:"+ edge.source);
                int neighbor = edge.destination;
                System.out.println("dest:"+ neighbor);
                if (!visited[neighbor]) { // If the neighbor hasn't been visited
                    visited[neighbor] = true; // Mark it as visited
                    queue.add(neighbor); // Enqueue the neighbor
                    parentMap.put(neighbor, current); // Track the path
                    System.out.println("map: "+parentMap.toString());
                }
            }
        }

        return null; // No path found
    }

    // Reconstruct the path from start to target
    private List<Integer> reconstructPath(Map<Integer, Integer> parentMap, int start, int target) {
        LinkedList<Integer> path = new LinkedList<>();
        for (Integer i = target; i != null; i = parentMap.get(i)) {
            path.addFirst(i); // Add to the front to reverse the order
        }
        return path;
    }

    public static void main(String[] args) {
        Graph graph = new Graph(7); // 7 vertices, 0 to 6
        // Total 9 edges between vertices
        graph.addEdge(1, 4, 4);
        graph.addEdge(1, 5, 2);
        graph.addEdge(5, 3, 4);
        graph.addEdge(5, 4, 1);
        graph.addEdge(3, 2, 2);
        graph.addEdge(3, 4, 1);
        graph.addEdge(4, 2, 4);
        graph.addEdge(4, 6, 6);
        graph.addEdge(2, 6, 1);

        List<Edge>[] vertices = graph.getVertices();
        for (int i = 1; i < vertices.length; i++) {
            System.out.print("Vertex " + i +" : ");
            for (Edge edge : vertices[i]) {
                System.out.print(edge.destination + " ");
            }
            System.out.println();
        }
        int start = 1;
        int target = 6;
        List<Integer> path = graph.bfs(start, target);

        if (path != null) {
            System.out.print("Path from " + start + " to " + target + ": ");
            for (Integer vertex : path) {
                System.out.print(vertex + " ");
            }
        } else {
            System.out.println("No path found from " + start + " to " + target + ".");
        }
    }
    }

