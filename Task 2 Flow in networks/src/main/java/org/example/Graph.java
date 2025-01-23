package org.example;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

class Edge {
    int capacity;
    int lenght;
    int source;
    int sink;

    public Edge(int capacity, int lenght, int source, int sink) {

        this.capacity = capacity;
        this.lenght = lenght;
        this.source = source;
        this.sink = sink;

    }
}

public class Graph {

    int vertices; //number of cities (node)
    List<Edge>[] adjacencyList;

    public Graph(int vertices) {
        this.vertices = vertices;
        adjacencyList = new ArrayList[vertices];
        for (int i = 0; i < vertices; i++) {
            adjacencyList[i] = new ArrayList<>();
        }
    }

    public void addEdge(int source, int sink, int capacity, int length) {
        Edge edge = new Edge(capacity, length, source, sink);
        Edge reverseEdge = new Edge(0, length, sink, source); // Properly initialize reverse edge
        adjacencyList[source].add(edge);
        adjacencyList[sink].add(reverseEdge);


    }


    //Questıon 1
    public int maxTruck(int source, int target) {
        //matrix for residual cap.
        int[][] residualCapacity = new int[vertices][vertices];
        for (List<Edge> edges : adjacencyList) {
            for (Edge edge : edges) {
                residualCapacity[edge.source][edge.sink] = edge.capacity;
            }
        }

        int maxFlow = 0;
        while (true) {
            //edmond karp use bfs to find augmenting paths
            List<Integer> path = bfs(residualCapacity, source, target);
            if (path == null) { //Repeat until no more augmenting paths exist
                break;
            }

            int fMin = Integer.MAX_VALUE;
            for (int i = 0; i < path.size() - 1; i++) {
                int u = path.get(i);
                int v = path.get(i + 1);
                fMin = Math.min(fMin, residualCapacity[u][v]); //find min cap. (bottleneck)
            }

            maxFlow += fMin; //add bottleneck to total
//update residual
            for (int i = 0; i < path.size() - 1; i++) {
                int u = path.get(i);
                int v = path.get(i + 1);
                residualCapacity[u][v] -= fMin; //decrease for forwardaded way
                residualCapacity[v][u] += fMin; //increase Residual Capacity
            }
        }
        return maxFlow;
    }
    private List<Integer> bfs(int[][] residualCapacity, int source, int sink) {
        boolean[] visited = new boolean[vertices];
        int[] path = new int[vertices];
        Arrays.fill(path, -1);
        Queue<Integer> queue = new LinkedList<>();
        queue.add(source);
        visited[source] = true;

        while (!queue.isEmpty()) {
            int current = queue.poll();
            for (Edge edge : adjacencyList[current]) {
                int neighbor = edge.sink;
                if (!visited[neighbor] && residualCapacity[current][neighbor] > 0) {
                    visited[neighbor] = true;
                    queue.add(neighbor);
                    path[neighbor] = current;

                    if (neighbor == sink) {
                        return reconstructPath(path, sink);
                    }
                }
            }
        }
        return null;
    }
    private List<Integer> reconstructPath(int[] path, int target) {
        LinkedList<Integer> resultPath = new LinkedList<>();
        for (int at = target; at != -1; at = path[at]) {
            resultPath.addFirst(at);
        }
        return resultPath;
    }

    //Question 2


    public  int[] minCostMaxFlow( int source, int sink) {
        int totalFlow = 0;
        int totalCost = 0;

        // Initialize the residual capacity matrix
        int[][] residualCapacity = new int[vertices][vertices];
        int[][] edgeLength = new int[vertices][vertices];
        for (List<Edge> edges : adjacencyList) {
            for (Edge edge : edges) {
                residualCapacity[edge.source][edge.sink] = edge.capacity;
                edgeLength[edge.source][edge.sink] = edge.lenght;
            }
        }

        while (true) {
            int[] path = new int[vertices]; // Predecessor array
            int[] distance = new int[vertices]; // Distance from source
            boolean[] inQueue = new boolean[vertices]; // If a node is in the queue

            Arrays.fill(distance, Integer.MAX_VALUE);
            Arrays.fill(path, -1);
            distance[source] = 0;

            Queue<Integer> queue = new LinkedList<>();
            queue.add(source);
            inQueue[source] = true;

            // Modified Bellman-Ford using a queue
            while (!queue.isEmpty()) {
                int current = queue.poll();
                inQueue[current] = false;

                for (Edge edge : adjacencyList[current]) {
                    if (residualCapacity[current][edge.sink] > 0 &&
                            distance[edge.sink] > distance[current] + edgeLength[current][edge.sink]) {
                        distance[edge.sink] = distance[current] + edgeLength[current][edge.sink];
                        path[edge.sink] = current;

                        if (!inQueue[edge.sink]) {
                            queue.add(edge.sink);
                            inQueue[edge.sink] = true;
                        }
                    }
                }
            }

            // If no path finish the alg.
            if (path[sink] == -1) {
                break;
            }

            // Find bottleneck capacity for the shortest path
            int pathFlow = Integer.MAX_VALUE;
            for (int v = sink; v != source; v = path[v]) {
                pathFlow = Math.min(pathFlow, residualCapacity[path[v]][v]);
            }

            // Update residual capacities and calculate total cost
            for (int v = sink; v != source; v = path[v]) { //traverse the path in reverse order
                residualCapacity[path[v]][v] -= pathFlow;
                residualCapacity[v][path[v]] += pathFlow;
                totalCost += pathFlow * edgeLength[path[v]][v];
            }

            totalFlow += pathFlow;
        }

        return new int[]{totalFlow, totalCost};
    }



    public static void main(String[] args) {
        //String filePath = "C:\\Users\\sozcu\\Downloads\\in1 (2).txt";
       String filePath = "C:\\Users\\sozcu\\Downloads\\in2.txt";
        try {
            BufferedReader reader = new BufferedReader(new FileReader(filePath));
            String line = reader.readLine();
            int vertices = Integer.parseInt(line.split(" ")[0]);
            int edges = Integer.parseInt(line.split(" ")[1]);

            Graph graph = new Graph(vertices);

            // Read and set graph

            for (int i = 0; i < edges; i++) {
                line = reader.readLine();
                int  source = Integer.parseInt(line.split(" ")[0]) - 1;
                int  sink = Integer.parseInt(line.split(" ")[1]) - 1;
                int capacity = Integer.parseInt(line.split(" ")[2]);
                int length = Integer.parseInt(line.split(" ")[3]);
                graph.addEdge(source, sink, capacity, length);
            }

            int maxTrucks = graph.maxTruck(0, vertices - 1);
            System.out.println("Max Trucks: " + maxTrucks);

         //  List<Integer> shortestPath = graph.q2(vertices - 1, 0);
           // System.out.println("Shortest Path: " + shortestPath);


            int[] result = graph.minCostMaxFlow(0, vertices - 1);
            System.out.println("Maximum Flow: " + result[0]);
            System.out.println("Minimum Cost: " + result[1]);

            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }



    }






