package org.example;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

//x-x   x:vertex(same with the node) - :edge
class Edge {

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

    private static int i;
    int vertices; //number of cities (node)
    List<Edge>[] adjacencyList;
    double[] latitudes;
    double[] longitudes;

    public Graph(int vertices) {
        this.vertices = vertices;
        adjacencyList = new ArrayList[vertices];
        latitudes = new double[vertices];
        longitudes = new double[vertices];

        for (int i = 0; i < vertices; i++) {
            adjacencyList[i] = new ArrayList<>();
        }
    }


    public void addEdge(int source, int destination, int weight) {
        // Adjust for 1-based indexing by subtracting 1 from the vertex indices
        source--;
        destination--;

        Edge sourceToDestinationEdge = new Edge(source, destination, weight);
        Edge destinationToSourceEdge = new Edge(destination, source, weight);


        adjacencyList[source].add(sourceToDestinationEdge);
        adjacencyList[destination].add(destinationToSourceEdge);// For undirected graph
    }

//QUESTION 1: Finding path between given two cities that passes through least number of other cities.
    public List<Integer> bfs(int start, int target) {
        Queue<Integer> queue = new LinkedList<>();


        boolean[] visited = new boolean[vertices];

        int[] path = new int[vertices];
        Arrays.fill(path, -1); // Set path to -1 initially
        visited[start] = true;
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.remove();

            if (current == target) {
                List<Integer> reversedpath = reconstructPath(path, target);
                return  reversedpath;

            }

            for (Edge edge : adjacencyList[current]) {

                int neighbor = edge.destination;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.add(neighbor);
                    path[neighbor] = current;



                }
            }
        }

        return null; // No path
    }

    //questıon 2: Finding path between given two cities that has the shortest total distance (WEIGHT)

    public List<Integer> shortestTotalDistance(int start, int target) {
        boolean[] visited = new boolean[vertices];
        int[] path = new int[vertices];
        int[] distance = new int[vertices];
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(node -> distance[node]));

        Arrays.fill(distance, Integer.MAX_VALUE);
        Arrays.fill(path, -1); // Set path to -1 initially

        distance[start] = 0;
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.poll();
            visited[current] = true;

            if (current == target) {
                System.out.println("Total distance: " + distance[target] );

                List<Integer> shortestPath = reconstructPath(path, target);
                System.out.println("Path: " );

                for(Integer i : shortestPath) {
                    System.out.print((i+1)+" ");
                }
                System.out.println(" ");
                return shortestPath;
            }

            for (Edge edge : adjacencyList[current]) {
                int neighbor = edge.destination;
                int newDistance = distance[current] + edge.weight;

                if (!visited[neighbor] && newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    path[neighbor] = current;
                    queue.add(neighbor);
                }
            }
        }

        return null;
    }

    private List<Integer> reconstructPath(int[] path, int target) {
        LinkedList<Integer> resultPath = new LinkedList<>();
        for (int at = target; at != -1; at = path[at]) {
            resultPath.addFirst(at); // Add each node to the front to reverse the order
        }
        return resultPath;
    }

    //Q3
    public List<Integer> aStar(int start, int target) {
        boolean[] visited = new boolean[vertices];
        int[] distance = new int[vertices];
        int[] f = new int[vertices]; //heuristic+ normal distance
        int[] path = new int[vertices];
        Arrays.fill(distance, Integer.MAX_VALUE);
        Arrays.fill(f, Integer.MAX_VALUE);
        Arrays.fill(path, -1);

        distance[start] = 0;
        f[start] = (int) heuristic(start, target);
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(node -> f[node]));
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.poll();

            if (current == target) {
                System.out.println("Total distance to target: " + distance[target] );
                List<Integer> reversedPath = reconstructPath(path, target);
                for (Integer i : reversedPath) {
                    System.out.print((i+1)+ " ");
                }
                return reversedPath ;
            }

            visited[current] = true;

            for (Edge edge : adjacencyList[current]) {
                int neighbor = edge.destination;

                if (visited[neighbor]) continue;

                int newDistance = distance[current] + edge.weight;

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    f[neighbor] = distance[neighbor] + (int) heuristic(neighbor, target);
                    path[neighbor] = current;

                    if (queue.contains(neighbor)) {
                        queue.remove(neighbor);
                    }
                    queue.add(neighbor);
                }
            }
        }
        return null;
    }


    private double GreatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
        final int Radius = 6371;
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return Radius * c; // Distance in km
    }

    // Calculate the heuristic based on Great Circle Distance
    private double heuristic(int node, int target) {
        return GreatCircleDistance(latitudes[node], longitudes[node], latitudes[target], longitudes[target]);
    }


    public static void main(String[] args) {

            BufferedReader reader;
            String filePath = "C:\\Users\\sozcu\\Downloads\\tests (1)\\in1000.txt";

            try {
                reader = new BufferedReader(new FileReader(filePath));
                String line = reader.readLine();
                Integer vertices = Integer.parseInt(line.split(" ")[0]);
                Integer edges = Integer.parseInt(line.split(" ")[1]);

                Graph graph = new Graph(vertices);

                // Read and set coordinates
                for (int i = 0; i < vertices; i++) {
                    line = reader.readLine();
                    graph.latitudes[i] = Double.parseDouble(line.split(" ")[0]);
                    graph.longitudes[i] = Double.parseDouble(line.split(" ")[1]);

                }

                // Read and add edges
                for (int i = 0; i < edges; i++) {
                    line = reader.readLine();
                    graph.addEdge(Integer.parseInt(line.split(" ")[0]), Integer.parseInt(line.split(" ")[1]), Integer.parseInt(line.split(" ")[2]));
                }

                // get vertex and edge number
                line = reader.readLine();
                int start = Integer.parseInt(line.split(" ")[0]);
                int target = Integer.parseInt(line.split(" ")[1]);

//Q1:
                List<Integer> bfsPath = graph.bfs(start - 1, target - 1); // Adjust indices for 0-based indexing
                System.out.println(bfsPath.size() - 2); // Excluding start and target

                if (bfsPath != null) {
                    System.out.print("Path from " + start + " to " + target + ": ");
                    for (Integer vertex : bfsPath) {
                        System.out.print((vertex + 1) + " "); // Adjust back to 1-based indexing for output
                    }
                    System.out.println();
                } else {
                    System.out.println("No path found from " + start + " to " + target + ".");
                }

                // Q2:
               graph.shortestTotalDistance(start - 1, target - 1); // Adjust indices for 0-based indexing


                // A* algorithm
               graph.aStar(start - 1, target - 1); // Adjust indices for 0-based indexing

                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }