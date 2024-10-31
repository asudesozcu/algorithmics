package org.example;

import java.util.*;
import java.util.stream.Collectors;

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
    double[] latitudes;
    double[] longitudes;

    public Graph(int vertices) {
        this.vertices = vertices;
        adjacencyList = new ArrayList[vertices]; //left part of adjacenylist
        latitudes = new double[vertices];
        longitudes = new double[vertices];

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
                System.out.println("current:" + current);

                System.out.println("edge:" + edge.source);
                int neighbor = edge.destination;
                System.out.println("dest:" + neighbor);
                if (!visited[neighbor]) { // If the neighbor hasn't been visited
                    visited[neighbor] = true; // Mark it as visited
                    queue.add(neighbor); // Enqueue the neighbor
                    parentMap.put(neighbor, current); // Track the path
                    System.out.println("map: " + parentMap.toString());
                }
            }
        }

        return null; // No path found
    }

    private List<Integer> reconstructPath(Map<Integer, Integer> parentMap, int start, int target) {
        LinkedList<Integer> path = new LinkedList<>();
        for (Integer i = target; i != null; i = parentMap.get(i)) {
            path.addFirst(i); // Add to the front to reverse the order
        }
        return path;
    }

    public int shortestTotalDistance(int start, int target) {
        boolean[] visited = new boolean[vertices];
        int[] path = new int[vertices];
        int[] distance = new int[vertices];
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(node -> distance[node]));

        Arrays.fill(distance, Integer.MAX_VALUE);
        Arrays.fill(path, -1); // Set path to -1 initially to indicate no predecessors

        distance[start] = 0;
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.poll();
            visited[current] = true;

            if (current == target) {
                System.out.println("Total distance: " + distance[target] + " units");
                List<Integer> shortestPath = reconstructPath(path, target);
                System.out.println("Path: " );

                for(Integer i : shortestPath) {
                    System.out.print(i+1+" ");
                }
                System.out.println(" ");
                return distance[target];
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

        System.out.println("No path found from " + start + " to " + target + ".");
        return -1;
    }
    private List<Integer> reconstructPath(int[] path, int target) {
        LinkedList<Integer> resultPath = new LinkedList<>();
        for (int at = target; at != -1; at = path[at]) {
            resultPath.addFirst(at); // Add each node to the front to reverse the order
        }
        return resultPath;
    }
    public int aStar(int start, int target) {
        boolean[] visited = new boolean[vertices];
        int[] g = new int[vertices];
        int[] f = new int[vertices];
        int[] path = new int[vertices];
        Arrays.fill(g, Integer.MAX_VALUE);
        Arrays.fill(f, Integer.MAX_VALUE);
        Arrays.fill(path, -1);

        g[start] = 0;
        f[start] = (int) heuristic(start, target);
        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingInt(node -> f[node]));
        queue.add(start);

        while (!queue.isEmpty()) {
            int current = queue.poll();

            if (current == target) {
                System.out.println("Total distance to target: " + g[target] + " units");
                List<Integer> fullPath = reconstructAStarPath(path, target);
                for (Integer i : fullPath) {
                    System.out.print((i+1)+ " ");
                }
                return g[target];
            }

            visited[current] = true;

            for (Edge edge : adjacencyList[current]) {
                int neighbor = edge.destination;

                if (visited[neighbor]) continue;

                int tentativeG = g[current] + edge.weight;

                if (tentativeG < g[neighbor]) {
                    g[neighbor] = tentativeG;
                    f[neighbor] = g[neighbor] + (int) heuristic(neighbor, target);
                    path[neighbor] = current;

                    if (queue.contains(neighbor)) {
                        queue.remove(neighbor);
                    }
                    queue.add(neighbor);
                }
            }
        }
        System.out.println("Target not reachable");
        return -1;
    }

    private List<Integer> reconstructAStarPath(int[] path, int target) {
        LinkedList<Integer> resultPath = new LinkedList<>();
        for (int at = target; at != -1; at = path[at]) {
            resultPath.addFirst(at);
        }
        return resultPath;
    }
    // Calculate Great Circle Distance using Haversine formula
    private double haversine(double lat1, double lon1, double lat2, double lon2) {
        final int R = 6371; // Radius of the earth in kilometers
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c; // Distance in kilometers
    }

    // Method to set the coordinates of cities
    public void setCoordinates(int index, double latitude, double longitude) {
        latitudes[index] = latitude;
        longitudes[index] = longitude;
    }

    // Calculate the heuristic based on Great Circle Distance
    private double heuristic(int node, int target) {
        return haversine(latitudes[node], longitudes[node], latitudes[target], longitudes[target]);
    }


    public static void main(String[] args) {
        Graph graph = new Graph(6); // 7 vertices, 0 to 6
        // Total 9 edges between vertices
        graph.addEdge(0, 3, 4);
        graph.addEdge(0, 4, 2);
        graph.addEdge(1, 3, 4);
        graph.addEdge(2, 1, 2);
        graph.addEdge(1, 5, 1);
        graph.addEdge(2, 3, 1);
        graph.addEdge(2, 4, 4);
        graph.addEdge(3, 4, 1);
        graph.addEdge(3, 5, 6);

        graph.setCoordinates(0, 55, 16); // City 1
        graph.setCoordinates(1, 54.30, 18.20); // City 2
        graph.setCoordinates(2, 53.70, 17); // City 3
        graph.setCoordinates(3, 54.5, 18); // City 4
        graph.setCoordinates(4, 53, 15.50); // City 5
        graph.setCoordinates(5, 53.50, 19.3);
        //adj. List print
//        List<Edge>[] vertices = graph.getVertices();
//        for (int i = 1; i < vertices.length; i++) {
//            System.out.print("Vertex " + i + " : ");
//            for (Edge edge : vertices[i]) {
//                System.out.print(edge.destination + " ");
//            }
//            System.out.println();
//        }

        int start = 0;
        int target = 5;
        List<Integer> bfsPath = graph.bfs(start, target);
        System.out.println(bfsPath.size() - 2); // Excluding start and target

        if (bfsPath != null) {
            System.out.print("Path from " + (start + 1) + " to " + (target + 1) + ": ");
            for (Integer vertex : bfsPath) {
                System.out.print((vertex + 1) + " ");
            }
            System.out.println();
        } else {
            System.out.println("No path found from " + (start + 1) + " to " + (target + 1) + ".");
        }

        // Shortest path distance
        int shortestDistance = graph.shortestTotalDistance(start, target);
        if (shortestDistance != -1) {
            System.out.println("Shortest path weight from " + start + " to " + target + ": " + shortestDistance);

        } else {
            System.out.println("No path found from " + start + " to " + target + ".");
        }

        System.out.println("-------------------------------");

        // A* algorithm
        graph.aStar(start, target);


    }
}