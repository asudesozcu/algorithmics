package org.example;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class ExactRoutePlanner {
    // Helper method to calculate Euclidean distance
    private static double calculateDistance(double[] p1, double[] p2) {
        return Math.sqrt(Math.pow(p1[0] - p2[0], 2) + Math.pow(p1[1] - p2[1], 2));
    }

    // Parse input file
    private static InputData parseInput(String filePath) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(filePath));
        String[] firstLine = reader.readLine().trim().split(" ");
        int n = Integer.parseInt(firstLine[0]);
        int m = Integer.parseInt(firstLine[1]);
        int k = Integer.parseInt(firstLine[2]);

        double[][] clientCoords = new double[n][2];
        for (int i = 0; i < n; i++) {
            String[] line = reader.readLine().trim().split(" ");
            clientCoords[i][0] = Double.parseDouble(line[0]);
            clientCoords[i][1] = Double.parseDouble(line[1]);
        }

        String[] depotLine = reader.readLine().trim().split(" ");
        double[] depotCoords = {Double.parseDouble(depotLine[0]), Double.parseDouble(depotLine[1])};

        reader.close();
        return new InputData(n, m, k, clientCoords, depotCoords);
    }

    // Exact solution for m = 1
    private static Solution exactSolution(int n, double[][] coords, double[] depot) {
        double[][] distMatrix = createDistanceMatrix(coords, depot);
        List<Integer> visited = new ArrayList<>();
        visited.add(0); // Start at depot
        double totalDistance = 0;

        while (visited.size() < n + 1) {
            int last = visited.get(visited.size() - 1);
            double minDistance = Double.MAX_VALUE;
            int nextClient = -1;

            for (int i = 1; i <= n; i++) {
                if (!visited.contains(i) && distMatrix[last][i] < minDistance) {
                    minDistance = distMatrix[last][i];
                    nextClient = i;
                }
            }

            visited.add(nextClient);
            totalDistance += minDistance;
        }

        totalDistance += distMatrix[visited.get(visited.size() - 1)][0]; // Return to depot
        visited.add(0);

        return Solution.singleRouteSolution(totalDistance, visited);
    }
    
    // Nearest Neighbor Heuristic for multiple vehicles
    private static Solution nearestNeighbor(int n, int m, int k, double[][] coords, double[] depot) {
        double[][] distMatrix = createDistanceMatrix(coords, depot);
        Set<Integer> unvisited = new HashSet<>();
        for (int i = 1; i <= n; i++) unvisited.add(i);

        List<List<Integer>> routes = new ArrayList<>();
        double totalDistance = 0;

        for (int vehicle = 0; vehicle < m; vehicle++) {
            List<Integer> route = new ArrayList<>();
            route.add(0); // Start at depot

            while (route.size() <= k && !unvisited.isEmpty()) {
                int last = route.get(route.size() - 1);
                double minDistance = Double.MAX_VALUE;
                int nextClient = -1;

                for (int client : unvisited) {
                    if (distMatrix[last][client] < minDistance) {
                        minDistance = distMatrix[last][client];
                        nextClient = client;
                    }
                }

                if (nextClient != -1) {
                    route.add(nextClient);
                    unvisited.remove(nextClient);
                }
            }

            route.add(0); // Return to depot
            routes.add(route);
            for (int i = 0; i < route.size() - 1; i++) {
                totalDistance += distMatrix[route.get(i)][route.get(i + 1)];
            }
        }

        return new Solution(totalDistance, routes);
    }

    // 2-opt Heuristic
    private static Solution twoOpt(int n, int m, int k, double[][] coords, double[] depot) {
        double[][] distMatrix = createDistanceMatrix(coords, depot);
        List<Integer> route = new ArrayList<>();
        for (int i = 0; i <= n; i++) route.add(i); // Initial route (0 -> 1 -> 2 -> ... -> n -> 0)

        boolean improvement = true;
        while (improvement) {
            improvement = false;
            for (int i = 1; i < route.size() - 2; i++) {
                for (int j = i + 1; j < route.size() - 1; j++) {
                    double delta = -distMatrix[route.get(i - 1)][route.get(i)]
                            - distMatrix[route.get(j)][route.get(j + 1)]
                            + distMatrix[route.get(i - 1)][route.get(j)]
                            + distMatrix[route.get(i)][route.get(j + 1)];

                    if (delta < 0) {
                        Collections.reverse(route.subList(i, j + 1));
                        improvement = true;
                    }
                }
            }
        }

        double totalDistance = 0;
        for (int i = 0; i < route.size() - 1; i++) {
            totalDistance += distMatrix[route.get(i)][route.get(i + 1)];
        }

        return Solution.singleRouteSolution(totalDistance, route);
    }

        }
        totalDistance += route.get(route.size() - 1).distanceTo(depot);
        return totalDistance;
    }

    // Create distance matrix
    private static double[][] createDistanceMatrix(double[][] coords, double[] depot) {
        int n = coords.length;
        double[][] distMatrix = new double[n + 1][n + 1];

        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= n; j++) {
                if (i == 0 && j == 0) {
                    distMatrix[i][j] = 0; // Depot to depot
                } else if (i == 0) {
                    distMatrix[i][j] = calculateDistance(depot, coords[j - 1]); // Depot to client
                } else if (j == 0) {
                    distMatrix[i][j] = calculateDistance(coords[i - 1], depot); // Client to depot
                } else {
                    distMatrix[i][j] = calculateDistance(coords[i - 1], coords[j - 1]); // Client to client
                }
            }
        }

        return distMatrix;
    }

    public static void main(String[] args) {
        try {
            // Change file path for testing
          //  String filePath = "C:\\Users\\sozcu\\Downloads\\test_2cars.txt";

            String filePath = "C:\\Users\\sozcu\\Downloads\\test_1car.txt";
            InputData inputData = parseInput(filePath);

            System.out.println("Exact Solution:");
            if (inputData.m == 1) {
                Solution solution = exactSolution(inputData.n, inputData.clientCoords, inputData.depotCoords);
                System.out.println("Total Distance: " + solution.totalDistance);
                System.out.println("Route: " + solution.routes.get(0));
            }

            System.out.println("\nNearest Neighbor Heuristic:");
            Solution heuristicSolution = nearestNeighbor(inputData.n, inputData.m, inputData.k, inputData.clientCoords, inputData.depotCoords);
            System.out.println("Total Distance: " + heuristicSolution.totalDistance);
            for (List<Integer> route : heuristicSolution.routes) {
                System.out.println("Route: " + route);
            }



            System.out.println("\n2-Opt Heuristic:");
            Solution twoOptSolution = twoOpt(inputData.n, inputData.m, inputData.k, inputData.clientCoords, inputData.depotCoords);
            System.out.println("Total Distance: " + twoOptSolution.totalDistance);
            System.out.println("Route: " + twoOptSolution.routes.get(0));


            System.out.println("\nValidating Nearest Neighbor Heuristic Routes:");
            validateRoutes(heuristicSolution.routes, 400);

            System.out.println("\nValidating 2-Opt Heuristic Routes:");
            validateRoutes(twoOptSolution.routes, 400);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Input data structure
    static class InputData {
        int n, m, k;
        double[][] clientCoords;
        double[] depotCoords;

        public InputData(int n, int m, int k, double[][] clientCoords, double[] depotCoords) {
            this.n = n;
            this.m = m;
            this.k = k;
            this.clientCoords = clientCoords;
            this.depotCoords = depotCoords;
        }
        // Input data structure

    }
    // Solution structure
    static class Solution {
        double totalDistance;
        List<List<Integer>> routes;

        public Solution(double totalDistance, List<List<Integer>> routes) {
            this.totalDistance = totalDistance;
            this.routes = routes;
        }

        public static Solution singleRouteSolution(double totalDistance, List<Integer> route) {
            List<List<Integer>> routes = new ArrayList<>();
            routes.add(route);
            return new Solution(totalDistance, routes);
        }
    }
}

