package org.example;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class ExactSolutıon {


        private static double minCost = Double.POSITIVE_INFINITY; // Minimum cost of the route
        private static List<Integer> bestRoute = new ArrayList<>(); // Best route found

        public static void main(String[] args) throws IOException {
            // Parse input
            BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
            String[] firstLine = reader.readLine().split(" ");
            int n = Integer.parseInt(firstLine[0]); // Number of clients
            int m = Integer.parseInt(firstLine[1]); // Number of vehicles (only 1 used here)
            int k = Integer.parseInt(firstLine[2]); // Max clients per vehicle

            double[][] clients = new double[n][2];
            for (int i = 0; i < n; i++) {
                String[] line = reader.readLine().split(" ");
                clients[i][0] = Double.parseDouble(line[0]);
                clients[i][1] = Double.parseDouble(line[1]);
            }

            String[] depotLine = reader.readLine().split(" ");
            double[] depot = {Double.parseDouble(depotLine[0]), Double.parseDouble(depotLine[1])};

            // Solve TSP using backtracking
            double[][] distMatrix = createDistanceMatrix(clients, depot);
            boolean[] visited = new boolean[n + 1]; // To track visited nodes (including depot)
            List<Integer> currentRoute = new ArrayList<>();

            // Start backtracking from depot (0)
            visited[0] = true; // Mark depot as visited
            currentRoute.add(0); // Start at depot
            tspBacktracking(0, n, distMatrix, visited, currentRoute, 0);

            // Output results
            System.out.println(minCost);
            System.out.println(bestRoute);
        }

        // Backtracking function to solve TSP
        private static void tspBacktracking(int currentCity, int n, double[][] distMatrix,
                                            boolean[] visited, List<Integer> currentRoute, double currentCost) {
            // Base case: if all cities are visited, return to depot
            if (currentRoute.size() == n + 1) { // +1 includes the depot
                double totalCost = currentCost + distMatrix[currentCity][0]; // Add cost to return to depot
                if (totalCost < minCost) {
                    minCost = totalCost;
                    bestRoute = new ArrayList<>(currentRoute); // Update best route
                    bestRoute.add(0); // Add depot at the end
                }
                return;
            }

            // Explore all possible next cities
            for (int nextCity = 1; nextCity <= n; nextCity++) { // Cities 1 to n
                if (!visited[nextCity]) { // If the city is not visited
                    visited[nextCity] = true; // Mark as visited
                    currentRoute.add(nextCity); // Add to the current route

                    // Recur to explore further
                    tspBacktracking(nextCity, n, distMatrix, visited, currentRoute,
                            currentCost + distMatrix[currentCity][nextCity]);

                    // Backtrack: undo the current step
                    visited[nextCity] = false; // Unmark as visited
                    currentRoute.remove(currentRoute.size() - 1); // Remove from current route
                }
            }
        }

        // Create distance matrix
        private static double[][] createDistanceMatrix(double[][] clients, double[] depot) {
            int n = clients.length;
            double[][] distMatrix = new double[n + 1][n + 1];

            // From depot to clients
            for (int i = 0; i < n; i++) {
                distMatrix[0][i + 1] = calculateDistance(depot, clients[i]);
                distMatrix[i + 1][0] = distMatrix[0][i + 1];
            }

            // Between clients
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    distMatrix[i + 1][j + 1] = calculateDistance(clients[i], clients[j]);
                }
            }

            return distMatrix;
        }

        // Calculate Euclidean distance
        private static double calculateDistance(double[] p1, double[] p2) {
            return Math.sqrt(Math.pow(p1[0] - p2[0], 2) + Math.pow(p1[1] - p2[1], 2));
        }
    }


