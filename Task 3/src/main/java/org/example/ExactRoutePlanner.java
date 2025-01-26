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
    

        if (m == 1) {
            // Single vehicle solution
            List<Client> optimalRoute = findOptimalRoute(clients, depot);
            double totalDistance = calculateTotalDistance(optimalRoute, depot);

            System.out.printf("Optimal total distance: %.6f\n", totalDistance);
            System.out.print("Optimal route: 0 ");
            for (Client client : optimalRoute) {
                System.out.print(client.id + " ");
            }
            System.out.println("0");
        } else {
            // Multi-vehicle logic can be added here
            System.out.println("Multi-vehicle solutions are not implemented in this exact algorithm.");
        }
    }

    public static List<Client> findOptimalRoute(List<Client> clients, Client depot) {
        List<Client> bestRoute = null;
        double minDistance = Double.MAX_VALUE;

        // Generate all permutations of clients
        List<Client> candidates = new ArrayList<>(clients);
        do {
            double currentDistance = calculateTotalDistance(candidates, depot);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                bestRoute = new ArrayList<>(candidates);
            }
        } while (nextPermutation(candidates));

        return bestRoute;
    }

    private static double calculateTotalDistance(List<Client> route, Client depot) {
        double totalDistance = 0.0;
        totalDistance += depot.distanceTo(route.get(0));
        for (int i = 0; i < route.size() - 1; i++) {
            totalDistance += route.get(i).distanceTo(route.get(i + 1));
        }
        totalDistance += route.get(route.size() - 1).distanceTo(depot);
        return totalDistance;
    }

    private static boolean nextPermutation(List<Client> route) {
        int k = -1;
        for (int i = 0; i < route.size() - 1; i++) {
            if (route.get(i).id < route.get(i + 1).id) {
                k = i;
            }
        }
        if (k == -1) return false;

        int l = -1;
        for (int i = k + 1; i < route.size(); i++) {
            if (route.get(k).id < route.get(i).id) {
                l = i;
            }
        }

        Collections.swap(route, k, l);

        List<Client> sublist = route.subList(k + 1, route.size());
        Collections.reverse(sublist);

        return true;
    }
}
