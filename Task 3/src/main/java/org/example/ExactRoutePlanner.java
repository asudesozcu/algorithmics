package org.example;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Scanner;

public class ExactRoutePlanner {

    static class Client {
        int id;
        double x, y;

        public Client(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }

        public double distanceTo(Client other) {
            return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
        }

        @Override
        public String toString() {
            return String.valueOf(id);
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        // Read input values
        System.out.print("Enter n (clients), m (vehicles), k (max clients per vehicle): ");
        int n = scanner.nextInt();
        int m = scanner.nextInt();
        int k = scanner.nextInt();

        List<Client> clients = new ArrayList<>();
        System.out.println("Enter client coordinates (x, y):");
        for (int i = 1; i <= n; i++) {
            double x = scanner.nextDouble();
            double y = scanner.nextDouble();
            clients.add(new Client(i, x, y));
        }

        System.out.print("Enter depot coordinates (x, y): ");
        double depotX = scanner.nextDouble();
        double depotY = scanner.nextDouble();
        Client depot = new Client(0, depotX, depotY);

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
