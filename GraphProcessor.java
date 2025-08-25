import java.security.InvalidAlgorithmParameterException;
import java.io.*;
import java.util.*;


/**
 * Models a weighted graph of latitude-longitude points
 * and supports various distance and routing operations.
 * To do: Add your name(s) as additional authors
 * @author Brandon Fain
 * @author Owen Astrachan modified in Fall 2023
 *
 */
public class GraphProcessor {
    /**
     * Creates and initializes a graph from a source data
     * file in the .graph format. Should be called
     * before any other methods work.
     * @param file a FileInputStream of the .graph file
     * @throws Exception if file not found or error reading
     */

    // include instance variables here
    private List<Point> vertices;
    private List<int[]> edges;
    public GraphProcessor(){
        // TODO initialize instance variables
        vertices = new ArrayList<>();
        edges = new ArrayList<>();

    }

    /**
     * Creates and initializes a graph from a source data
     * file in the .graph format. Should be called
     * before any other methods work.
     * @param file a FileInputStream of the .graph file
     * @throws IOException if file not found or error reading
     */

     public void initialize(FileInputStream file) throws IOException {
        Scanner scanner = new Scanner(file); 
        try {
            // Read the number of vertices and edges
            int numVertices = scanner.nextInt(); 
            int numEdges = scanner.nextInt(); 
            scanner.nextLine(); 
    
            // Read the vertices 
            for (int i = 0; i < numVertices; i++) {
                String name = scanner.next(); 
                double lat = scanner.nextDouble();
                double lon = scanner.nextDouble(); 
                vertices.add(new Point(lat, lon));
            }
    
            // Read edges 
            for (int i = 0; i < numEdges; i++) {
                int u = scanner.nextInt();
                int v = scanner.nextInt(); 
                edges.add(new int[]{u, v});
            }
        } catch (InputMismatchException e) {
            // Handle the exception (e.g., print error message, log details)
            System.err.println("Error reading input data: " + e.getMessage());
            e.printStackTrace(); // Print stack trace for debugging
        } finally {
            scanner.close(); // Close the scanner in a finally block
        }
    }
    
    /**
     * Searches for the point in the graph that is closest in
     * straight-line distance to the parameter point p
     * @param p is a point, not necessarily in the graph
     * @return The closest point in the graph to p
     */
    public Point nearestPoint(Point p) {
        double minDistance = Double.POSITIVE_INFINITY; 
        Point nearest = null; 

        for(Point vertex : vertices){
            double distance = vertex.distance(p);
            if(distance < minDistance){
                minDistance = distance; 
                nearest = vertex;
            }
        }

        return nearest;
    }

    /**
     * Calculates the total distance along the route, summing
     * the distance between the first and the second Points, 
     * the second and the third, ..., the second to last and
     * the last. Distance returned in miles.
     * @param start Beginning point. May or may not be in the graph.
     * @param end Destination point May or may not be in the graph.
     * @return The distance to get from start to end
     */
    public double routeDistance(List<Point> route) {
        double d = 0.0;
        // TODO implement routeDistance
        for(int i = 0; i < route.size() -1; i++){
            Point current = route.get(i); 
            Point next = route.get(i + 1); 
            d += current.distance(next);
        }
        return d;
    }
    

    /**
     * Checks if input points are part of a connected component
     * in the graph, that is, can one get from one to the other
     * only traversing edges in the graph
     * @param p1 one point
     * @param p2 another point
     * @return true if and onlyu if p2 is reachable from p1 (and vice versa)
     */
    public boolean connected(Point p1, Point p2) {
        // TODO implement connected
        if(p1.equals(p2)){
            return true;
        }
        Set<Point> visited = new HashSet<>();
        Queue<Point> queue = new LinkedList<>();
        
        queue.offer(p1);
        visited.add(p1);
        
        while(!queue.isEmpty()){
            Point current = queue.poll();
            for (Point neighbor : getNeighbors(current)){
                if(!visited.contains(neighbor)){
                    if(neighbor.equals(p2)){
                        return true;
                    }
                    queue.offer(neighbor);
                    visited.add(neighbor);
                }
            }
        }
        return false;
    }
    private List<Point> getNeighbors(Point p){
        List<Point> neighbors = new ArrayList<>();
        for (int[] edge : edges){
            if(edge[0] == vertices.indexOf(p)){
                neighbors.add(vertices.get(edge[1]));
            } else if (edge[1] == vertices.indexOf(p)){
                neighbors.add(vertices.get(edge[0]));
            }
        }
        return neighbors;
    }
    /**
     * Returns the shortest path, traversing the graph, that begins at start
     * and terminates at end, including start and end as the first and last
     * points in the returned list. If there is no such route, either because
     * start is not connected to end or because start equals end, throws an
     * exception.
     * @param start Beginning point.
     * @param end Destination point.
     * @return The shortest path [start, ..., end].
     * @throws IllegalArgumentException if there is no such route, 
     * either because start is not connected to end or because start equals end.
     */
    public List<Point> route(Point start, Point end) throws IllegalArgumentException {
        if (!connected(start, end)) {
            throw new IllegalArgumentException("No route exists between start and end");
        }
        
        List<Point> path = new ArrayList<>();
        Map<Point, Point> parentMap = new HashMap<>();
        Map<Point, Double> distances = new HashMap<>();
        PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparingDouble(distances::get));
        
        // Initialize distances
        for (Point vertex : vertices) {
            distances.put(vertex, Double.POSITIVE_INFINITY); // Initially set all distances to infinity
        }
        distances.put(start, 0.0); // Set distance of start vertex to 0 as the starting point
        
        queue.add(start); // Add start vertex to the priority queue
        
        // Dijkstra's algorithm loop
        while (!queue.isEmpty()) {
            Point current = queue.poll(); // Extract the vertex with the smallest distance from the priority queue
            if (current.equals(end)) {
                break; // If we've reached the end vertex, terminate the loop
            }
            
            // Explore neighbors of the current vertex
            for (Point neighbor : getNeighbors(current)) {
                double newDistance = distances.get(current) + current.distance(neighbor); // Calculate the new distance to the neighbor
                if (newDistance < distances.get(neighbor)) { // If the new distance is shorter than the current recorded distance
                    parentMap.put(neighbor, current); // Update the parent of the neighbor
                    distances.put(neighbor, newDistance); // Update the distance to the neighbor
                    queue.add(neighbor); // Add the neighbor to the priority queue
                }
            }
        }
        
        // Reconstruct the shortest path using the parent-child relationships stored in the parentMap
        Point current = end;
        while (current != null) {
            path.add(current);
            current = parentMap.get(current);
        }
        Collections.reverse(path); // Reverse the path to get it in the correct order
        
        return path;
    }
    
    
    public static void main(String[] args) throws FileNotFoundException, IOException {
        String name = "data/usa.graph";
        GraphProcessor gp = new GraphProcessor();
        gp.initialize(new FileInputStream(name));
        System.out.println("running GraphProcessor");
    }


    
}
