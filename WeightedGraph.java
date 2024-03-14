package graph;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

/**
 * <P>This class represents a general "directed graph", which could 
 * be used for any purpose.  The graph is viewed as a collection 
 * of vertices, which are sometimes connected by weighted, directed
 * edges.</P> 
 * 
 * <P>This graph will never store duplicate vertices.</P>
 * 
 * <P>The weights will always be non-negative integers.</P>
 * 
 * <P>The WeightedGraph will be capable of performing three algorithms:
 * Depth-First-Search, Breadth-First-Search, and Djikatra's.</P>
 * 
 * <P>The Weighted Graph will maintain a collection of 
 * "GraphAlgorithmObservers", which will be notified during the
 * performance of the graph algorithms to update the observers
 * on how the algorithms are progressing.</P>
 */
public class WeightedGraph<V> {
	
	private Map<V, Map<V, Integer>> adjacencyList;

	private Collection<GraphAlgorithmObserver<V>> observerList;
	

	/** Initialize the data structures to "empty", including
	 * the collection of GraphAlgorithmObservers (observerList).
	 */
	public WeightedGraph() {
		adjacencyList = new HashMap<>();
        observerList = new ArrayList<>();
	}

	/** Add a GraphAlgorithmObserver to the collection maintained
	 * by this graph (observerList).
	 * 
	 * @param observer
	 */
	public void addObserver(GraphAlgorithmObserver<V> observer) {
		 observerList.add(observer);
	}

	/** Add a vertex to the graph.  If the vertex is already in the
	 * graph, throw an IllegalArgumentException.
	 * 
	 * @param vertex vertex to be added to the graph
	 * @throws IllegalArgumentException if the vertex is already in
	 * the graph
	 */
	public void addVertex(V vertex) {
		if (adjacencyList.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex already exists in the graph");
        }
        adjacencyList.put(vertex, new HashMap<>());
	}
	
	/** Searches for a given vertex.
	 * 
	 * @param vertex the vertex we are looking for
	 * @return true if the vertex is in the graph, false otherwise.
	 */
	public boolean containsVertex(V vertex) {
		return adjacencyList.containsKey(vertex);
	}

	/** 
	 * <P>Add an edge from one vertex of the graph to another, with
	 * the weight specified.</P>
	 * 
	 * <P>The two vertices must already be present in the graph.</P>
	 * 
	 * <P>This method throws an IllegalArgumentExeption in three
	 * cases:</P>
	 * <P>1. The "from" vertex is not already in the graph.</P>
	 * <P>2. The "to" vertex is not already in the graph.</P>
	 * <P>3. The weight is less than 0.</P>
	 * 
	 * @param from the vertex the edge leads from
	 * @param to the vertex the edge leads to
	 * @param weight the (non-negative) weight of this edge
	 * @throws IllegalArgumentException when either vertex
	 * is not in the graph, or the weight is negative.
	 */
	public void addEdge(V from, V to, Integer weight) {
		if (!containsVertex(from) || !containsVertex(to)) {
            throw new IllegalArgumentException("Vertices are not in the graph.");
        }
        if (weight < 0) {
            throw new IllegalArgumentException("Weight is negative.");
        }
        adjacencyList.get(from).put(to, weight);
	}

	/** 
	 * <P>Returns weight of the edge connecting one vertex
	 * to another.  Returns null if the edge does not
	 * exist.</P>
	 * 
	 * <P>Throws an IllegalArgumentException if either
	 * of the vertices specified are not in the graph.</P>
	 * 
	 * @param from vertex where edge begins
	 * @param to vertex where edge terminates
	 * @return weight of the edge, or null if there is
	 * no edge connecting these vertices
	 * @throws IllegalArgumentException if either of
	 * the vertices specified are not in the graph.
	 */
	public Integer getWeight(V from, V to) {
		if (!containsVertex(from) || !containsVertex(to)) {
            throw new IllegalArgumentException("Vertices are not in the graph.");
        }
        return adjacencyList.get(from).get(to);
	}

	/** 
	 * <P>This method will perform a Breadth-First-Search on the graph.
	 * The search will begin at the "start" vertex and conclude once
	 * the "end" vertex has been reached.</P>
	 * 
	 * <P>Before the search begins, this method will go through the
	 * collection of Observers, calling notifyBFSHasBegun on each
	 * one.</P>
	 * 
	 * <P>Just after a particular vertex is visited, this method will
	 * go through the collection of observers calling notifyVisit
	 * on each one (passing in the vertex being visited as the
	 * argument.)</P>
	 * 
	 * <P>After the "end" vertex has been visited, this method will
	 * go through the collection of observers calling 
	 * notifySearchIsOver on each one, after which the method 
	 * should terminate immediately, without processing further 
	 * vertices.</P> 
	 * 
	 * @param start vertex where search begins
	 * @param end the algorithm terminates just after this vertex
	 * is visited
	 */
	public void DoBFS(V start, V end) {
		notifyBFSHasBegun();
        
		Queue<V> queue = new LinkedList<>(); // Queue to manage the vertices to visit.
        Set<V> visited = new HashSet<>(); // Set to keep track of visited vertices.
        
        queue.add(start);
        visited.add(start);
        
        while (!queue.isEmpty()) {
            V current = queue.poll(); // Retrieve and remove the head of the queue.
            notifyVisit(current);
           
            if (current.equals(end)) {
                break; // Stop if the end vertex is reached.
            }
            
            // Add unvisited neighboring vertices to the queue.
            for (V neighbor : adjacencyList.get(current).keySet()) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    queue.add(neighbor);
                }
            }
        }
        notifySearchIsOver();
    }

	
	/** 
	 * <P>This method will perform a Depth-First-Search on the graph.
	 * The search will begin at the "start" vertex and conclude once
	 * the "end" vertex has been reached.</P>
	 * 
	 * <P>Before the search begins, this method will go through the
	 * collection of Observers, calling notifyDFSHasBegun on each
	 * one.</P>
	 * 
	 * <P>Just after a particular vertex is visited, this method will
	 * go through the collection of observers calling notifyVisit
	 * on each one (passing in the vertex being visited as the
	 * argument.)</P>
	 * 
	 * <P>After the "end" vertex has been visited, this method will
	 * go through the collection of observers calling 
	 * notifySearchIsOver on each one, after which the method 
	 * should terminate immediately, without visiting further 
	 * vertices.</P> 
	 * 
	 * @param start vertex where search begins
	 * @param end the algorithm terminates just after this vertex
	 * is visited
	 */
	public void DoDFS(V start, V end) {
		notifyDFSHasBegun();
	    Set<V> visited = new HashSet<>(); // Set to keep track of visited vertices.
	    boolean found = dfsRecursive(start, end, visited); // Start the recursive DFS function.
	    if (found) {
	        notifySearchIsOver();
	    }
	}
	/**
	 * The recursive function for performing DFS.
	 * 
	 * @param current The current vertex being visited.
	 * @param end The end vertex to reach.
	 * @param visited A set of visited vertices to avoid cycles and repeats.
	 * @return true if the end vertex is found, false otherwise.
	 */
	private boolean dfsRecursive(V current, V end, Set<V> visited) {
	    notifyVisit(current);
	    visited.add(current);
	    
	    if (current.equals(end)) {
	        return true; // End vertex found.
	    }
	    
	    // Recursively visit each unvisited neighbor.
	    for (V neighbor : adjacencyList.get(current).keySet()) {
	        if (!visited.contains(neighbor) && dfsRecursive(neighbor, end, visited)) {
	            return true;
	        }
	    }
	    
	    return false; // End vertex not found in this path
	}
	
	/** 
	 * <P>Perform Dijkstra's algorithm, beginning at the "start"
	 * vertex.</P>
	 * 
	 * <P>The algorithm DOES NOT terminate when the "end" vertex
	 * is reached.  It will continue until EVERY vertex in the
	 * graph has been added to the finished set.</P>
	 * 
	 * <P>Before the algorithm begins, this method goes through 
	 * the collection of Observers, calling notifyDijkstraHasBegun 
	 * on each Observer.</P>
	 * 
	 * <P>Each time a vertex is added to the "finished set", this 
	 * method goes through the collection of Observers, calling 
	 * notifyDijkstraVertexFinished on each one (passing the vertex
	 * that was just added to the finished set as the first argument,
	 * and the optimal "cost" of the path leading to that vertex as
	 * the second argument.)</P>
	 * 
	 * <P>After all of the vertices have been added to the finished
	 * set, the algorithm will calculate the "least cost" path
	 * of vertices leading from the starting vertex to the ending
	 * vertex.  Next, it will go through the collection 
	 * of observers, calling notifyDijkstraIsOver on each one, 
	 * passing in as the argument the "lowest cost" sequence of 
	 * vertices that leads from start to end (I.e. the first vertex
	 * in the list will be the "start" vertex, and the last vertex
	 * in the list will be the "end" vertex.)</P>
	 * 
	 * @param start vertex where algorithm will start
	 * @param end special vertex used as the end of the path 
	 * reported to observers via the notifyDijkstraIsOver method.
	 */
	public void DoDijsktra(V start, V end) {
		notifyDijkstraHasBegun();
        
		final Map<V, Integer> distances = new HashMap<>(); // Map to store the shortest distance to each vertex.
        final Map<V, V> predecessors = new HashMap<>(); // Map to store the preceding vertex on the shortest path.
        Set<V> verticesToProcess = new HashSet<>(adjacencyList.keySet());
        
        // Initialize distances to all vertices as infinity, except the start vertex
        for (V vertex : verticesToProcess) {
            distances.put(vertex, Integer.MAX_VALUE);
        }
        distances.put(start, 0);

        while (!verticesToProcess.isEmpty()) {
            // Find the vertex with the smallest distance in the set
            V current = null;
            int smallestDistance = Integer.MAX_VALUE;
            for (V vertex : verticesToProcess) {
                int currentDistance = distances.get(vertex);
                if (currentDistance < smallestDistance) {
                    smallestDistance = currentDistance;
                    current = vertex;
                }
            }
            
            if (current == null) {
                break; // All remaining vertices are inaccessible from start
            }

            verticesToProcess.remove(current);
            int currentDistance = distances.get(current);

            // Update the distances to neighboring vertices
            for (Map.Entry<V, Integer> edge : adjacencyList.get(current).entrySet()) {
                V neighbor = edge.getKey();
                if (verticesToProcess.contains(neighbor)) {
                    int weight = edge.getValue();
                    int distanceThroughU = currentDistance + weight;
                    if (distanceThroughU < distances.get(neighbor)) {
                        distances.put(neighbor, distanceThroughU);
                        predecessors.put(neighbor, current);
                    }
                }
            }
            notifyDijkstraVertexFinished(current, currentDistance);
        }

        // Reconstruct the shortest path from start to end, if reachable
        List<V> path = new ArrayList<>();
        V step = end;
        if (distances.get(end) != null && distances.get(end) != Integer.MAX_VALUE) {
            while (step != null) {
                path.add(step);
                step = predecessors.get(step);
            }
            Collections.reverse(path);
        }
        
        notifyDijkstraIsOver(path);
    }
	
	/**
	 * Notifies all observers that the Breadth-First Search has begun.
	 */
	private void notifyBFSHasBegun() {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyBFSHasBegun(); 
	    }
	}

	/**
	 * Notifies all observers that the Depth-First Search has begun.
	 */
	private void notifyDFSHasBegun() {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyDFSHasBegun(); 
	    }
	}

	/**
	 * Notifies all observers that a vertex has been visited.
	 *
	 * @param vertex The vertex that has been visited.
	 */
	private void notifyVisit(V vertex) {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyVisit(vertex); 
	    }
	}

	/**
	 * Notifies all observers that the search is over.
	 */
	private void notifySearchIsOver() {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifySearchIsOver(); 
	    }
	}

	/**
	 * Notifies all observers that Dijkstra's algorithm has begun.
	 */
	private void notifyDijkstraHasBegun() {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyDijkstraHasBegun(); 
	    }
	}

	/**
	 * Notifies all observers that a vertex has finished processing in Dijkstra's algorithm.
	 *
	 * @param vertex The vertex that has been processed.
	 * @param cost The cost (or distance) from the start vertex to the processed vertex.
	 */
	private void notifyDijkstraVertexFinished(V vertex, Integer cost) {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyDijkstraVertexFinished(vertex, cost); 
	    }
	}

	/**
	 * Notifies all observers that Dijkstra's algorithm is over and sends the optimal path.
	 *
	 * @param path The list of vertices that form the shortest path from start to end.
	 */
	private void notifyDijkstraIsOver(List<V> path) {
	    for (GraphAlgorithmObserver<V> observer : observerList) {
	        observer.notifyDijkstraIsOver(path); 
	    }
	}
}
