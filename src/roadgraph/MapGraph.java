/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	/**
	 * We have chosen adjacencyList as no of edges for any point is very less compared to no of points
	 */
	Map<GeographicPoint,List<GeographicEdge>> graphMap = null;
	/**
	 * Storing the count of edges so that numberOfEdges can return in constant time.
	 */
	AtomicInteger edgeCount = null;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		graphMap = new ConcurrentHashMap<>();
		edgeCount = new AtomicInteger();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return graphMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return graphMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edgeCount.get();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(location == null || graphMap.containsKey(location)){
			return false;
		} else {
			graphMap.put(location,new ArrayList<>());
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(from == null || to == null){
			throw new IllegalArgumentException("Atleast one of the given points is not valid");
		}

		if(!graphMap.containsKey(from) || !graphMap.containsKey(to)){
			throw new IllegalArgumentException("Atleast one of the given points does not exist in the graph." +
							" call addVertex() to add the points");
		}

		if(length<0){
			throw new IllegalArgumentException("Length should be a non negative value");
		}

		GeographicEdge geographicEdge = new GeographicEdge(from, to,roadName,roadType,length);
		graphMap.get(from).add(geographicEdge);
		edgeCount.incrementAndGet();
		
	}

	/**
	 * Internal method which gives neighbor points for a given point from the Edges
	 * @param location Point for which neighbors need be given
	 * @return List of points
	 */
	private List<GeographicPoint> getNeighbors(GeographicPoint location){
		List<GeographicEdge> edges = graphMap.get(location);
		List<GeographicPoint> neighbors = new ArrayList<>();
		for(GeographicEdge edge: edges){
			neighbors.add(edge.getTo());
		}
		return neighbors;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		boolean found = bfsSearch(start, goal, nodeSearched, parentMap);

		if (!found) {
			return null;
		}
		// reconstruct the path
		return reconstrcutPath(start, goal, parentMap);
	}

	/** Do the BreadthFirst Search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @param parentMap A map for BFS to store the the connections to construct the path
	 * @return if the goal is found
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,
														Map<GeographicPoint, GeographicPoint> parentMap){
		Set<GeographicPoint> visited = new HashSet<>();
		Queue<GeographicPoint> toExplore = new LinkedList<>();
		toExplore.add(start);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			GeographicPoint curr = toExplore.remove();
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			nodeSearched.accept(curr);
			List<GeographicPoint> neighbors = getNeighbors(curr);
			ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				GeographicPoint next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		return found;

	}

	/** Reconstruct the path from BFS result
	 *
	 * @param start  The starting location
	 * @param goal   The goal location
	 * @param parentMap  Map that has the connections between points, this is populated during BFS
	 * @return
	 */
	private List<GeographicPoint> reconstrcutPath(GeographicPoint start, GeographicPoint goal,
																							Map<GeographicPoint, GeographicPoint> parentMap){
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;

	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
