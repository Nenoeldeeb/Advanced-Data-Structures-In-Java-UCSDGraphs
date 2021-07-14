package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class, which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	/**A list of edges (streets) in this graph.*/
	private final List<GraphEdge> edges;
	/**A map of graph vertices and there locations, which the keys are the location, and the value is the node itself.*/
	private final Map<GeographicPoint,GraphNode> vertices;


	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph ()
	{
		vertices = new HashMap<> ();
		edges = new ArrayList<> ();
	}
	
	/**
	 * Get the amount vertices (road intersections) in the graph
	 * @return The amount vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size ();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return new HashSet<> (vertices.keySet ());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The amount edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size ();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 */
	public void addVertex(GeographicPoint location)
	{
		if (!vertices.containsKey (location) && location != null) {
			vertices.put (location,new GraphNode (location));
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
		if(roadName == null || roadType == null || from == null || to == null)return;
		GraphEdge edge = new GraphEdge (from,to,roadName,roadType,length);
		for (GraphEdge e : edges) {
			if(e.equals (edge))return;
		}
		edges.add (edge);
		addNeighbor (from,to);
		
	}

	/*
	This method adds a neighbor to the node  when they are connected with an edge.
	 */
	private void addNeighbor (GeographicPoint node, GeographicPoint neighborNode) {
		if (vertices.containsKey (node)) {
			GraphNode main = vertices.get (node);
			if (vertices.containsKey (neighborNode)) {
				GraphNode neighbor = vertices.get (neighborNode);
				if(!main.isNeighbor (neighborNode))main.addNeighbor (neighbor);
			}
			else{
				addVertex (neighborNode);
				GraphNode neighbor = vertices.get (neighborNode);
				main.addNeighbor (neighbor);
			}
		}
		else{
			addVertex (node);
			addVertex (neighborNode);
			GraphNode main = vertices.get (node);
			GraphNode neighbor = vertices.get (neighborNode);
			main.addNeighbor (neighbor);
		}
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
		if(start == null || goal == null)return null;
		Set<GeographicPoint> visited = new HashSet<> ();
		Queue<GeographicPoint> toExplore = new LinkedList<> ();
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<> ();
		toExplore.add (start);

		boolean found = bfsSearch (goal,toExplore,visited,parentMap,nodeSearched);

		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return constructPath (start,goal,parentMap,found);
	}

	/*
	This method performs a breadth first search algorithm.
	It'd separated from bfs method to make the code more clean.
	 */
	private boolean bfsSearch (GeographicPoint goal, Queue<GeographicPoint> toExplore,
	                           Set<GeographicPoint> visited,Map<GeographicPoint, GeographicPoint> parentMap,
	                           Consumer<GeographicPoint> nodeSearched) {
		while (!toExplore.isEmpty ()) {
			GeographicPoint curr = toExplore.remove ();
			nodeSearched.accept (curr);
			if(curr.equals (goal))return true;
			List<GraphNode> neighbors = vertices.get (curr).getNeighbors ();
			ListIterator<GraphNode> it = neighbors.listIterator (neighbors.size ());
			while (it.hasPrevious ()) {
				GeographicPoint next = it.previous ().getLocation ();
				if (!visited.contains (next)) {
					visited.add (next);
					parentMap.put (next,curr);
					toExplore.add (next);
				}
			}
		}
		return false;
	}

	/*
	This method create a path from the start node to the goal on the graph.
	It'd separated from the bfs method to make code reusable and readable.
	*/
	private List<GeographicPoint> constructPath (GeographicPoint start, GeographicPoint goal,
	                                       Map<GeographicPoint, GeographicPoint> parentMap,boolean found) {
		if(!found)return null;
		LinkedList<GeographicPoint> path = new LinkedList<> ();
		GeographicPoint curr = goal;
		while (!curr.equals (start)) {
			path.addFirst (curr);
			curr = parentMap.get (curr);
		}
		path.addFirst (start);
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
		// TODO: Implement this method in WEEK 4

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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph ();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
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
