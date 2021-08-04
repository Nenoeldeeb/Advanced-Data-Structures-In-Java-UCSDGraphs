/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.BiFunction;
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
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint, GraphNode> pointNodeMap;
	private HashSet<GraphEdge> edges;

	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint, GraphNode>();
		edges = new HashSet<GraphEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
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
		if (location == null) {
			return false;
		}
		GraphNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new GraphNode (location);
			pointNodeMap.put(location, n);
			return true;
		}
		else {
			return false;
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

		GraphNode n1 = pointNodeMap.get(from);
		GraphNode n2 = pointNodeMap.get(to);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+from+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+to+"is not in graph");

		GraphEdge edge = new GraphEdge (roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
		
	}
		
	/** 
	 * Get a set of neighbor nodes from a mapNode
	 * @param node  The node to get the neighbors from
	 * @return A set containing the GraphNode objects that are the neighbors
	 * 	of node
	 */
	private Set<GraphNode> getNeighbors(GraphNode node) {
		return node.getNeighbors();
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
	public List<GeographicPoint> bfs (GeographicPoint start,
	                                  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		if(start == null || goal == null)throw new NullPointerException ("Cannot find the route from or to Null node.");
		GraphNode from = pointNodeMap.get (start);
		GraphNode to = pointNodeMap.get (goal);
		if (from == null) {
			System.err.println ("The start point is not exist.");
			return null;
		}
		if (to == null) {
			System.err.println ("The goal point is not exist.");
			return null;
		}
		Set<GraphNode> visited = new HashSet<> ();
		Queue<GraphNode> toExplore = new LinkedList<> ();
		Map<GraphNode, GraphNode> parentMap = new HashMap<> ();
		toExplore.add (from);

		boolean found = bfsSearch (to, toExplore, visited, parentMap, nodeSearched);


		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return constructPath (from, to, parentMap, found);
	}

	/*
This method performs a breadth first search algorithm.
It'd separated from bfs method to make the code more clean.
 */
	private boolean bfsSearch (GraphNode goal, Queue<GraphNode> toExplore, Set<GraphNode> visited, Map<GraphNode, GraphNode> parentMap,
	                           Consumer<GeographicPoint> nodeSearched) {
		GraphNode curr ;
		while (!toExplore.isEmpty ()) {
			curr = toExplore.remove ();
			nodeSearched.accept (curr.getLocation ());
			if (curr.equals (goal)) return true;
			for (GraphNode next : this.getNeighbors (curr)) {
				if (!visited.contains (next)) {
					visited.add (next);
					parentMap.put (next, curr);
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
	private List<GeographicPoint> constructPath (GraphNode start, GraphNode goal,
	                                             Map<GraphNode, GraphNode> parentMap, boolean found) {
		if (!found) return null;
		LinkedList<GeographicPoint> path = new LinkedList<> ();
		GraphNode curr = goal;
		while (!curr.equals (start)) {
			path.addFirst (curr.getLocation ());
			curr = parentMap.get (curr);
		}
		path.addFirst (start.getLocation ());
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
		GraphNode from = pointNodeMap.get (start);
		GraphNode to = pointNodeMap.get (goal);
		if (start == null || goal == null || from == null || to == null) return null;
		Set<GraphNode> visited = new HashSet<> ();
		PriorityQueue<GraphNode> toExplore = new PriorityQueue<> ();
		Map<GraphNode, GraphNode> parentMap = new HashMap<> ();
		this.setInitialDistance ();
		from.setCurrentDistance (0.0);
		from.setTotalDistance (0.0);
		toExplore.add (from);
		boolean found = advancedSearch (to, toExplore,visited,parentMap,nodeSearched,(a,b)->0.0);
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return constructPath (from, to,parentMap,found);
	}

	/*
	This resets the vertices distance to positive infinity values.
	 */
	private void setInitialDistance () {
		for (GraphNode node : pointNodeMap.values ()) {
			node.setCurrentDistance (Double.MAX_VALUE);
			node.setTotalDistance (Double.MAX_VALUE);
		}
	}

	private boolean advancedSearch (GraphNode goal, PriorityQueue<GraphNode> toExplore, Set<GraphNode> visited, Map<GraphNode, GraphNode> parentMap,
	                                Consumer<GeographicPoint> nodeSearched, BiFunction<GraphNode,GraphNode,Double> bf) {
		while (!toExplore.isEmpty ()) {
			GraphNode curr = toExplore.remove ();
			nodeSearched.accept (curr.getLocation ());
			if(!visited.contains (curr)) {
				visited.add (curr);
				if (curr.equals (goal)) return true;
				Map<GraphNode,Double> distances = this.getNeighborsWithDistances (curr);
				for(GraphNode neighbor : curr.getNeighbors ()){
					double theDistance = curr.getCurrentDistance () + distances.get (neighbor);
					if (theDistance < neighbor.getCurrentDistance ()) {
						neighbor.setCurrentDistance (theDistance);
						theDistance += bf.apply (neighbor,goal);
						neighbor.setTotalDistance (theDistance);
						parentMap.put (neighbor, curr);
						toExplore.add (neighbor);
					}

				}
			}
		}
		return false;
	}

	private Map<GraphNode, Double> getNeighborsWithDistances (GraphNode node) {
		Map<GraphNode,Double> result = new HashMap<> ();
		for (GraphEdge edge : node.getEdges ()) {
			result.put (edge.getEndNode (),edge.getLength ());
		}
		return result;
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
		GraphNode from = pointNodeMap.get (start);
		GraphNode to = pointNodeMap.get (goal);
		if (start == null || goal == null || from == null || to == null) return null;
		Set<GraphNode> visited = new HashSet<> ();
		PriorityQueue<GraphNode> toExplore = new PriorityQueue<> ();
		Map<GraphNode, GraphNode> parentMap = new HashMap<> ();
		this.setInitialDistance ();
		from.setCurrentDistance (0.0);
		from.setTotalDistance (0.0);
		toExplore.add (from);
		boolean found = advancedSearch (to, toExplore,visited,parentMap,nodeSearched,(a,b)->a.getLocation ().distance (b.getLocation ()));
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return constructPath (from,to,parentMap,found);
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
