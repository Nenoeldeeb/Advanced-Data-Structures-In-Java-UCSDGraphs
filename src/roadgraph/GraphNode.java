/**
 * A class to represent a node in the map
 */
package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author UCSD MOOC development team
 * 
 * Class representing a vertex (or node) in our MapGraph
 *
 */
class GraphNode implements Comparable<GraphNode>
{
	/** The list of edges out of this node */
	private HashSet<GraphEdge> edges;
		
	/** the latitude and longitude of this node */
	private GeographicPoint location;


	private double currentDistance;


	private double totalDistance;
	/**The time from this node to the next neighbor.*/
	private double time;
		
	/** 
	 * Create a new GraphNode at a given Geographic location
	 * @param loc the location of this node
	 */
	GraphNode (GeographicPoint loc)
	{
		location = loc;
		edges = new HashSet<GraphEdge>();
		currentDistance = 0.0;
		totalDistance = 0.0;
	}
		
	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	void addEdge(GraphEdge edge)
	{
		edges.add(edge);
	}
	
	/**  
	 * Return the neighbors of this GraphNode
	 * @return a set containing all the neighbors of this node
	 */
	Set<GraphNode> getNeighbors()
	{
		Set<GraphNode> neighbors = new HashSet<GraphNode>();
		for (GraphEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	/**
	 * Get the geographic location that this node represents
	 * @return the geographic location of this node
	 */
	GeographicPoint getLocation()
	{
		return location;
	}
	
	/**
	 * return the edges out of this node
	 * @return a set contianing all the edges out of this node.
	 */
	Set<GraphEdge> getEdges()
	{
		return edges;
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof GraphNode) || (o == null)) {
			return false;
		}
		GraphNode node = (GraphNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a GraphNode object
	 *  @return the string representation of a GraphNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (GraphEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (GraphEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}


	public double getCurrentDistance () {
		return currentDistance;
	}

	public void setCurrentDistance (double currentDistance) {
		this.currentDistance = currentDistance;
	}

	public double getTotalDistance () {
		return totalDistance;
	}

	public void setTotalDistance (double totalDistance) {
		this.totalDistance = totalDistance;
	}

	@Override
	public int compareTo (GraphNode graphNode) {
		return Double.compare (this.getCurrentDistance (),graphNode.getCurrentDistance ());
	}

	public double getTime () {
		return time;
	}

	public void setTime (double time) {
		this.time = time;
	}
}
