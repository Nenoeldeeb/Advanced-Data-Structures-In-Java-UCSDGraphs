package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * This class created to make adding searching or adding vertices neighbors very easy.
 * It contains. All graph node variables such as a "location" and neighbors list.
 * Also contains all public and helper methods such as isNeighbor() addNeighbor() and getLocation.
 */
public class GraphNode {

	/**
	 * The location of the vertex / street intersection.
	 */
	private final GeographicPoint location;
	/**
	 * The list of neighbors vertices of this vertex that directly connect with an edge.
	 */
	private final List<GraphNode> neighbors;

	/**
	 * The constructor, which takes a location. That initialize a location and neighbors list of this vertex.
	 */
	public GraphNode (GeographicPoint location) {
		this.location = location;
		neighbors = new ArrayList<> ();
	}

	/**
	 * A getter for vertex location.
	 *
	 * @return location of this node.
	 */
	public GeographicPoint getLocation () {
		return new GeographicPoint (location.getX (), location.getY ());
	}

	/**
	 * A getter for the list of neighbors that adjacency to this node.
	 *
	 * @return A list of node that connect via an edge with this node.
	 */
	public List<GraphNode> getNeighbors () {
		return new ArrayList<> (neighbors);
	}

	/**
	 * This method checks if the given node is in neighbors list that means it is connected to this node or not.
	 *
	 * @param loc The location of checked node.
	 * @return true if the checked node in the neighbors list means it is connected with this node.
	 * Otherwise, return false.
	 */
	public boolean isNeighbor (GeographicPoint loc) {
		if (loc == null) return false;
		for (GraphNode neighbor : neighbors) {
			GeographicPoint location = neighbor.getLocation ();
			if (loc.getX () == location.getX () && loc.getY () == location.getY ()) return true;
		}
		return false;
	}

	/**
	 * This method add a node to a neighbors list of this node.
	 *
	 * @param neighbor A node to add to current node neighbors list.
	 */
	public void addNeighbor (GraphNode neighbor) {
		if (neighbor == null || isNeighbor (neighbor.getLocation ())) return;
		neighbors.add (neighbor);
	}

	@Override
	public int hashCode () {
		return location.hashCode ();
	}

	@Override
	public boolean equals (Object o) {
		if(!(o instanceof GraphNode) || o == null)return false;
		if(o == this)return true;
		GraphNode node = (GraphNode) o;
		return this.getLocation ().equals (node.getLocation ());
	}
}
