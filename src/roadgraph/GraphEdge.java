package roadgraph;

import geography.GeographicPoint;

/**
 * This class holds all graph edge data
 * such as start location , end location , name etc.
 * It makes the adding or searching edges very easy and fast.
 */
public class GraphEdge {
	/**The start location of the edge (street).*/
	private final GeographicPoint start;
	/**The end location of the edge (street).*/
	private final GeographicPoint end;
	/**The street name.*/
	private final String  name;
	/**The street type (highway city street etc).*/
	private final String type;
	/**The distance between the start point, and the end point.*/
	private final double length;

	/**
	 * The class constructor, which take and initialize all member variables.
	 *
	 * @param start The start location of the edge (street).
	 * @param end The end location of the edge (street).
	 * @param name The street name.
	 * @param type The street type (highway city street etc).
	 * @param length The distance between the start point, and the end point.
	 */
	public GraphEdge (GeographicPoint start, GeographicPoint end, String name, String type, double length) {
		this.start = start;
		this.end = end;
		this.name = name;
		this.type = type;
		this.length = length;
	}

	/**
	 * A getter for start location.
	 * @return start location.
	 */
	public GeographicPoint getStart () {
		return new GeographicPoint (start.getX (),start.getY ());
	}

	/**
	 * A getter to end location.
	 * @return end location.
	 */
	public GeographicPoint getEnd () {
		return new GeographicPoint (end.getX (),end.getY ());
	}

	/**
	 * A getter for street / edge name.
	 * @return street / edge name.
	 */
	public String getName () {
		return name;
	}

	/**
	 * A getter for street type.
	 * @return street type.
	 */
	public String getType () {
		return type;
	}

	/**
	 * A getter for street length (distance between start and end location).
	 * @return street length.
	 */
	public double getLength () {
		return length;
	}



}
