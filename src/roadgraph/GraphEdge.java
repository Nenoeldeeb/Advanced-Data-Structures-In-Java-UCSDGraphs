/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author UCSD Intermediate Programming MOOC team
 *
 * A directed edge in a map graph from Node start to Node end
 */
class GraphEdge
{
	/** The name of the road */
	private String roadName;
	
	/** The type of the road */
	private String roadType;
	
	/** The two end points of the edge */
	private GraphNode start;
	private GraphNode end;
	
	
	/** The length of the road segment, in km */
	private double length;
	/**The speed limit of this edge/road.*/
	private double speed;

	static final double DEFAULT_LENGTH = 0.01;
	
	
	/** Create a new GraphEdge object
	 * 
	 * @param roadName
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	GraphEdge (String roadName, GraphNode n1, GraphNode n2)
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new GraphEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 */
	GraphEdge (String roadName, String roadType, GraphNode n1, GraphNode n2)
	{
		this(roadName, roadType, n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new GraphEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 * @param length The length of the road segment
	 */	
	GraphEdge (String roadName, String roadType,
	           GraphNode n1, GraphNode n2, double length)
	{
		this.roadName = roadName;
		start = n1;
		end = n2;
		this.roadType = roadType;
		this.length = length;
		this.setSpeed ();
	}
	
	/**
	 * return the GraphNode for the end point
	 * @return the GraphNode for the end point
	 */
	GraphNode getEndNode() {
	   return end;
	}
	
	/**
	 * Return the location of the start point
	 * @return The location of the start point as a GeographicPoint
	 */
	GeographicPoint getStartPoint()
	{
		return start.getLocation();
	}
	
	/**
	 * Return the location of the end point
	 * @return The location of the end point as a GeographicPoint
	 */
	GeographicPoint getEndPoint()
	{
		return end.getLocation();
	}
	
	/**
	 * Return the length of this road segment
	 * @return the length of the road segment
	 */
	double getLength()
	{
		return length;
	}
	
	/**
	 * Get the road's name
	 * @return the name of the road that this edge is on
	 */
	public String getRoadName()
	{
		return roadName;
	}

	/**
	 * Given one of the nodes involved in this edge, get the other one
	 * @param node The node on one side of this edge
	 * @return the other node involved in this edge
	 */
	GraphNode getOtherNode(GraphNode node)
	{
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	/**
	 * Return a String representation for this edge.
	 */
	@Override
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}

	public double getSpeed () {
		return speed;
	}

	public void setSpeed () {
		switch (this.roadType) {
			case "motorway":
				speed=180.0;break;
			case "motorway_link":
				speed=120.0;break;
			case "secondary":
				speed=80.0;break;
			case "unclassified":
				speed=70.0;break;
			case "primary":
				speed=80.0;break;
			case "residential":
				speed=50.0;break;
			case "trunk":
				speed=80.0;break;
			case "trunk_link":
				speed=70.0;break;
			case "tertiary":
				speed=80.0;break;
			case "living_street":
				speed=30.0;break;
			default:
				speed=50.0;
		}
	}

	public double getTime () {
		return length/speed;
	}
}
