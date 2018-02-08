package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode> {
	GeographicPoint	loc;
	List<MapEdge> edges;
	private double dist;
	static final double DEFAULT_DIST = Double.POSITIVE_INFINITY;
	
	public MapNode(GeographicPoint location) {
		this.loc = new GeographicPoint(location.x, location.y);
		this.edges = new LinkedList<MapEdge>();
		this.dist = DEFAULT_DIST;
	}
	
	public List<MapEdge> getNeighbors() {
		return this.edges;
	}
	
	public double getDistance() {
		return this.dist;
	}

	@Override
	public int compareTo(MapNode other) {
		return Double.compare(this.dist, other.getDistance());
	}
	
	public void resetDistance() {
		this.dist = DEFAULT_DIST;
	}
	
	public void setDistance(double distance) {
		this.dist = distance;
	}
}
