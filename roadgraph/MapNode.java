package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode> {
	GeographicPoint	loc;
	List<MapEdge> edges;
	private double dist;
	private double predictDist;
	private int whichCompare;
	
	static final double DEFAULT_DIST = Double.POSITIVE_INFINITY;
	static final int METRIC_ACTUAL = 0;
	static final int METRIC_PREDICT = 1;
	
	public MapNode(GeographicPoint location) {
		this.loc = new GeographicPoint(location.x, location.y);
		this.edges = new LinkedList<MapEdge>();
		this.dist = DEFAULT_DIST;
		this.predictDist = DEFAULT_DIST;
		this.whichCompare = METRIC_ACTUAL;
	}
	
	public List<MapEdge> getNeighbors() {
		return this.edges;
	}
	
	public double getDistance() {
		return this.dist;
	}
	
	public double getPredictDistance() {
		return this.predictDist;
	}

	@Override
	public int compareTo(MapNode other) {
		if (this.whichCompare == METRIC_ACTUAL) {
			return Double.compare(this.dist, other.getDistance());
		} 
		else {
			return Double.compare(this.predictDist, other.getPredictDistance());
		}
	}
	
	public void resetDistance() {
		this.dist = DEFAULT_DIST;
		this.predictDist = DEFAULT_DIST;
		this.whichCompare = METRIC_ACTUAL;
	}
	
	public void setDistance(double distance) {
		this.dist = distance;
	}
	
	public void setPredictDistance(double predDistance) {
		this.predictDist = predDistance;
	}
	
	
	public void setCompareActual() {
		this.whichCompare = METRIC_ACTUAL;
	}
	
	public void setComparePredict() {
		this.whichCompare = METRIC_PREDICT;
	}
	
	public String toString() {
		return ("loc: " + this.loc);
	}
	
	public String toStringOld() {
		return ("location: " + this.loc + "\t, distance = " + Math.round(this.dist) + "\t, pred dist = " +
					Math.round(this.predictDist));
				
	}
	
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double dist) {
		// create a MapEdge
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, dist);
		// look for the "from node" to add the edge
		// add this new edge to that node
		this.edges.add(newEdge);
	}
	
	public int getNumEdges() {
		//the number of edges
		return edges.size();
	}
	
	public void printAllEdges() {
		
		for (MapEdge route: edges) {
			System.out.print("\t\t" + route.end.getX() + ",\t" + route.end.getY());
			System.out.print(",\t" + route.streetName + ",\t" + route.streetType);
			System.out.println(",\t" + Math.round(route.getDistance()));
		}
	}
	
	public GeographicPoint getLocation() {
		return this.loc;
	}
}
