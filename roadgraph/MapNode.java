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
		return ("location: " + this.loc + "\t, distance = " + Math.round(this.dist) + "\t, pred dist = " +
					Math.round(this.predictDist));
				
	}
}
