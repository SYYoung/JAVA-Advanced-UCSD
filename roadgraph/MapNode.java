package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	GeographicPoint	loc;
	List<MapEdge> edges;
	
	public MapNode(GeographicPoint location) {
		this.loc = new GeographicPoint(location.x, location.y);
		this.edges = new LinkedList<MapEdge>();
	}
}
