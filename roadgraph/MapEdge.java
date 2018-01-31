package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	GeographicPoint	start;
	GeographicPoint	end;
	String	streetName;
	String	streetType;
	double	distance;
	
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double len) {
		this.start = new GeographicPoint(from.getX(), from.getY());
		this.end = new GeographicPoint(to.getX(), to.getY());
		this.streetName = new String(roadName);
		this.streetType = new String(roadType);
		this.distance = len;
	}
}
