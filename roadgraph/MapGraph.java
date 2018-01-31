/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
import week3example.MazeNode;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	HashMap<GeographicPoint, MapNode> nodes;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodes = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return nodes.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return nodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int number = 0;
		// for each node, get the number of edges
		for (MapNode each: nodes.values()) {
			number += each.edges.size();
		}
		return number;
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
		// check if the vertex is null or was already in the graph, return false
		if ((location == null) || nodes.containsKey(location))
			return false;
		// create a new MapNode with this new location
		MapNode newNode = new MapNode(location);
		// update the hashmap
		nodes.put(location, newNode);
		return true;
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

		if (! nodes.containsKey(from) || !nodes.containsKey(to))
			throw new IllegalArgumentException("the node(s) does not exist");
		if ((from==null) || (to==null) || (roadName==null) || (roadType == null) || (length<0)) 
			throw new IllegalArgumentException("the arguments are null or length is less than zero");
		
		// create a MapEdge
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		// look for the "from node" to add the edge
		MapNode whichNode = nodes.get(from);
		// add this new edge to that node
		whichNode.edges.add(newEdge);
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
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		// 1. inistialize: queue, visted hashset, parent hashMap
		// 2. enqueue start onto the queue and add to visited
		// 3. while queue is not empty
		//		dequeue node curr from front of the queue
		//		if curr == G then return parent map
		//		for each of curr's neighbors, n, not in visited set:
		//			add n to visited set
		//			add curr as n's parent in parent map
		//			enqueue n onto the queue
		//	4. if we get here, then there is no path
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);

		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}

		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		toExplore.add(startNode);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			if (curr == goal) {
				found = true;
				break;
			}
			List<MapNode> neighbors = curr.getNeighbors();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<MapNode>();
		}
		// reconstruct the path
		LinkedList<MapNode> path = new LinkedList<MapNode>();
		MapNode curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
		

		return null;
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
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		// for testing purpose. print out the vertices and edges
		// firstMap.printVertices();
		firstMap.printGraph();
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}

	private void printAllEdges(MapNode vertex) {
		List<MapEdge> allEdge = vertex.edges;
		
		for (MapEdge route: allEdge) {
			System.out.print("\t\t" + route.end.getX() + ",\t" + route.end.getY());
			System.out.print(",\t" + route.streetName + ",\t" + route.streetType);
			System.out.println(",\t" + Math.round(route.distance));
		}
	}

	private void printGraph() {
		
		System.out.println("Inside printGraph()");
		System.out.println("Number of vertices is: " + this.getNumVertices());
		System.out.println("Number of edges is: " + this.getNumEdges());
		for (MapNode vertex: nodes.values()) {
			System.out.println(vertex.loc.getX() + ",\t" + vertex.loc.getY() +":\t" );
			printAllEdges(vertex);
		}
	}
	
	private void printVertices() {
		
		System.out.println("Inside printVertices()");
		System.out.println("Number of vertices is: " + this.getNumVertices());
		for (GeographicPoint loc: this.getVertices()) {
			System.out.println("lat: " + loc.getX() + ",\t" + "long: " + loc.getY());
		
		}
	}
	
}
