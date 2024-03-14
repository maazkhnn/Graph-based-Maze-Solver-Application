package graph;
import graph.WeightedGraph;
import maze.Juncture;
import maze.Maze;

/** 
 * <P>The MazeGraph is an extension of WeightedGraph.  
 * The constructor converts a Maze into a graph.</P>
 */
public class MazeGraph extends WeightedGraph<Juncture> {
	
	/** 
	 * <P>Construct the MazeGraph using the "maze" contained
	 * in the parameter to specify the vertices (Junctures)
	 * and weighted edges.</P>
	 * 
	 * <P>The Maze is a rectangular grid of "junctures", each
	 * defined by its X and Y coordinates, using the usual
	 * convention of (0, 0) being the upper left corner.</P>
	 * 
	 * <P>Each juncture in the maze should be added as a
	 * vertex to this graph.</P>
	 * 
	 * <P>For every pair of adjacent junctures (A and B) which
	 * are not blocked by a wall, two edges should be added:  
	 * One from A to B, and another from B to A.  The weight
	 * to be used for these edges is provided by the Maze. 
	 * (The Maze methods getMazeWidth and getMazeHeight can
	 * be used to determine the number of Junctures in the
	 * maze. The Maze methods called "isWallAbove", "isWallToRight",
	 * etc. can be used to detect whether or not there
	 * is a wall between any two adjacent junctures.  The 
	 * Maze methods called "getWeightAbove", "getWeightToRight",
	 * etc. should be used to obtain the weights.)</P>
	 * 
	 * @param maze to be used as the source of information for
	 * adding vertices and edges to this MazeGraph.
	 */
	public MazeGraph(Maze maze) {
		super(); // Initialize WeightedGraph components

		// Creating all juncture vertices based on the maze's dimensions.
	    Juncture[][] junctures = new Juncture[maze.getMazeWidth()][maze.getMazeHeight()];
	    for (int x = 0; x < maze.getMazeWidth(); x++) {
	        for (int y = 0; y < maze.getMazeHeight(); y++) {
	            junctures[x][y] = new Juncture(x, y); // Create a juncture at each coordinate.
	            addVertex(junctures[x][y]); // Add each juncture as a vertex in the graph.
	        }
	    }

	    // Adding edges between adjacent junctures where no walls exist.
	    for (int x = 0; x < maze.getMazeWidth(); x++) {
	        for (int y = 0; y < maze.getMazeHeight(); y++) {
	            Juncture current = junctures[x][y]; // The current juncture being processed.

	            // Check and add edges for each direction where there is no wall.
                // Adding an edge both ways between adjacent junctures, with the appropriate weight.
	            
	            if (!maze.isWallAbove(current) && y > 0) {
	                addEdge(current, junctures[x][y - 1], maze.getWeightAbove(current)); // Edge for no wall above.
	                addEdge(junctures[x][y - 1], current, maze.getWeightAbove(current));
	            }
	            if (!maze.isWallToRight(current) && x < maze.getMazeWidth() - 1) {
	                addEdge(current, junctures[x + 1][y], maze.getWeightToRight(current)); // Edge for no wall to the right.
	                addEdge(junctures[x + 1][y], current, maze.getWeightToRight(current));
	            } 
	            if (!maze.isWallBelow(current) && y < maze.getMazeHeight() - 1) {
	                addEdge(current, junctures[x][y + 1], maze.getWeightBelow(current)); // Edge for no wall below.
	                addEdge(junctures[x][y + 1], current, maze.getWeightBelow(current));
	            }
	            if (!maze.isWallToLeft(current) && x > 0) {
	                addEdge(current, junctures[x - 1][y], maze.getWeightToLeft(current)); // Edge for no wall to the left.
	                addEdge(junctures[x - 1][y], current, maze.getWeightToLeft(current));
	            }
	        }
	    }
	}
}
