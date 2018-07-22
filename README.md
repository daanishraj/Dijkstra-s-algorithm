
*****Introduction:

Given two points on a graph, the shortest path between them is the straight line that joins them. However, drawing a straight line may not always be possible. For example, in a road network, straight roads between two cities seldom exist. In such a case, there might be multiple routes between the two cities, and the challenge then is to find the shortest one.

Dijkstra’s algorithm helps us find the shortest path between a source node and every other node in a graph.

The algorithm is ubiquitous. For example, it is used in computer networking where it ascertains the shortest path between the source router and other routers in the network. Also, each time we are navigating via google maps, complex algorithms based on Dijkstra are being used to provide us with the best route. 


****Implementing Dijkstra’s algorithm

In this short project, we implement Dijkstra’s algorithm. We have been provided with data and some code for a GUI, which generates a map of cities in the US and the different connections between them. Our goal is to compute for the user, the shortest path between any two cities they select on the map.


****Files submitted and respective descriptions


---Dijkstra.java - This is a file which contains the code for implementing Dijkstra’s shortest path algorithm.

---Display.java, Edge.java, Vertex.java - These files were provided to us with some existing code. We need these to create instances of the vertex object, edge object and implement relevant methods in the Dijkstra.java file. The Display.java file is required to run the GUI. NO CHANGES HAVE BEEN MADE TO THESE files.

---citypairs.txt, cityxy.txt - These are files provided to us with the data. NO CHANGES HAVE BEEN MADE TO THESE files. The cityxy.txt file contains a list of cities and their (X,Y) coordinates on the map. These are the vertices in the graph. The citypairs.txt lists direct, connections between pairs of cities. These links are bidirectional, if you can go from NewYork to Boston, you can get from Boston to New York. These are the edges of the graph.


*****Instructions to execute the program*******
Go the command line and navigate to the local directory which has all the files in this repository. Now, compile all java sources in the directory by typing javac 'filename'.java. Then run Display.java by typing java Display. This will launch the GUI. Now click on the appropriate buttons to test the corresponding methods. 





