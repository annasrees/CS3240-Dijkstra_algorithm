/**
 * Anna Rees
 * Algorithm Design and Analysis - Byung Lee
 * February 2024
 *
 * Write a program code to implement the Dijkstra’s single-source shortest paths finding algorithm discussed in class
 * and show the trace of the algorithm against the directed graph
 */

/**
 * base code:
 * Dijkstra(G,s) {
 *     Initialize Q of V with pi(v) <- infinity for each node v in V
 *     S <- {}, pi(s)<-0
 *     while(Q is not empty){
 *         v = ExtractMin(Q)
 *         add v to S
 *         for each edge e=(v,w) such that w is not in set S {
 *             if(pi(v) + l(e) < pi(w)){
 *                 pi(w) = pi(w) + l(e)
 *                 ChangeKey(Q, w, pi(w))
 *             }
 *         }
 *         endwhile
 *     }
 * }
 */

import java.util*;
import java.util.PriorityQueue;

public class Dijkstra{

//    initializing Dijkstra's alg w/ Edge class
    static class Edge {
        int destination;
        int weight;

        public Edge(int destination, int weight){
            this.destination = destination; //destination node
            this.weight = weight;
        }
    }

    static class Node{
        String ID; //node ID
        int shortestDist; //shortest distance from node
        int previous; //previous node
        list<Edge> edges; //list of edges

        public Node(int ID) {
            this.ID = ID;
            this.shortestDist = Integer.MAX_VALUE; //initialize to infinity (kinda)
            this.previous = -1 //no previous node initially
            this.edges = new ArrayList<>();
        }
    }

    static class Graph {
        List<Node> nodes;

        //      graph constructor
        public Graph(int numNodes) {
//            creating a list of all the nodes
            nodes = new ArrayList<>(numNodes);
            for (int i = 0; i < numNodes; i++) {
                nodes.add(new Node(i))
            }
        }

        public void addEdge(int sourceNode, int destinationNode, int weight) {
            //adding edge to list of edges
            nodes.get(sourceNode).edges.add(new Edge(destinationNode, weight));
        }

    }

    /**
     * creating PriorityQueueNode objects so the nodes can be compared and placed
     * in the queue correctly
     */
    static class PriorityQueueNode implements Comparable<PriorityQueueNode> {
        String nodeID;
        int dist;

        public PriorityQueueNode(String nodeID, int dist) {
            this.nodeID = nodeID;
            this.dist = dist;
        }

        //comparing 2 nodes to get the priority queue of which nodes are next to current node
        @Override
        public int compare(PriorityQueueNode otherNode) {
            return Integer.compare(this.distance, otherNode.distance)
        }
    }

    static void Dijkstra(Graph graph, int source) {
        /**
         * Dijkstra(G,s) {
         *  *     Initialize Q of V with pi(v) <- infinity for each node v in V
         *  *     S <- {}, pi(s)<-0
         *  *     while(Q is not empty){
         *  *         v = ExtractMin(Q)
         *  *         add v to S
         *  *         for each edge e=(v,w) such that w is not in set S {
         *  *             if(pi(v) + l(e) < pi(w)){
         *  *                 pi(w) = pi(w) + l(e)
         *  *                 ChangeKey(Q, w, pi(w))
         *  *             }
         *  *         }
         *  *         endwhile
         *  *     }
         *  * }
         */
        int numNodes = graph.nodes.size();
        PriorityQueue<Integer> priorityQueue = new PriorityQueue<>();
        //Initialize Q of V with pi(v) <- infinity for each node v in V
        for (Node node : graph.nodes) {
            //initializing the shortest distance and previous nodes
            node.shortestDist = Integer.MAX_VALUE;
            node.previous = -1
        }
        //first node in the list (starting node) is distance 0 from self
        //S <- {}, pi(s)<-0
        graph.nodes.get(sourceNode).shortestDist = 0;
        priorityQueue.add(new PriorityQueueNode(source, 0));

        //while(Q is not empty){
        while(!priorityQueue.isEmpty()) {
            //v = ExtractMin(Q)
            PriorityQueueNode minNode = priorityQueue.poll();
            Node minNode = graph.nodes.get(minNode.nodeID);

            //add v to S
            System.out.print("Node " + minNode.ID + " included in S with the shortest path length " + minNode.shortestDistance + " on the path s");
            int predecessor = minNode.previous;
            while(predecessor != -1) {
                System.out.print(" - " + predecessor);
                predecessor = graph.nodes.get(predecessor).previous;
            }
            System.out.println(" - " + minNode.ID + ".");

            //for each edge e=(v,w) such that w is not in set S {
            //  if(pi(v) + l(e) < pi(w)){
            //  pi(w) = pi(w) + l(e)
            //  ChangeKey(Q, w, pi(w))
            //  }
            // }
            for (Edge edge : minNode.edges) {
                Node next = graph.nodes.get(edge.destination);
                if (minNode.shortestDist + edge.weight < next.shortestDist) {
                    next.shortestDist = minNode.shortestDist + edge.weight;
                    next.previous = minNode.ID;
                    priorityQueue.add(new PriorityQueueNode(next.ID, next.shortestDist));
                }
            }
        }
    }

    public static void main(String[] args) {
        int numNodes = 8;
        Graph graph = new Graph(numNodes);
        //graph.addEdge(u ID, v ID, weight)
        graph.addEdge('s','2', 9);
        graph.addEdge('s','6', 14);
        graph.addEdge('s','7', 15);

        graph.addEdge('2','3', 23);

        graph.addEdge('6','3', 18);
        graph.addEdge('6','5', 30);
        graph.addEdge('6','7', 5);

        graph.addEdge('7','5', 20);
        graph.addEdge('7','t', 44);

        graph.addEdge('3','t', 19);
        graph.addEdge('3','5', 2);

        graph.addEdge('5','4', 11);
        graph.addEdge('5','t', 16);

        graph.addEdge('4','3', 6);
        graph.addEdge('4','t', 6);

        //Run algorithm
        System.out.println("Graph is represented using an adjacency list.");
        Dijkstra(graph, 's'); // Source node is 's'

    }
}

