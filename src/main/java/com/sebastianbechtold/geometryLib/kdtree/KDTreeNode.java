package main.java.com.sebastianbechtold.geometryLib.kdtree;

import java.util.ArrayList;
import java.util.Comparator;

import main.java.com.sebastianbechtold.geometryLib.shapes.AABB;
import main.java.com.sebastianbechtold.geometryLib.shapes.IShape;

public class KDTreeNode {

	KDTreeNode left = null;
	KDTreeNode right = null;

	double splitPos = 0;
	int splitAxis = 0;

	static int stats_maxNumPrimsInLeaf = 0;
	static int stats_minNumPrimsInLeaf = Integer.MAX_VALUE;
	static int stats_maxDepthReached = 0;

	ArrayList<IShape> primitives = null;

	private static class KDTreePrimitiveComparator implements Comparator<IShape> {

		int axis = 0;

		public KDTreePrimitiveComparator(int axis) {
			this.axis = axis;
		}

		@Override
		public int compare(IShape a, IShape b) {

			double[] centroid_a = a.getCentroid().toArray();
			double[] centroid_b = b.getCentroid().toArray();

			if (centroid_a[axis] < centroid_b[axis]) {
				return -1;
			} else if (centroid_a[axis] > centroid_b[axis]) {
				return 1;
			}

			return 0;
		}
	}

	public static KDTreeNode build(ArrayList<IShape> primitives) {

		KDTreeNode root = buildRecursive(primitives, 0);

		/*
		 * System.out.println("\n"); System.out.println("Max. # primitives in leaf: " + stats_maxNumPrimsInLeaf);
		 * System.out.println("Min. # primitives in leaf: " + stats_minNumPrimsInLeaf); System.out.println("Max. depth reached: : " + stats_maxDepthReached);
		 */
		return root;
	}

	private static KDTreeNode buildRecursive(ArrayList<IShape> primitives, int depth) {

		int primsSize = primitives.size();

		if (primsSize == 0) {
			return null;
		}

		// Update maximum reached depth:
		if (depth > stats_maxDepthReached) {
			stats_maxDepthReached = depth;
		}

		KDTreeNode node = new KDTreeNode();

		// TODO 5: Implement surface area heuristics?
		int splitAxis = depth % 3;

		// Sort faces along split axis:
		// ATTENTION: Sorting must happen *BEFORE* splitPos is selecty using the median!!

		// Sort primitives along split axis:
		Comparator<IShape> comparator = new KDTreePrimitiveComparator(splitAxis);
		java.util.Collections.sort(primitives, comparator);

		// Compute split position:
		double splitPos = primitives.get(primsSize / 2).getCentroid().toArray()[splitAxis];

		// ########## BEGIN Fill children's primitive lists ##########

		ArrayList<IShape> sublist_left = new ArrayList<>();
		ArrayList<IShape> sublist_right = new ArrayList<>();

		for (IShape t : primitives) {

			AABB box = t.getAABB();

			if (box.min.toArray()[splitAxis] <= splitPos) {
				sublist_left.add(t);
			}

			if (box.max.toArray()[splitAxis] > splitPos) {
				sublist_right.add(t);
			}
		}

		if (sublist_left.size() != primsSize && sublist_right.size() != primsSize) {

			node.splitAxis = splitAxis;
			node.splitPos = splitPos;

			if (sublist_left.size() > 0) {
				node.left = buildRecursive(sublist_left, depth + 1);
			}

			if (sublist_right.size() > 0) {
				node.right = buildRecursive(sublist_right, depth + 1);
			}
		} else {
			// Otherwise, make this node a leaf:
			node.splitAxis = -1;
			node.primitives = primitives;
		}

		return node;

	}
}
