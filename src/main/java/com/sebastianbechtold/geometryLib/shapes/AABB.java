package main.java.com.sebastianbechtold.geometryLib.shapes;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class AABB implements IShape {

	public Vector3D min = null;
	public Vector3D max = null;

	public AABB(Vector3D min, Vector3D max) {

		this.min = min;
		this.max = max;
	}

	@Override
	public AABB getAABB() {
		return this;
	}

	@Override
	public Vector3D getCentroid() {
		Vector3D size = getSize();

		return min.add(size.scalarMultiply(0.5));
	}

	
	
	public static AABB getForPrimitives(ArrayList<IShape> primitives) {

		double minX = Double.MAX_VALUE;
		double minY = Double.MAX_VALUE;
		double minZ = Double.MAX_VALUE;

		double maxX = Double.MIN_VALUE;
		double maxY = Double.MIN_VALUE;
		double maxZ = Double.MIN_VALUE;

		for (IShape p : primitives) {

			AABB box = p.getAABB();

			// Find minimum:
			if (box.min.getX() < minX) {
				minX = box.min.getX();
			}

			if (box.min.getY() < minY) {
				minY = box.min.getY();
			}

			if (box.min.getZ() < minZ) {
				minZ = box.min.getZ();
			}

			// Find maximum:
			if (box.max.getX() > maxX) {
				maxX = box.max.getX();
			}

			if (box.max.getY() > maxY) {
				maxY = box.max.getY();
			}

			if (box.max.getZ() > maxZ) {
				maxZ = box.max.getZ();
			}
		}

		Vector3D min = new Vector3D(minX, minY, minZ);
		Vector3D max = new Vector3D(maxX, maxY, maxZ);

		return new AABB(min, max);
	}

	
	
	


	@Override
	public double getIncidenceAngle_rad(Vector3D rayOrigin, Vector3D rayDir) {
		// TODO 5: Implement this
		return 0;
	}

	public double[] getRayIntersection(Vector3D orig, Vector3D dir) {
	
		// See http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
	
		Vector3D invdir = new Vector3D(1.0f / dir.getX(), 1.0f / dir.getY(), 1.0f / dir.getZ());
	
		int[] sign = { 0, 0, 0 };
	
		sign[0] = invdir.getX() < 0 ? 1 : 0;
		sign[1] = invdir.getY() < 0 ? 1 : 0;
		sign[2] = invdir.getZ() < 0 ? 1 : 0;
	
		double tmin, tmax, tymin, tymax, tzmin, tzmax;
	
		Vector3D[] bounds = { min, max };
	
		tmin = (bounds[sign[0]].getX() - orig.getX()) * invdir.getX();
		tmax = (bounds[1 - sign[0]].getX() - orig.getX()) * invdir.getX();
		tymin = (bounds[sign[1]].getY() - orig.getY()) * invdir.getY();
		tymax = (bounds[1 - sign[1]].getY() - orig.getY()) * invdir.getY();
	
		if (tmin > tymax || tymin > tmax)
			return null;
	
		if (tymin > tmin) {
			tmin = tymin;
		}
	
		if (tymax < tmax) {
			tmax = tymax;
		}
	
		tzmin = (bounds[sign[2]].getZ() - orig.getZ()) * invdir.getZ();
		tzmax = (bounds[1 - sign[2]].getZ() - orig.getZ()) * invdir.getZ();
	
		if (tmin > tzmax || tzmin > tmax)
			return null;
	
		if (tzmin > tmin) {
			tmin = tzmin;
		}
	
		if (tzmax < tmax) {
			tmax = tzmax;
		}
	
		double[] result = { tmin, tmax };
	
		return result;
	}

	public Vector3D getSize() {
		return max.subtract(min);
	}

	public String toString() {

		String result = "";

		result += "Min: " + min.toString() + ", Max: " + max.toString();
		return result;
	}

	
}
