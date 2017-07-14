package main.java.com.sebastianbechtold.geometryLib.shapes;

import java.util.Arrays;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Triangle implements IShape {

	public final Vertex[] mVerts = new Vertex[3];

	Vector3D mFaceNormal = new Vector3D(0, 0, 0);

	// TODO 3: Make triangle immutable
	public Triangle() {

	}

	public Triangle(Vertex v0, Vertex v1, Vertex v2) {

		mVerts[0] = v0;
		mVerts[1] = v1;
		mVerts[2] = v2;

		update();
	}

	@Override
	public AABB getAABB() {
		double minX = Math.min(Math.min(mVerts[0].getX(), mVerts[1].getX()), mVerts[2].getX());
		double minY = Math.min(Math.min(mVerts[0].getY(), mVerts[1].getY()), mVerts[2].getY());
		double minZ = Math.min(Math.min(mVerts[0].getZ(), mVerts[1].getZ()), mVerts[2].getZ());

		Vector3D min = new Vector3D(minX, minY, minZ);

		double maxX = Math.max(Math.max(mVerts[0].getX(), mVerts[1].getX()), mVerts[2].getX());
		double maxY = Math.max(Math.max(mVerts[0].getY(), mVerts[1].getY()), mVerts[2].getY());
		double maxZ = Math.max(Math.max(mVerts[0].getZ(), mVerts[1].getZ()), mVerts[2].getZ());

		Vector3D max = new Vector3D(maxX, maxY, maxZ);

		return new AABB(min, max);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((mFaceNormal == null) ? 0 : mFaceNormal.hashCode());
		result = prime * result + Arrays.hashCode(mVerts);
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Triangle other = (Triangle) obj;
		if (mFaceNormal == null) {
			if (other.mFaceNormal != null)
				return false;
		} else if (!mFaceNormal.equals(other.mFaceNormal))
			return false;
		if (!Arrays.equals(mVerts, other.mVerts))
			return false;
		return true;
	}

	@Override
	public Vector3D getCentroid() {

		return (mVerts[0].pos.add(mVerts[1].pos).add(mVerts[2].pos)).scalarMultiply(1.0 / 3);
	}

	public Vector3D getFaceNormal() {

		if (this.mFaceNormal == null) {
			this.update();
		}

		return this.mFaceNormal;
	}

	@Override
	public double getIncidenceAngle_rad(Vector3D rayOrigin, Vector3D rayDir) {

		// TODO 1: Fix this in HELIOS
		return Vector3D.angle(getFaceNormal(), rayDir);
	}

	@Override
	public double[] getRayIntersection(Vector3D rayOrigin, Vector3D rayDir) {

		// See http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
		double[] result = new double[1];

		Vector3D e1 = this.mVerts[1].pos.subtract(this.mVerts[0].pos);
		Vector3D e2 = this.mVerts[2].pos.subtract(this.mVerts[0].pos);

		Vector3D h = rayDir.crossProduct(e2);

		// ###### BEGIN Early abort if ray runs parallel to the triangle's face ########
		double a = e1.dotProduct(h);

		if (a > -0.00001 && a < 0.00001) {
			result[0] = -1;
			return result;
		}
		// ###### END Early abort if ray runs parallel to the triangle's face ########

		double f = 1.0 / a;

		Vector3D s = rayOrigin.subtract(this.mVerts[0].pos);

		// Compute U:
		double u = f * s.dotProduct(h);

		if (u < 0.0 || u > 1.0) {
			result[0] = -1;
			return result;
		}

		Vector3D q = s.crossProduct(e1);

		// Compute v:
		double v = f * rayDir.dotProduct(q);

		if (v < 0.0 || u + v > 1.0) {
			result[0] = -1;
			return result;
		}

		// Compute t (distance from ray origin to intersection):
		double t = f * e2.dotProduct(q);

		if (t < 0.00001) {
			result[0] = -1;
			return result;			
		}

		
		result[0] = t;
		return result;
	}

	public void update() {

		Vector3D normal_unnormalized = (mVerts[1].pos.subtract(mVerts[0].pos)).crossProduct(mVerts[2].pos.subtract(mVerts[0].pos));

		if (normal_unnormalized.getNorm() > 0) {
			this.mFaceNormal = normal_unnormalized.normalize();
		}
	}

	public void setAllVertexNormalsFromFace() {
		mVerts[0].normal = mFaceNormal;
		mVerts[1].normal = mFaceNormal;
		mVerts[2].normal = mFaceNormal;
	}
}
