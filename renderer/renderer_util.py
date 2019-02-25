import numpy as np

from collections import deque
from itertools import izip


#-------------------------------------------------------------------------------

# v: Nx3 set of vertices
# f: Nx3 set of vertex indices
def compute_vertex_normals(v, f):
    # compute face normals (cross product of face edges)
    face_normals = np.zeros(f.shape, dtype=np.float32)
    face_normals[:] = np.cross(v[f[:,1]] - v[f[:,0]], v[f[:,2]] - v[f[:,0]])
    face_normals /= np.linalg.norm(face_normals, axis=1)[:,np.newaxis]

    # compute vertex normals
    # TODO (True): for now, just using simple averaging
    n = np.zeros_like(v)

    for i in xrange(3):
        np.add.at(n, f[:,i], face_normals)

    n_len = np.linalg.norm(n, axis=1)
    mask = (n_len > 0)
    n[mask] /= n_len[mask,np.newaxis]
                    
    return n


#-------------------------------------------------------------------------------

# given an ordering for a set of vertices, return an Nx3 array of triangles that
# cover the polygon
# returns:
# - polygon_idxs: Nx3 array of indices such that polygon[polygon_idxs] gives the
#   actual Nx3 triangle set of indices into vertices
def convert_polygon_to_triangles(vertices, polygon):
    if len(polygon) == 3:
        return polygon

    polygon_idxs = []

    vi = vertices[polygon]

    # compute 2x the signed area of the polygon (assuming a planar polygon);
    # this will allow us to determine the appropriate sign of the polygon normal
    # that gives a CCW winding order
    n = np.cross(vi[1:-1] - vi[0], vi[2:] - vi[0])
    n0 = n[0] / np.linalg.norm(n[0])
    area = np.sum(n, axis=0).dot(n0)

    if area < 0:
        n0 *= -1.

    # iteratively add triangles
    prev_ = 0 # previous index
    idx_queue = deque(xrange(1, len(polygon)))
    while len(idx_queue) >= 2:
        # ensure the triangle is not exterior, and that no polygon
        # vertices intersect with it
        idxs = (prev_, idx_queue[0], idx_queue[1])
        a, b, c = (vi[idx] for idx in idxs)
        ab = a - b
        cb = c - b

        if np.cross(cb, ab).dot(n0) > 0:
            # https://stackoverflow.com/a/2049712
            vb = vi - b
            st = np.linalg.lstsq(np.column_stack((ab, cb)), vb.T)[0]
            st[:,idxs] = -1.
            any_inside = np.any(
                np.all(st >= 0, axis=0) & (np.sum(st, axis=0) <= 1))
            if any_inside:
                idx_queue.append(prev_)
                prev_ = idx_queue.popleft()
            else:
                polygon_idxs.append(idxs)
                idx_queue.popleft()
        else:
            idx_queue.append(prev_)
            prev_ = idx_queue.popleft()

    return np.array(polygon_idxs)
