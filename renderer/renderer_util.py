import numpy as np

from itertools import izip

# v: Nx3 set of vertices
# f: Nx3 set of vertex indices
def compute_vertex_normals(v, f):
    # compute face normals (cross product of face edges)
    face_normals = np.zeros(f.shape, dtype=np.float32)
    face_normals[:] = np.cross(v[f[:,1]] - v[f[:,0]], v[f[:,2]] - v[f[:,0]])
    face_normals /= np.linalg.norm(face_normals, axis=1)[:,np.newaxis]

    # compute vertex normals
    # TODO (True): for now, just using simple averaging in a for loop
    n = np.zeros_like(v)

    for face, fn in izip(f, face_normals):
        n[face[0]] += fn
        n[face[1]] += fn
        n[face[2]] += fn

    n_len = np.linalg.norm(n, axis=1)
    mask = (n_len > 0)
    n[mask] /= n_len[mask,np.newaxis]
                    
    return n
