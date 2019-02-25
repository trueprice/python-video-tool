import panda3d as p3d
import numpy as np

from itertools import izip
from plyfile import PlyData, PlyElement, make2d as PlyMake2D # pip install plyfile
from renderer_util import compute_vertex_normals

class PLYNode(p3d.core.GeomNode):
    # TODO (True): large point clouds will overrun the buffer; so, we'll have to
    # split up into smaller
    MAX_NUM_VERTICES = 2**32

    DEFAULT_MESH_COLOR = 180 # grayscale value

    def __init__(self, ply_file, name=""):
        super(PLYNode, self).__init__(name)

        # load mesh
        mesh = PlyData.read(ply_file)
        v = mesh["vertex"] # v is an easier-to-read reference to the vertex data

        self.has_faces = ("face" in mesh and mesh["face"].count > 0)

        vertices = np.column_stack(
            [v["x"].astype(np.float32, copy=False),
             v["y"].astype(np.float32, copy=False),
             v["z"].astype(np.float32, copy=False)])

        vertex_data = []

        self.has_normals = all(
            x in v._property_lookup for x in ["nx", "ny", "nz"])
        if self.has_normals:
            vertex_data += [
                v["nx"].astype(np.float32, copy=False),
                v["ny"].astype(np.float32, copy=False),
                v["nz"].astype(np.float32, copy=False)]

        if self.has_faces:
            #faces = np.array([f[0] for f in mesh["face"]])
            faces = PlyMake2D(mesh["face"].data["vertex_indices"])

            # set up vertex normals from faces
            if not self.has_normals:
                vertex_data.append(compute_vertex_normals(vertices, faces))
                self.has_normals = True

        self.has_colors = all(
            x in v._property_lookup for x in ["red", "green", "blue"])
        if self.has_colors:
            colors = np.empty((len(vertices), 4), dtype=np.uint8)
            # TODO (True): maybe check for all integer types?
            if v["red"].dtype == np.uint8:
                colors[:,0] = v["red"]
                colors[:,1] = v["green"]
                colors[:,2] = v["blue"]
            else:
                colors[:,0] = v["red"] * 255.
                colors[:,1] = v["green"] * 255.
                colors[:,2] = v["blue"] * 255.

            colors[:,3] = 255

            # alias the color uint8 values as a single float32 for convience
            vertex_data.append(colors.view(np.float32))
        elif self.has_faces:
            # draw a colorless mesh as gray
            self.has_colors = True
            colors = np.empty((len(vertices), 4), dtype=np.uint8)
            colors[:] = np.array(
                (DEFAULT_MESH_COLOR,) * 3 + (255,), dtype=np.uint8)
            vertex_data.append(colors.view(np.float32))
        
        if vertex_data:
            vertices = np.column_stack([vertices] + vertex_data)

        # set up data in chunks
        for i in xrange(0, len(vertices), PLYNode.MAX_NUM_VERTICES):
            stop = min(i + PLYNode.MAX_NUM_VERTICES, len(vertices))
            n = stop - i

            if self.has_colors and self.has_normals:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3n3c4()
            elif self.has_colors:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3c4()
            elif self.has_normals:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3n3()
            else:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3()

            p3d_data = p3d.core.GeomVertexData(
                name, p3d_data_format, p3d.core.Geom.UHStatic)
            p3d_data.setNumRows(n)

            p3d_data.modifyArray(0).modifyHandle().setData(
                vertices[i:stop].tostring())

            # add faces, if available
            if self.has_faces:
                p3d_primitives = p3d.core.GeomTriangles(p3d.core.Geom.UHStatic)
                p3d_primitives.setIndexType(p3d.core.GeomEnums.NTUint32)
                mask = np.all(faces >= i, axis=1) & np.all(faces < stop, axis=1)
                p3d_primitives.modifyVertices().modifyHandle().setData(
                    faces[mask].tostring())

            # otherwise, render a point cloud
            else:
                p3d_primitives = p3d.core.GeomPoints(p3d.core.Geom.UHStatic)
                p3d_primitives.addNextVertices(n)

            geom = p3d.core.Geom(p3d_data)
            geom.addPrimitive(p3d_primitives)
            self.addGeom(geom)

