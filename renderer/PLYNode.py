import panda3d as p3d
import numpy as np

from itertools import izip
from plyfile import PlyData, PlyElement, make2d as PlyMake2D # pip install plyfile
from renderer_util import compute_vertex_normals

class PLYNode(p3d.core.GeomNode):
    # TODO (True): large point clouds will overrun the buffer; so, we'll have to
    # split up into smaller
    MAX_NUM_VERTICES = 2**32

    def __init__(self, ply_file, name=""):
        super(PLYNode, self).__init__(name)

        # load mesh
        mesh = PlyData.read(ply_file)
        v = mesh["vertex"] # v is an easier-to-read reference to the vertex data

        self.has_faces = ("face" in mesh and mesh["face"].count > 0)

        vertices = np.column_stack((v["x"], v["y"], v["z"]))

        # TODO (True): couldn't find a better way to check these...
        self.has_colors = True
        try:
            v["red"]; v["green"]; v["blue"]
            INV_255 = 1. / 255.
            colors = \
                np.column_stack((v["red"], v["green"], v["blue"])) * INV_255
        except:
            self.has_colors = False
        
        self.has_normals = True
        try:
            v["nx"]; v["ny"]; v["nz"]
            normals = np.column_stack((v["nx"], v["ny"], v["nz"]))
        except:
            self.has_normals = False

        if self.has_faces:
            #faces = np.array([f[0] for f in mesh["face"]])
            faces = PlyMake2D(mesh["face"].data["vertex_indices"])

            # set up vertex normals from faces
            if not self.has_normals:
                normals = compute_vertex_normals(vertices, faces)
                self.has_normals = True

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

            # load vertex positions
            v_writer = p3d.core.GeomVertexWriter(p3d_data, "vertex") 
            for vertex in vertices[i:stop]:
                v_writer.addData3f(*vertex)

            # load colors
            if self.has_colors:
                c_writer = p3d.core.GeomVertexWriter(p3d_data, "color") 
                for color in colors[i:stop]:
                    c_writer.addData4f(color[0], color[1], color[2], 1.)

            # load normals
            if self.has_normals:
                n_writer = p3d.core.GeomVertexWriter(p3d_data, "normal") 
                for normal in normals[i:stop]:
                    n_writer.addData3f(*normal)

            # add faces, if available
            if self.has_faces:
                p3d_primitives = p3d.core.GeomTriangles(p3d.core.Geom.UHStatic)
                mask = np.all(faces >= i, axis=1) & np.all(faces < stop, axis=1)
                for f in faces[mask]:
                    p3d_primitives.addVertices(*(f - i))

            # otherwise, render a point cloud
            else:
                p3d_primitives = p3d.core.GeomPoints(p3d.core.Geom.UHStatic)
                p3d_primitives.addNextVertices(n)

            geom = p3d.core.Geom(p3d_data)
            geom.addPrimitive(p3d_primitives)
            self.addGeom(geom)

