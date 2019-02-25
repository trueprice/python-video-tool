# TODO (True): currently only supporting one texture and no materials

import panda3d as p3d
import numpy as np
import os

from renderer_util import compute_vertex_normals, convert_polygon_to_triangles


#-------------------------------------------------------------------------------

class OBJNode(p3d.core.GeomNode):
    # TODO (True): large point clouds will overrun the buffer; so, we'll have to
    # split up into smaller
    MAX_NUM_VERTICES = 2**32

    def __init__(self, obj_file, name=""):
        super(OBJNode, self).__init__(name)

        vertices = []
        texcoord = []
        normals = []
        faces = []
        face_texcoord = []
        face_normals = []

        self.mtl_file = None

        with open(obj_file, "r") as fid:
            for line in fid:
                if line.startswith("v "):
                    vertices.append(map(float, line.split()[1:]))

                elif line.startswith("vt "):
                    texcoord.append(map(float, line.split()[1:]))

                elif line.startswith("vn "):
                    normals.append(map(float, line.split()[1:]))

                elif line.startswith("f "):
                    fi, ft, fn = [], [], []
                    for entry in line.split()[1:]:
                        data = entry.split("/")
                        fi.append(int(data[0]) - 1)

                        if len(data) > 1:
                            if data[1]:
                                ft.append(int(data[1]) - 1)
                            if len(data) == 3 and data[2]:
                                fn.append(int(data[2]) - 1)

                    # append face data in a way that supports polygons
                    # NOTE (True): this assumes all used vertices have been
                    # previously defined
                    fi = np.array(fi)
                    idxs = convert_polygon_to_triangles(vertices, fi)
                    faces.append(fi[idxs])

                    if ft:
                        face_texcoord.append(np.array(ft)[idxs])
                    if fn:
                        face_normals.append(np.array(fn)[idxs])

                elif line.startswith("mtllib "):
                    self.mtl_file = line.split()[1]

        vertices = np.array(vertices)

        self.has_texture = (len(texcoord) > 0)
        if self.has_texture:
            texcoord = np.array(texcoord)

        self.has_normals = (len(normals) > 0)
        if self.has_normals:
            normals = np.array(normals)

        self.has_faces = (len(faces) > 0)
        if self.has_faces:
            faces = np.row_stack(faces)

        self.has_face_texture = (len(face_texcoord) > 0)
        if self.has_face_texture:
            face_texcoord = np.row_stack(face_texcoord)

        self.has_face_normals = (len(face_normals) > 0)
        if self.has_face_normals:
            face_normals = np.row_stack(face_normals)

        if self.has_faces:
            # set up vertex normals from faces
            if not self.has_normals:
                normals = compute_vertex_normals(vertices, faces)
                self.has_normals = True

            # if the faces have their own textures or normals, we'll have to
            # duplicate all the vertices
            if self.has_face_texture or self.has_face_normals:
                vertices = vertices[faces].reshape(-1, 3)
                faces = np.arange(len(vertices)).reshape(-1, 3)

                if self.has_face_texture:
                    texcoord = texcoord[face_texcoord].reshape(-1, 2)

                if self.has_face_normals:
                    normals = normals[face_normals].reshape(-1, 3)


        # set up data in chunks
        for i in xrange(0, len(vertices), OBJNode.MAX_NUM_VERTICES):
            stop = min(i + OBJNode.MAX_NUM_VERTICES, len(vertices))
            n = stop - i

            if self.has_texture and self.has_normals:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3n3t2()
            elif self.has_texture:
                p3d_data_format = p3d.core.GeomVertexFormat().getV3t2()
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
            if self.has_texture:
                t_writer = p3d.core.GeomVertexWriter(p3d_data, "texcoord") 
                for coord in texcoord[i:stop]:
                    t_writer.addData2f(coord[0], coord[1])

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

#-------------------------------------------------------------------------------

class OBJNodePath(p3d.core.NodePath):
    def __init__(self, obj_file):
        self.geom_node = OBJNode(obj_file)
        super(OBJNodePath, self).__init__(self.geom_node)

        folder = os.path.dirname(obj_file)

        if self.geom_node.mtl_file is not None:
            mtl_file = os.path.join(folder, self.geom_node.mtl_file)
            with open(mtl_file, "r") as fid:
                for line in fid:
                    if line.startswith("map_Kd"):
                        texture_file = os.path.join(folder, line.split()[1])
                        texture = p3d.core.Texture()
                        texture.read(texture_file)
                        self.setTexture(texture)
