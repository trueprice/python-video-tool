""" This is definitely not a complete X3D parser implementation; it's only
    currently meant to serve the purposes of this project """

import panda3d as p3d
import numpy as np
import xml.etree.ElementTree as ET

from itertools import izip
from renderer_util import compute_vertex_normals

#-------------------------------------------------------------------------------

# returns the cross product matrix representation of a 3-vector v
def cross_prod_matrix(v):
    return np.array(((0., -v[2], v[1]), (v[2], 0., -v[0]), (-v[1], v[0], 0.)))

# www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
# axis_angle: (x, y, z, theta)
# note this assumes a unit vector for the axis
def axis_angle_to_rotation_matrix(axis_angle):
    angle = axis_angle[3]
    cp_axis = cross_prod_matrix(axis_angle[:3])
    return np.eye(3) + (
        np.sin(angle) * cp_axis + (1. - np.cos(angle)) * cp_axis.dot(cp_axis))

#-------------------------------------------------------------------------------
#
# X3DNodePath
#
#-------------------------------------------------------------------------------

class X3DNodePath(p3d.core.NodePath):
    def __init__(self, x3d_file):
        super(X3DNodePath, self).__init__(x3d_file)

        #
        # load mesh
        #

        root = ET.parse(x3d_file).getroot()

        # assuming at most one scene node
        while root.tag != "Scene":
            root = root[0]

        # mesh_id => p3d.core.NodePath
        self.mesh_nps = dict()

        for child in root:
            if child.tag == "IndexedTriangleSet":
                # TODO (True): in the future, support texture coordinates
                assert(child[0].tag == "Coordinate" and len(child) == 1)

                mesh_id = child.attrib["DEF"] if "DEF" in child.attrib else \
                          len(meshes)

                vertices = np.fromstring(
                    child[0].attrib["point"], sep=" ").reshape(-1, 3)
                faces = np.fromstring(
                        child.attrib["index"], sep=" ", dtype=np.uint32) \
                    .reshape(-1, 3)

                p3d_data_format = p3d.core.GeomVertexFormat().getV3n3()
                p3d_data = p3d.core.GeomVertexData(
                    mesh_id, p3d_data_format, p3d.core.Geom.UHStatic)
                p3d_data.setNumRows(len(vertices))

                # load vertices
                v_writer = p3d.core.GeomVertexWriter(p3d_data, "vertex") 
                for vertex in vertices:
                    v_writer.addData3f(*vertex)

                # compute and load normals
                normals = compute_vertex_normals(vertices, faces)
                n_writer = p3d.core.GeomVertexWriter(p3d_data, "normal") 
                for normal in normals:
                    n_writer.addData3f(*normal)

                # load faces
                p3d_primitives = p3d.core.GeomTriangles(p3d.core.Geom.UHStatic)
                for f in faces:
                    p3d_primitives.addVertices(*f)

                geom = p3d.core.Geom(p3d_data)
                geom.addPrimitive(p3d_primitives)

                mesh_node = p3d.core.GeomNode(mesh_id)
                mesh_node.addGeom(geom)

                self.mesh_nps[mesh_id] = p3d.core.NodePath(mesh_node)

            # this assumes that there is one global transformation
            elif child.tag == "Transform":
                self._addSceneElements(child)
                self.setMat(self._createTransformationMatrix(**child.attrib))

    # this assumes a very strict format for the children:
    # <Transform>
    #   <Shape><IndexedTriangleSet/><Appearance><Material/></Appearance></Shape>
    #   ...
    def _addSceneElements(self, root_transformation):
        for idx, child in enumerate(root_transformation):
            instance_np = self.attachNewNode("instance_%i" % idx)
            instance_np.setMat(self._createTransformationMatrix(**child.attrib))

            for shape in child:
                mesh_id = shape[0].attrib["USE"]
                part_np = instance_np.attachNewNode(mesh_id)
                self.mesh_nps[mesh_id].instanceTo(part_np)

                # load material
                rgb = np.fromstring(shape[1][0].attrib["diffuseColor"], sep=" ")
                material = p3d.core.Material()
                #material.setAmbient((0, 0, 0, 1)) # has to be set?
                #material.setDiffuse(tuple(rgb) + (1.,))
                material.setAmbient(tuple(rgb) + (1.,))
                material.setDiffuse(tuple(rgb) + (1.,))
                material.setLocal(True)
                part_np.setMaterial(material)

    # **transform (all optional): "translation", "rotation", "scale"
    def _createTransformationMatrix(self, **transform):
        out_matrix = np.eye(4)
        if "rotation" in transform:
            axis_angle = np.fromstring(transform["rotation"], sep=" ")
            out_matrix[:3,:3] = axis_angle_to_rotation_matrix(axis_angle).T
        if "scale" in transform:
            S = np.diag(np.fromstring(transform["scale"], sep=" "))
            out_matrix[:3,:3] = S.dot(out_matrix[:3, :3])
        if "translation" in transform:
            out_matrix[3,:3] = np.fromstring(transform["translation"], sep=" ")

        # return as the panda3d type
        return p3d.core.Mat4(*out_matrix.flatten())
