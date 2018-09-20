import numpy as np
import os
import sys

import imageio # or trade out with your favorite alternative image writer

if sys.version_info >= (3, 0):
    import tkinter as tk
else:
    import Tkinter as tk

import panda3d as p3d
import panda3d.core
from direct.showbase.ShowBase import ShowBase

from itertools import izip
from renderer import CameraTrackEditingWidget, PLYNode, X3DNodePath
from renderer.rotation import Quaternion


#-------------------------------------------------------------------------------
#
# VideoTool
#
#-------------------------------------------------------------------------------

class VideoTool(ShowBase):
    VIEW_RADIUS_MIN = 0.001
    VIEW_RADIUS_MAX = 20.
    VIEW_RADIUS_MULT = 1.1
    INV_VIEW_RADIUS_MULT = 1. / VIEW_RADIUS_MULT

    ROT_MULT = 2.2
    XY_TRANS_MULT = 1.8
    Z_TRANS_MULT = 1.8

    BACKGROUND_COLOR = (0, 0, 0) # RGB \in [0,255]
    #BACKGROUND_COLOR = (135, 206, 235) # RGB \in [0,255]

    # Panda3d has this weird "look down the +y axis" convention; when setting
    # camera pose, we'll right-multiply by VIEW_QUAT to get the camera looking
    # down the +z axis, with +y down.
    VIEW_QUAT = Quaternion((1. / np.sqrt(2.), 1. / np.sqrt(2.), 0., 0.))

    def __init__(self, args):
        ShowBase.__init__(self)
        base.startTk()

        self.starting_frame = args.starting_frame
        self.stopping_frame = args.stopping_frame

        self.window_size = args.window_size
        self.fps = args.fps

        base.disableMouse()
        c = np.array(VideoTool.BACKGROUND_COLOR) / 255.
        base.setBackgroundColor(*c)

        self.editing_widget = CameraTrackEditingWidget(
            base.tkRoot, self.fps, self.starting_frame)
        self.editing_widget.load_asset_callback = self.addMesh
        self.editing_widget.remove_asset_callback = self.removeMesh
        self.editing_widget.remove_all_assets_callback = self.removeAllMeshes
        self.editing_widget.keyframe_activation_callback = self.setCameraView
        self.editing_widget.get_view_params = lambda: \
            (self.camera_rot, self.camera_center, self.view_radius)

        self.editing_widget.change_ambient_callback = self.updateAmbient
        self.editing_widget.change_directional_callback = self.updateDirectional

        self.editing_widget.start_recording_callback = self.startRecording
        self.editing_widget.stop_recording_callback = self.stopRecording

        self.write_frame_complete = False

        # loaded meshes
        self.meshes = []

        # lights
        dlight1 = p3d.core.DirectionalLight('dlight1')
        dlight1.setColor(p3d.core.VBase4(1, 1, 1, 1))
        self.dlight1_node = render.attachNewNode(dlight1)

        #dlight2 = p3d.core.DirectionalLight('dlight2')
        #dlight2.setColor(p3d.core.VBase4(1, 1, 1, 1))
        #self.dlight2_node = render.attachNewNode(dlight2)
        #self.dlight2_node.setHpr(180, 0, 0)

        alight = p3d.core.AmbientLight('alight')
        alight.setColor(p3d.core.VBase4(1, 1, 1, 1))
        self.alight_node = render.attachNewNode(alight)

        # camera lens
        lens = self.camNode.getLens()
        lens.setNear(1e-3)
        lens.setFar(200.)

        # camera controls
        self.camera_center = np.array((0., -4., 0.))
        self.camera_rot = Quaternion()
        self.view_radius = 1.

        self.start_new_mouse_action = True
        self.do_rotate = False
        self.do_xy_trans = False
        self.do_z_trans = False

        self.accept("mouse1", self.startMouseMove)
        self.accept("mouse1-up", self.stopMouseMove)
        self.accept("mouse2", self.startZTrans)
        self.accept("mouse2-up", self.stopZTrans)
        self.accept("mouse3", self.startXYTrans)
        self.accept("mouse3-up", self.stopXYTrans)
        self.accept("wheel_up", self.decreaseViewRadius)
        self.accept("wheel_down", self.increaseViewRadius)

        self.accept("space", self.addNewCameraPosition)
        self.accept("a", self.insertNewCameraPosition)
        self.accept("d", self.deleteCameraPosition)
        self.accept("m", self.modifySelectedCameraPosition)

        taskMgr.add(self.cameraMoveTask, "cameraMoveTask")

        self.exit_when_recording_done = False

        if args.input_file is not None:
            print "Loading", args.input_file
            self.editing_widget.loadTrack(args.input_file)

            if args.output_folder is not None:
                print "Rendering to", args.output_folder
                self.editing_widget.toggleRecording(args.output_folder)
                self.exit_when_recording_done = True

    #---------------------------------------------------------------------------

    def addMesh(self, filepath):
        if filepath.endswith(".ply"):
            self.meshes.append(render.attachNewNode(PLYNode(filepath)))
        elif filepath.endswith(".x3d"):
            self.meshes.append(X3DNodePath(filepath))
            self.meshes[-1].reparentTo(render)

        # set up lights for the mesh
        self.meshes[-1].setLight(self.dlight1_node)
        #self.meshes[-1].setLight(self.dlight2_node)
        self.meshes[-1].setLight(self.alight_node)

        self.meshes[-1].setAntialias(p3d.core.AntialiasAttrib.MMultisample)

        self._update_camera_view()

    def removeMesh(self, index):
        self.meshes.pop(index).removeNode()

    def removeAllMeshes(self):
        map(lambda node_path: node_path.removeNode(), self.meshes)
        self.meshes = []

    #---------------------------------------------------------------------------

    def startMouseMove(self):
        self.do_rotate = True
        self.start_new_mouse_action = True

    def stopMouseMove(self):
        self.do_rotate = False

    def startZTrans(self):
        self.do_z_trans = True
        self.start_new_mouse_action = True

    def stopZTrans(self):
        self.do_z_trans = False

    def startXYTrans(self):
        self.do_xy_trans = True
        self.start_new_mouse_action = True

    def stopXYTrans(self):
        self.do_xy_trans = False

    def decreaseViewRadius(self):
        new_view_radius = self.view_radius * VideoTool.INV_VIEW_RADIUS_MULT
        if new_view_radius >= VideoTool.VIEW_RADIUS_MIN:
            self.view_radius = new_view_radius

    def increaseViewRadius(self):
        new_view_radius = self.view_radius * VideoTool.VIEW_RADIUS_MULT
        if new_view_radius <= VideoTool.VIEW_RADIUS_MAX:
            self.view_radius = new_view_radius

    #---------------------------------------------------------------------------

    def _update_camera_view(self):
        # note that the y-axis is the viewing direction in Panda3d
        C = -self.camera_rot.ToR()[:,1] * self.view_radius + self.camera_center
        self.camera.setPos(*C)
        q = p3d.core.Quat(*self.camera_rot.q)
        self.camera.setQuat(q)

    #---------------------------------------------------------------------------

    def cameraMoveTask(self, task):
        if self.editing_widget.is_recording and self.write_frame_complete:
            self.setCameraView(*self.editing_widget.nextFrame())
            self.writeFrame()
            return task.cont

        if self.editing_widget.is_playing:
            self.setCameraView(*self.editing_widget.nextFrame())
            return task.cont

        if not base.mouseWatcherNode.hasMouse():
            return task.cont

        mouse_xy = base.mouseWatcherNode.getMouse()
        mouse_xy = np.array((mouse_xy[0], mouse_xy[1]))

        # need to update the initial mouse position before performing any other
        # actions
        if self.start_new_mouse_action:
            self.start_new_mouse_action = False

        elif self.do_rotate:
            dxy = (self.old_mouse_xy - mouse_xy) * VideoTool.ROT_MULT
            dxy = np.array((dxy[0], -1., dxy[1]))
            dxy /= np.linalg.norm(dxy) # normalize to unit length

            # u = [0, -1, 0], v = dxy; q.xyz = u cross v, q.w = sqrt(2) + u.v
            q = Quaternion((np.sqrt(2.) - dxy[1], -dxy[2], 0., dxy[0]))
            q.normalize()

            self.camera_rot *= q

        elif self.do_xy_trans:
            dxy = mouse_xy - self.old_mouse_xy
            dxy = np.array((dxy[0], 0., dxy[1]))

            self.camera_center -= (
                self.camera_rot.ToR().dot(dxy) * self.view_radius *
                VideoTool.XY_TRANS_MULT)

        elif self.do_z_trans:
            self.camera_center += (
                (mouse_xy[1] - self.old_mouse_xy[1]) *
                self.camera_rot.ToR()[:,1] * self.view_radius *
                VideoTool.Z_TRANS_MULT)

        self._update_camera_view()
        
        self.old_mouse_xy = mouse_xy

        return task.cont

    #---------------------------------------------------------------------------

    def addNewCameraPosition(self):
        self.editing_widget.addTrackEntry(
            self.camera_rot, self.camera_center, self.view_radius)

    def deleteCameraPosition(self):
        self.editing_widget.deleteTrackEntry()

    def insertNewCameraPosition(self):
        self.editing_widget.addTrackEntry(
            self.camera_rot, self.camera_center, self.view_radius, True)

    def modifySelectedCameraPosition(self):
        self.editing_widget.modifyTrackEntry(
            self.camera_rot, self.camera_center, self.view_radius)

    #---------------------------------------------------------------------------

    def setCameraView(self, camera_rot, camera_center, view_radius):
        self.camera_rot = camera_rot.copy()
        self.camera_center = camera_center.copy()
        self.view_radius = view_radius
        self._update_camera_view()

    #---------------------------------------------------------------------------

    def updateAmbient(self, asset_idx, light_is_on):
        if light_is_on:
            self.meshes[asset_idx].setLight(self.alight_node)
        else:
            self.meshes[asset_idx].setLightOff(self.alight_node)

    def updateDirectional(self, asset_idx, light_is_on):
        if light_is_on:
            self.meshes[asset_idx].setLight(self.dlight1_node)
            #self.meshes[asset_idx].setLight(self.dlight2_node)
        else:
            self.meshes[asset_idx].setLightOff(self.dlight1_node)
            #self.meshes[asset_idx].setLightOff(self.dlight2_node)

    #---------------------------------------------------------------------------
    # video recording functions
    #---------------------------------------------------------------------------

    def startRecording(self, folder):
        if not os.path.exists(folder):
            os.makedirs(folder)
        self.output_image_path = os.path.join(folder, "%06d.png")
        self.write_frame_complete = True

        self.current_frame = self.starting_frame

    def writeFrame(self):
        self.write_frame_complete = False

        # force render update
        self.graphicsEngine.renderFrame()

        # get image; see
        # gist.github.com/alexlee-gk/b28fb962c9b2da586d1591bac8888f1f
        texture = self.win.getScreenshot()
        data = texture.getRamImageAs("BGR")
        image = np.frombuffer(data.get_data(), np.uint8)
        image.shape = (texture.getYSize(), texture.getXSize(), 3)
        image = np.flipud(image).astype(np.float32)

        imageio.imwrite(self.output_image_path % self.current_frame, image)
        self.current_frame += 1
        self.write_frame_complete = True

        if (self.stopping_frame is not None and
                self.current_frame >= self.stopping_frame):
            exit()

    def stopRecording(self):
        self.output_folder = None
        self.write_frame_complete = False

        if self.exit_when_recording_done:
            exit()

#-------------------------------------------------------------------------------
#
# main
#
#-------------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--window_size", nargs=2, type=int, default=(1024, 768))
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument(
        "--input_file", type=str, default=None,
        help="pre-load the specified camera track")
    parser.add_argument(
        "--output_folder", type=str, default=None,
        help="if input_file is specified, automatically render that camera "
             "track to this output folder and exit")

    parser.add_argument(
        "--starting_frame", type=int, default=0,
        help="start play at the this (absolute) frame index")

    parser.add_argument(
        "--stopping_frame", type=int, default=None,
        help="stop play at the this (absolute) frame index, exclusive")

    args = parser.parse_args()

#    p3d.core.loadPrcFileData('', 'want-tk true') # on Mac
    p3d.core.loadPrcFileData('', 'win-size %i %i' % tuple(args.window_size))
    p3d.core.loadPrcFileData('', 'win-fixed-size 1')
    p3d.core.loadPrcFileData('', 'audio-library-name null')
    p3d.core.loadPrcFileData('', 'framebuffer-multisample 1')
    p3d.core.loadPrcFileData('', 'depth-bits 24') # set for NVidia GTX 980 Ti
    p3d.core.loadPrcFileData('', 'multisamples 16') # set for NVidia GTX 980 Ti

    # debugging
#    p3d.core.loadPrcFileData('', 'notify-level-display debug')
#    p3d.core.loadPrcFileData('', 'notify-level-glgsg debug')

    # fixed frame rate
    p3d.core.loadPrcFileData('', 'clock-mode limited')
    p3d.core.loadPrcFileData('', 'clock-frame-rate %i' % args.fps)

    VideoTool(args).run()
