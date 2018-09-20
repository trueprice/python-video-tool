import numpy as np
import os
import sys

if sys.version_info >= (3, 0):
    import tkinter as tk
    from tkinter.filedialog import askopenfilename, asksaveasfilename
    from tkinter.filedialog import askdirectory
else:
    import Tkinter as tk
    from tkFileDialog import askopenfilename, asksaveasfilename, askdirectory

from AssetEntry import AssetEntryListbox
from CameraTrackEntry import CameraTrackEntry
from rotation import Quaternion, DualQuaternion

#-------------------------------------------------------------------------------
#
# CameraTrackEditingWidget
#
#-------------------------------------------------------------------------------

INSTRUCTIONS = """
Controls:
<Space>: Add new keyframe to end
<a>: Add new keyframe after selected keyframe
<d>: Delete selected keyframe
<m>: Modify selected keyframe to current view

Turn on/off Ambient/Directional lighting using the buttons next to the assets.

Press record to save a video of the camera tracks to a file,
and press record while capturing to cancel this.
"""

class CameraTrackEditingWidget(tk.Frame, object):
    def __init__(self, parent, fps, starting_frame=0):
        super(CameraTrackEditingWidget, self).__init__(parent)

        self.starting_frame = starting_frame

        self.load_asset_callback = None
        self.remove_asset_callback = None
        self.remove_all_assets_callback = None

        self.keyframe_activation_callback = None

        self.change_ambient_callback = None
        self.change_directional_callback = None

        self.start_recording_callback = None
        self.stop_recording_callback = None

        self.get_view_params = None # function handle; see video_tool.py

        self.fps = fps

        # top row of buttons
        top_frame = tk.Frame(self)
        load_button = tk.Button(top_frame, text="Load", command=self.loadTrack)
        load_button.pack(side=tk.LEFT)
        save_button = tk.Button(top_frame, text="Save", command=self.saveTrack)
        save_button.pack(side=tk.LEFT)
        top_frame.pack()

        # second row of buttons
        controls_frame = tk.Frame(self)
        play_button = tk.Button(controls_frame, text="Play", command=self.play)
        play_button.pack(side=tk.LEFT)
        play_from_start_button = tk.Button(
            controls_frame, text="Play From Start", command=self.playFromStart)
        play_from_start_button.pack(side=tk.LEFT)
        pause_button = tk.Button(controls_frame, text="Pause",
            command=self.pause)
        pause_button.pack(side=tk.LEFT)
        controls_frame.pack()

        # third row of buttons
        self.is_recording = False
        record_frame = tk.Frame(self)
        self.record_button = tk.Button(record_frame, text="Record",
            command=self.toggleRecording)
        self.record_button.pack()
        self.default_record_button_color = self.record_button.cget("bg")
        record_frame.pack()

        # assets list
        assets_label = tk.Label(self, text="Assets")
        assets_label.pack(fill=tk.X)

        self.asset_buttons_frame = tk.Frame(self)

        add_asset_button = tk.Button(
            self.asset_buttons_frame, text="Add", command=self._load_asset)
        delete_asset_button = tk.Button(
            self.asset_buttons_frame, text="Delete", command=self._remove_asset)

        add_asset_button.pack(side=tk.LEFT)
        delete_asset_button.pack(side=tk.LEFT)

        self.asset_buttons_frame.pack()

        self.assets_list = AssetEntryListbox(
            self, self._change_ambient, self._change_directional)
        self.assets_list.pack(fill=tk.X)

        # camera track list
        track_label = tk.Label(self, text="Camera Track")
        track_label.pack()

        self.track_buttons_frame = tk.Frame(self)

        add_track_button = tk.Button(self.track_buttons_frame, text="Add",
            command=self._add_track_entry)
        insert_track_button = tk.Button(self.track_buttons_frame, text="Insert",
            command=self._insert_track_entry)
        delete_track_button = tk.Button(self.track_buttons_frame, text="Delete",
            command=self.deleteTrackEntry)
        modify_track_button = tk.Button(self.track_buttons_frame, text="Modify",
            command=self._modify_track_entry)

        add_track_button.pack(side=tk.LEFT)
        insert_track_button.pack(side=tk.LEFT)
        delete_track_button.pack(side=tk.LEFT)
        modify_track_button.pack(side=tk.LEFT)

        self.track_buttons_frame.pack()

        self.track_frame = tk.Frame(self)
        self.track_frame.pack(side=tk.LEFT)

        # unique identifier for generated keyframes
        self.selected_keyframe_idx = -1
        self.next_keyframe_id = 0
        self.keyframes = dict()
        self.keyframes_list = [] # order of unique ids in the list

        # instructions in another window
        instructions = tk.Toplevel(parent)
        message = tk.Message(instructions, text=INSTRUCTIONS, width=800)
        message.pack()

        self.pack()

        # finally, set up the play functionality
        self.resetPlay()

    #---------------------------------------------------------------------------

    def _load_asset(self, filepath=None):
        if not filepath:
            filepath = askopenfilename(
                filetypes=[("Asset files", ("*.ply", "*.x3d"))])

        if filepath:
            self.assets_list.addAsset(filepath)

            if self.load_asset_callback:
                self.load_asset_callback(filepath)

    def _remove_asset(self):
        asset_idx = self.assets_list.removeAsset()

        if asset_idx >= 0 and self.remove_asset_callback:
            self.remove_asset_callback(asset_idx)

    def _change_ambient(self, asset_idx, light_is_on):
        if self.change_ambient_callback:
            self.change_ambient_callback(asset_idx, light_is_on)

    def _change_directional(self, asset_idx, light_is_on):
        if self.change_directional_callback:
            self.change_directional_callback(asset_idx, light_is_on)

    #---------------------------------------------------------------------------

    def _add_track_entry(self):
        self.addTrackEntry(*self.get_view_params())

    def _insert_track_entry(self):
        self.addTrackEntry(*self.get_view_params(), insert_requested=True)

    def _modify_track_entry(self):
        self.modifyTrackEntry(*self.get_view_params())

    #---------------------------------------------------------------------------
    
    def _activate_keyframe(self):
        if self.selected_keyframe_idx != -1:
            # set up play to start from this keyframe
            self.current_keyframe_idx = self.selected_keyframe_idx
            self.current_frame = -1

            if self.keyframe_activation_callback:
                keyframe = self.keyframes[ \
                    self.keyframes_list[self.selected_keyframe_idx]]
                self.keyframe_activation_callback(
                    keyframe.camera_rot, keyframe.camera_center,
                    keyframe.view_radius)

    #---------------------------------------------------------------------------

    def _change_selected_keyframe(self, keyframe_id, keyframe_selected):
        if self.selected_keyframe_idx != -1 and keyframe_selected:
            self.keyframes[
                self.keyframes_list[self.selected_keyframe_idx]].Unselect()

        self.selected_keyframe_idx = self.keyframes_list.index(keyframe_id) \
                                     if keyframe_selected else -1

    #---------------------------------------------------------------------------

    def addTrackEntry(self, camera_rot, camera_center, view_radius,
                      insert_requested=False):
        if self.is_recording: return

        self.resetPlay()

        self.keyframes[self.next_keyframe_id] = CameraTrackEntry(
            self.track_frame, self.next_keyframe_id,
            camera_rot, camera_center, view_radius,
            self._change_selected_keyframe, self._activate_keyframe, self.fps)

        # if an insert was requested, check also that any keyframe was selected,
        # and that if there is one selected, that it wasn't the last one

        insert_requested &= (
            self.selected_keyframe_idx != -1 and
            self.selected_keyframe_idx != len(self.keyframes_list) - 1)

        if insert_requested:
            row_idx = self.selected_keyframe_idx + 1 # insertion row in the grid

            for idx, keyframe_id in enumerate(self.keyframes_list[row_idx:]):
                keyframe = self.keyframes[keyframe_id]
                keyframe.grid_forget()
                keyframe.grid(row=row_idx + idx + 1, column=0)

            self.keyframes_list.insert(row_idx, self.next_keyframe_id)
        else:
            row_idx = len(self.keyframes_list)
            self.keyframes_list.append(self.next_keyframe_id)

        # actually do the final grid insertion
        self.keyframes[self.next_keyframe_id].grid(row=row_idx, column=0)

        # force unselection of all keyframes
        if self.selected_keyframe_idx != -1:
            self.keyframes[
                self.keyframes_list[self.selected_keyframe_idx]].Unselect()
            self.selected_keyframe_idx = -1

        self.next_keyframe_id += 1

    #---------------------------------------------------------------------------

    def modifyTrackEntry(self, camera_rot, camera_center, view_radius):
        if self.is_recording: return

        if self.selected_keyframe_idx != -1:
            self.resetPlay()

            keyframe = \
                self.keyframes[self.keyframes_list[self.selected_keyframe_idx]]
            keyframe.camera_rot = camera_rot.copy()
            keyframe.camera_center = camera_center.copy()
            keyframe.view_radius = view_radius

    #---------------------------------------------------------------------------

    def deleteTrackEntry(self):
        if self.is_recording: return

        if self.selected_keyframe_idx != -1:
            self.resetPlay()

            row_idx = self.selected_keyframe_idx + 1 # next row in the grid

            self.keyframes[self.keyframes_list[self.selected_keyframe_idx]] \
                .grid_forget()

            # shift all remaining keyframes up
            for idx, keyframe_id in enumerate(self.keyframes_list[row_idx:]):
                keyframe = self.keyframes[keyframe_id]
                keyframe.grid_forget()
                keyframe.grid(row=row_idx + idx - 1, column=0)

            del self.keyframes[
                self.keyframes_list.pop(self.selected_keyframe_idx)]

            self.selected_keyframe_idx = -1

    #---------------------------------------------------------------------------

    def resetPlay(self):
        self.pause()
        self.current_keyframe_idx = 0
        self.current_frame = -1

    def playFromStart(self):
        self.resetPlay()
        self.play()

    def play(self):
        for n in xrange(self.starting_frame):
            self.nextFrame()

        if self.current_keyframe_idx < len(self.keyframes_list):
            self.is_playing = True

    def pause(self):
        self.is_playing = False

    #---------------------------------------------------------------------------

    def nextFrame(self):
        self.current_frame += 1

        current_keyframe = self.keyframes[
            self.keyframes_list[self.current_keyframe_idx]]

        current_duration = int(current_keyframe.duration.get())

        # Hermite spline interpolation of dual quaternions
        # pdfs.semanticscholar.org/05b1/8ede7f46c29c2722fed3376d277a1d286c55.pdf
        if 1 <= self.current_keyframe_idx < len(self.keyframes_list) - 2:
            # get the previous keyframe and the next two keyframes
            k0 = self.keyframes[
                self.keyframes_list[self.current_keyframe_idx - 1]]
            k1 = current_keyframe
            k2 = self.keyframes[
                self.keyframes_list[self.current_keyframe_idx + 1]]
            k3 = self.keyframes[
                self.keyframes_list[self.current_keyframe_idx + 2]]

            prev_duration = int(k0.duration.get())
            next_duration = int(k2.duration.get())

            dq0 = DualQuaternion.FromQT(k0.camera_rot, k0.camera_center)
            dq1 = DualQuaternion.FromQT(k1.camera_rot, k1.camera_center)
            dq2 = DualQuaternion.FromQT(k2.camera_rot, k2.camera_center)
            dq3 = DualQuaternion.FromQT(k3.camera_rot, k3.camera_center)

            # like quaternions, dq(x) = -dq(x), so we'll need to pick the one
            # more appropriate for interpolation by taking -dq if the dot
            # product of the two q-vectors is negative
            if dq0.q0.dot(dq1.q0) < 0:
                dq1 = -dq1
            if dq1.q0.dot(dq2.q0) < 0:
                dq2 = -dq2
            if dq2.q0.dot(dq3.q0) < 0:
                dq3 = -dq3

            # approximate the derivatives at dq1 and dq2 using weighted central
            # differences
            dt1 = 1. / float(prev_duration + current_duration)
            dt2 = 1. / float(current_duration + next_duration)

            m1 = (current_duration * dt1) * (dq2 - dq1) + \
                 (prev_duration * dt1) * (dq1 - dq0) 
            m2 = (next_duration * dt2) * (dq3 - dq2) + \
                 (current_duration * dt2) * (dq2 - dq1) 

            # we'll also need to interpolate the view radius (denote r:=m)
            r1 = (current_duration * dt1) * (k2.view_radius - k1.view_radius) + \
                 (prev_duration * dt1) * (k1.view_radius - k0.view_radius) 
            r2 = (next_duration * dt2) * (k3.view_radius - k2.view_radius) + \
                 (current_duration * dt2) * (k2.view_radius - k1.view_radius) 

            #
            # actually perform the interpolation
            #

            t = float(self.current_frame) / current_duration # 0 to 1
            t2 = t * t # t squared
            t3 = t2 * t # t cubed

            # coefficients of the Hermite spline (a=>dq and b=>m)
            a1 = 2. * t3 - 3. * t2 + 1.
            b1 = t3 - 2. * t2 + t
            a2 = -2. * t3 + 3. * t2
            b2 = t3 - t2

            dq = a1 * dq1 + b1 * m1 + a2 * dq2 + b2 * m2
            dq.normalize()

            # we'll also need to interpolate the view radius
            radius = a1 * current_keyframe.view_radius + b1 * r1 + \
                     a2 * k2.view_radius + b2 * r2
            #radius = (1. - t) * current_keyframe.view_radius + \
            #         t * k2.view_radius

            pose = dq.ToQT() + (radius,)

        # or, dual quaternion linear blending if not the last frame
        elif self.current_keyframe_idx < len(self.keyframes_list) - 1:
            next_keyframe = self.keyframes[
                self.keyframes_list[self.current_keyframe_idx + 1]]

            alpha = float(self.current_frame) / current_duration

            dq0 = DualQuaternion.FromQT(
                current_keyframe.camera_rot, current_keyframe.camera_center)
            dq1 = DualQuaternion.FromQT(
                next_keyframe.camera_rot, next_keyframe.camera_center)

            # like quaternions, dq(x) = -dq(x), so we'll need to pick the one
            # more appropriate for interpolation by taking -dq if the dot
            # product of the two q-vectors is negative
            if dq0.q0.dot(dq1.q0) < 0:
                dq1 = -dq1

            dq = (1. - alpha) * dq0 + alpha * dq1
            dq.normalize()
            radius = ((1. - alpha) * current_keyframe.view_radius +
                      alpha * next_keyframe.view_radius)

            pose = dq.ToQT() + (radius,)

        # the last frame doesn't move
        else:
            pose = (current_keyframe.camera_rot,
                    current_keyframe.camera_center,
                    current_keyframe.view_radius)

        if self.current_frame == current_duration:
            self.current_frame = 0
            self.current_keyframe_idx += 1

            if self.current_keyframe_idx == len(self.keyframes_list):
                self.is_playing = False

                # finalize recording
                if self.is_recording:
                    self.toggleRecording()

        return pose

    #---------------------------------------------------------------------------

    def toggleRecording(self, folder=None):
        if not self.is_recording:
            if folder is None:
                folder = askdirectory()
                if not folder: # no folder selected
                    self.is_recording = False
                    return

            self.record_button.configure(bg="red")
            self.is_recording = True

            if self.start_recording_callback:
                self.start_recording_callback(folder)

            self.resetPlay()
            self.play()

        else:
            self.is_recording = False
            if self.stop_recording_callback:
                self.stop_recording_callback()
            self.record_button.configure(bg=self.default_record_button_color)

    #---------------------------------------------------------------------------
    # load/save functions
    #---------------------------------------------------------------------------

    def loadTrack(self, filename=None):
        if filename is None:
            filename = askopenfilename()

            if not filename: return

        self.is_recording = False
        self.resetPlay()

        # clear existing data
        self.assets_list.clear()

        if self.remove_all_assets_callback:
            self.remove_all_assets_callback()

        self.selected_keyframe_idx = -1
        self.keyframes_list = []
        self.next_keyframe_id = 0
        for keyframe in self.keyframes.itervalues():
            keyframe.grid_forget()
            keyframe.destroy()
        self.keyframes = dict()

        # load new data
        with open(filename, "r") as fid:
            for line in fid:
                if line.startswith("asset "):
                    filepath, ambient, directional = line[6:].rsplit(" ", 2)
                    print "Loading", filepath
                    ambient, directional = int(ambient), int(directional)
                    self._load_asset(filepath)
                    if not ambient:
                        self.assets_list.asset_entries[-1].toggleAmbient()
                    if not directional:
                        self.assets_list.asset_entries[-1].toggleDirectional()
                else:
                    data = map(float, line.split())
                    self.addTrackEntry(
                        Quaternion(data[:4]), np.array(data[4:7]), data[7])
                    self.keyframes[self.keyframes_list[-1]].setDuration(
                        int(data[8]))

        # set camera view to first keyframe
        if self.keyframe_activation_callback:
            keyframe = self.keyframes[self.keyframes_list[0]]
            self.keyframe_activation_callback(
                keyframe.camera_rot, keyframe.camera_center,
                keyframe.view_radius)

    def saveTrack(self):
        filename = asksaveasfilename()

        if not filename: return

        with open(filename, "w") as fid:
            for asset_entry in self.assets_list.asset_entries:
                print>>fid, "asset", os.path.abspath(asset_entry.filepath),
                print>>fid, asset_entry.ambient_on.get(),
                print>>fid, asset_entry.directional_on.get()

            for idx in self.keyframes_list:
                keyframe = self.keyframes[idx]
                print>>fid, ' '.join(map(str, keyframe.camera_rot.q)),
                print>>fid, ' '.join(map(str, keyframe.camera_center)),
                print>>fid, keyframe.view_radius,
                print>>fid, keyframe.duration.get()
