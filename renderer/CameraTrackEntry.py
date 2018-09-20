import sys

if sys.version_info >= (3, 0):
    import tkinter as tk
    from tkinter.filedialog import askopenfilename, askdirectory
else:
    import Tkinter as tk
    from tkFileDialog import askopenfilename, askdirectory

#-------------------------------------------------------------------------------
#
# CameraTrackEntry
#
#-------------------------------------------------------------------------------

class CameraTrackEntry(tk.Frame, object):
    MAX_NUM_FRAMES = 9999

    def __init__(self, parent, id_, camera_rot, camera_center, view_radius,
            onclick_callback, ondoubleclick_callback, initial_duration):
        super(CameraTrackEntry, self).__init__(parent)

        self.id = id_

        self.camera_rot = camera_rot.copy()
        self.camera_center = camera_center.copy()
        self.view_radius = view_radius

        self.onclick_callback = onclick_callback
        self.ondoubleclick_callback = ondoubleclick_callback

        self.selected = False

        self.label = tk.Label(self, text="keyframe")
        self.label.pack(side=tk.LEFT)
        self.unselected_label_color = self.label.cget("bg")

        self.duration = tk.Spinbox(
            self, from_=0, to=CameraTrackEntry.MAX_NUM_FRAMES,
            width=len(str(CameraTrackEntry.MAX_NUM_FRAMES)))
        self.setDuration(initial_duration)
        self.duration.pack(side=tk.LEFT)

        self.label.bind("<Button-1>", self._on_click)
        self.label.bind("<Double-Button-1>", self._on_double_click)

    #---------------------------------------------------------------------------

    def Select(self):
        self.selected = True
        self.label.configure(bg="sky blue")

    def Unselect(self):
        self.selected = False
        self.label.configure(bg=self.unselected_label_color)

    def setDuration(self, duration):
        self.duration.delete(0, tk.END)
        self.duration.insert(0, duration)

    #---------------------------------------------------------------------------

    def _on_click(self, event):
        if not self.selected:
            self.Select()
        else:
            self.Unselect()

        if self.onclick_callback:
            self.onclick_callback(self.id, self.selected)

    def _on_double_click(self, event):
        self.Select()
        if self.ondoubleclick_callback:
            self.ondoubleclick_callback()


