import os
import sys

if sys.version_info >= (3, 0):
    import tkinter as tk
    from tkinter.filedialog import askopenfilename, askdirectory
else:
    import Tkinter as tk
    from tkFileDialog import askopenfilename, askdirectory

#-------------------------------------------------------------------------------
#
# AssetEntryListbox
#
#-------------------------------------------------------------------------------

class AssetEntryListbox(tk.Frame, object):
    def __init__(self, parent,
                 onambient_callback=None,
                 ondirectional_callback=None):
        super(AssetEntryListbox, self).__init__(parent)

        self.onambient_callback = onambient_callback
        self.ondirectional_callback = ondirectional_callback

        self.selected_asset_idx = -1
        self.asset_entries = []

    #---------------------------------------------------------------------------

    def addAsset(self, filepath):
        # create and add the asset entry
        asset_idx = len(self.asset_entries)

        self.asset_entries.append(
            AssetEntry(self, asset_idx, filepath, self.changeSelected,
                       self.onambient_callback, self.ondirectional_callback))
        self.asset_entries[-1].grid(row=asset_idx, column=0)

    def removeAsset(self):
        if self.selected_asset_idx == -1: return -1

        row_idx = self.selected_asset_idx + 1 # next row in the grid

        self.asset_entries[self.selected_asset_idx].grid_forget()

        # shift all remaining assets up
        for idx, asset_entry in enumerate(self.asset_entries[row_idx:]):
            asset_entry.id -= 1
            asset_entry.grid_forget()
            asset_entry.grid(row=asset_entry.id, column=0)

        self.asset_entries.pop(self.selected_asset_idx)

        self.selected_asset_idx = -1

        return row_idx - 1

    def clear(self):
        for asset_entry in self.asset_entries:
            asset_entry.grid_forget()
            asset_entry.destroy()
        self.selected_asset_idx = -1
        self.asset_entries = []

    #---------------------------------------------------------------------------

    def changeSelected(self, idx, selected):
        if self.selected_asset_idx != -1 and selected:
            self.asset_entries[self.selected_asset_idx].Unselect()

        self.selected_asset_idx = idx if selected else -1

#-------------------------------------------------------------------------------
#
# AssetEntry
#
#-------------------------------------------------------------------------------

class AssetEntry(tk.Frame, object):
    def __init__(self, parent, id_, filepath, onclick_callback,
                 onambient_callback, ondirectional_callback):
        super(AssetEntry, self).__init__(parent)

        self.onclick_callback = onclick_callback
        self.onambient_callback = onambient_callback
        self.ondirectional_callback = ondirectional_callback

        self.id = id_
        self.filepath = filepath
        self.selected = False

        self.label = tk.Label(self, text=os.path.basename(filepath))
        self.label.pack(side=tk.LEFT)
        self.unselected_label_color = self.label.cget("bg")

        self.ambient_on = tk.IntVar()
        self.ambient_on.set(1)
        self.ambient_checkbox = tk.Checkbutton(
            self, text="A", variable=self.ambient_on,
            command=self.toggleAmbient_callback)

        self.directional_on = tk.IntVar()
        self.directional_on.set(1)
        self.directional_checkbox = tk.Checkbutton(
            self, text="D", variable=self.directional_on,
            command=self.toggleDirectional_callback)

        self.ambient_checkbox.pack(side=tk.LEFT)
        self.directional_checkbox.pack(side=tk.LEFT)

        self.label.bind("<Button-1>", self._on_click)

    #---------------------------------------------------------------------------

    def Select(self):
        self.selected = True
        self.label.configure(bg="sky blue")

    def Unselect(self):
        self.selected = False
        self.label.configure(bg=self.unselected_label_color)

    #---------------------------------------------------------------------------

    def toggleAmbient(self):
        self.ambient_checkbox.toggle()
        self.toggleAmbient_callback()

    def toggleDirectional(self):
        self.directional_checkbox.toggle()
        self.toggleDirectional_callback()

    def toggleAmbient_callback(self):
        if self.onambient_callback:
            self.onambient_callback(self.id, self.ambient_on.get())

    def toggleDirectional_callback(self):
        if self.ondirectional_callback:
            self.ondirectional_callback(self.id, self.directional_on.get())

    #---------------------------------------------------------------------------

    def _on_click(self, event):
        if not self.selected:
            self.Select()
        else:
            self.Unselect()

        if self.onclick_callback:
            self.onclick_callback(self.id, self.selected)
