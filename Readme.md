Modifiziere TfViewerApp.py

installiere
pip3 install pyqt6

Add other requirements here....

To install the package
Go into your 'src' folder of your ros2 installation
Open a terminal
'git clone https://github.com/match-PM/pm_tf_viewer.git'

To build the package (make sure you are in the ros workspace)
'colcon build --packages-select pm_tf_viewer'

To start the Application
'ros2 run pm_tf_viewer'

To change the branch
'git checkout main'
'git checkout leonard_working'

To add changes 
'git add --all'
"git commit - m 'MEANINGFULL MESSAGE'"
'git push'




## Comparing frames

The changes from `leonard_working` are ported manually to main. The transform
list sorts frame names alphabetically (letters, then digits, then other names).
Open the **Calculation Panel** and select two frames directly in a calculation
slide. All frames available relative to the selected parent are listed
automatically. Use **+ Add calculation** for additional comparisons.
**Refresh Transforms** fetches the latest poses;
values refresh on these actions and when units or the parent frame change.

Each slide shows Frame 2 minus Frame 1 translation and vector length, together
with the active rotation `R2 * inverse(R1)`. Both differences use the selected
parent frame's axes. Rotation can be shown as xyz Euler angles (rad/deg) or a
quaternion (x, y, z, w).

**Freeze** captures one selected pose and locks that selection until released.
Snapshots keep their original metre values, so changing display units remains
valid. Changing the parent frame releases all frozen poses to avoid mixing
coordinate systems. Deleting another slide does not affect frozen poses.
**Copy Results** copies the displayed values and frame names to the clipboard.
**Clear all** deletes all calculation slides. Double-clicking a frame in
the **Transforms** tab selects it in the most recent slide as a shortcut.

Use **Save…** in the calculation toolbar to save all slides to a JSON file.
**Load…** replaces the current slides and restores frame selections, frozen
poses, parent frame, and display units. Frozen poses are stored in metres and
can be viewed without live TF data. Live selections resume when their frames
are available; unavailable selections are retained. Invalid files leave the
current slides unchanged.
