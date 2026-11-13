# Sketch DXF Exporter

This Fusion 360 add-in adds **Export Sketches to DXF** to the Solid
workspace's **Utilities > Scripts and Add-Ins** panel. It exports every sketch
in the active design to:

`C:\Users\A1234\OneDrive\Documentos\Default Project\schematics`

In Fusion, open **Utilities > Scripts and Add-Ins**, add this `fusion_addin`
folder as an add-in if it is not listed, and click **Run**. Use **Stop** in
that dialog to remove the button.

After exporting at least one sketch, the add-in runs Git from the project
directory, stages only `schematics/`, commits schematic changes, and pushes
the current branch. It uses
`C:\Program Files\Git\cmd\git.exe` directly, disables interactive credential
prompts, and times out Git commands after 45 seconds. Configure the repository
remote and authenticate Git for Windows before using auto-sync; sync errors are
shown in Fusion's export result.
