import os
import re
import traceback

import adsk.core
import adsk.fusion


COMMAND_ID = "saExportActiveSketchesToDxf"
COMMAND_NAME = "Export Sketches to DXF"
WORKSPACE_ID = "FusionSolidEnvironment"
PANEL_ID = "SolidScriptsAddinsPanel"
OUTPUT_DIRECTORY = (
    r"C:\Users\A1234\OneDrive\Documentos\Default Project\schematics"
)

_app = None
_ui = None
_command_definition = None
_command_control = None
_command_created_handler = None
_handlers = []


def _safe_filename_part(value):
    value = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", value).strip().rstrip(". ")
    if not value:
        value = "Sketch"
    if value.split(".")[0].upper() in {
        "CON", "PRN", "AUX", "NUL",
        "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9",
        "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9",
    }:
        value = "_" + value
    return value


def _unique_dxf_path(component_name, sketch_name, used_names):
    stem = "{}_{}".format(
        _safe_filename_part(component_name),
        _safe_filename_part(sketch_name),
    )
    filename = stem + ".dxf"
    suffix = 2
    while filename.lower() in used_names:
        filename = "{}_{}.dxf".format(stem, suffix)
        suffix += 1
    used_names.add(filename.lower())
    return os.path.join(OUTPUT_DIRECTORY, filename)


class ExportSketchesCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            design = adsk.fusion.Design.cast(_app.activeProduct)
            if not design:
                _ui.messageBox("Open a Fusion design before exporting sketches.")
                return

            os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
            used_names = set()
            exported = 0
            failures = []

            for component in design.allComponents:
                for sketch in component.sketches:
                    path = _unique_dxf_path(component.name, sketch.name, used_names)
                    try:
                        if sketch.saveAsDXF(path):
                            exported += 1
                        else:
                            failures.append("{}: Fusion could not save the DXF.".format(
                                os.path.basename(path)
                            ))
                    except Exception as error:
                        failures.append("{}: {}".format(
                            os.path.basename(path), error
                        ))

            if exported == 0 and not failures:
                _ui.messageBox("No sketches were found in the active design.")
                return

            message = "Exported {} sketch(es) to:\n{}".format(
                exported, OUTPUT_DIRECTORY
            )
            if failures:
                message += "\n\nFailed to export {} sketch(es):\n{}".format(
                    len(failures), "\n".join(failures)
                )
            _ui.messageBox(message)
        except Exception:
            _ui.messageBox("DXF export failed:\n{}".format(traceback.format_exc()))


class ExportSketchesCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            execute_handler = ExportSketchesCommandExecuteHandler()
            args.command.execute.add(execute_handler)
            _handlers.append(execute_handler)
        except Exception:
            _ui.messageBox("Could not initialize DXF export:\n{}".format(
                traceback.format_exc()
            ))


def run(context):
    global _app, _ui, _command_definition, _command_control
    global _command_created_handler

    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        workspace = _ui.workspaces.itemById(WORKSPACE_ID)
        if not workspace:
            raise RuntimeError("The Fusion Solid workspace is unavailable.")

        panel = workspace.toolbarPanels.itemById(PANEL_ID)
        if not panel:
            raise RuntimeError("The Utilities Scripts and Add-Ins panel is unavailable.")

        _command_definition = _ui.commandDefinitions.addButtonDefinition(
            COMMAND_ID,
            COMMAND_NAME,
            "Export every sketch in the active design as a DXF file.",
        )
        _command_created_handler = ExportSketchesCommandCreatedHandler()
        _command_definition.commandCreated.add(_command_created_handler)

        _command_control = panel.controls.addCommand(_command_definition)
        _command_control.isPromoted = True
    except Exception:
        if _ui:
            _ui.messageBox("Failed to start the sketch DXF add-in:\n{}".format(
                traceback.format_exc()
            ))


def stop(context):
    global _command_definition, _command_control, _command_created_handler

    if _command_definition and _command_created_handler:
        _command_definition.commandCreated.remove(_command_created_handler)
    if _command_control:
        _command_control.deleteMe()
    if _command_definition:
        _command_definition.deleteMe()

    _command_definition = None
    _command_control = None
    _command_created_handler = None
    _handlers.clear()
