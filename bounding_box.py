import adsk.core, adsk.fusion, traceback
import numpy as np
from scipy.spatial import ConvexHull

app = None
ui  = None
commandId = 'CalculateTightBoundingBox'
commandName = 'Calculate Tight Bounding Box'
commandDescription = 'Display tight bounding box dimensions in status bar for selected solid bodies.'

handlers = []

def display_tight_bounding_box_dimensions(solid_body):
    global ui
    try:
        volume, centroid = solid_body.physicalProperties.volume, solid_body.physicalProperties.centroid
        mesh = solid_body.meshManager.displayMeshes
        vertices = [np.array([v.geometry.x, v.geometry.y, v.geometry.z]) for m in mesh for v in m.vertexBuffer]
        hull = ConvexHull(vertices)

        min_volume = float('inf')
        min_oriented_bbox = None
        for simplex in hull.simplices:
            edge1 = vertices[simplex[1]] - vertices[simplex[0]]
            edge2 = vertices[simplex[2]] - vertices[simplex[0]]
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal)
            R = np.column_stack((edge1 / np.linalg.norm(edge1), edge2 / np.linalg.norm(edge2), normal))
            rotated_points = np.dot(vertices, R)
            min_coords = rotated_points.min(axis=0)
            max_coords = rotated_points.max(axis=0)
            volume = np.prod(max_coords - min_coords)

            if volume < min_volume:
                min_volume = volume
                min_oriented_bbox = (min_coords, max_coords, R)

        min_coords, max_coords, R = min_oriented_bbox
        box_size = max_coords - min_coords
        T, W, L = box_size[0], box_size[1], box_size[2]
        ui.statusBarText = f"Tight bounding box dimensions: T: {T:.2f} x W: {W:.2f} x L: {L:.2f}"
    except:
        ui.statusBarText = "Error calculating tight bounding box dimensions."

class CalculateBoundingBoxExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            cmd = args.firingEvent.sender
            inputs = cmd.commandInputs
            global commandId
            selectionInput = None
            for inputI in inputs:
                if inputI.id == commandId + '_selection':
                    selectionInput = inputI

            selected_entity = selectionInput.selection(0).entity
            solid_body = adsk.fusion.BRepBody.cast(selected_entity)

            if solid_body is None:
                ui.messageBox("Selected object is not a solid body.")
                return

            display_tight_bounding_box_dimensions(solid_body)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class CalculateBoundingBoxDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            adsk.terminate()
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class CalculateBoundingBoxCreatedHandler(adsk.core.CommandCreatedEventHandler):    
    def __init__(self):
        super().__init__()        
    def notify(self, args):
        try:
            cmd = args.command
            cmd.isRepeatable = False
            onExecute = CalculateBoundingBoxExecuteHandler()
            cmd.execute.add(onExecute)
            
            onDestroy = CalculateBoundingBoxDestroyHandler()
            cmd.destroy.add(onDestroy)
            handlers.append(onExecute)
            handlers.append(onDestroy)
            inputs = cmd.commandInputs
            global commandId
            selectionInput = inputs.addSelectionInput(commandId + '_selection', 'Select', 'Select a solid body')
            selectionInput.setSelectionLimits(1)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    try:
        global app
        app = adsk.core.Application.get()
        global ui
        ui = app.userInterface

        global commandId
        global commandName
        global commandDescription
        
        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)

        onCommandCreated = CalculateBoundingBoxCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        handlers.append(onCommandCreated)

        inputs = adsk.core.NamedValues.create()
        cmdDef.execute(inputs)

        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

