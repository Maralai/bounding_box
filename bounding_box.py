import adsk.core, adsk.fusion, traceback
import numpy as np
from scipy.spatial import ConvexHull

app = adsk.core.Application.get()
ui = app.userInterface

def get_oriented_bbox(vertices, hull):
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

    return min_oriented_bbox

def display_tight_bounding_box_dimensions(solid_body):
    global ui
    try:
        volume, centroid = solid_body.physicalProperties.volume, solid_body.physicalProperties.centroid
        vertices = [np.array([v.geometry.x, v.geometry.y, v.geometry.z]) for m in solid_body.meshManager.displayMeshes for v in m.vertexBuffer]

        hull = ConvexHull(vertices)
        min_coords, max_coords, R = get_oriented_bbox(vertices, hull)
        box_size = max_coords - min_coords
        T, W, L = box_size[0], box_size[1], box_size[2]
        ui.statusBarText = f"Tight bounding box dimensions: T: {T:.2f} x W: {W:.2f} x L: {L:.2f}"
    except:
        ui.statusBarText = "Error calculating tight bounding box dimensions."

def run(context):
    try:
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            ui.messageBox('No active Fusion design', 'No Design')
            return

        selection = ui.selectEntity("Select a solid body", "SolidBodies")
        if selection is None:
            return

        solid_body = adsk.fusion.BRepBody.cast(selection)
        if solid_body is None:
            ui.messageBox("Selected object is not a solid body.")
            return

        display_tight_bounding_box_dimensions(solid_body)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
