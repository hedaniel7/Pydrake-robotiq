import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
    Parser,
    RigidTransform,
    RotationMatrix,
    StartMeshcat,
)

from manipulation.scenarios import SetColor

# Start the visualizer.
meshcat = StartMeshcat()

def grasp_poses_example():
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    grasp = parser.AddModelsFromUrl(
        "package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
    )[0]
    pregrasp = parser.AddModelsFromUrl(
        "package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
    )[0]
    brick = parser.AddModelsFromUrl(
        "package://drake/examples/manipulation_station/models/061_foam_brick.sdf"
    )[0]
    plant.Finalize()

    B_O = plant.GetBodyByName("base_link", brick)
    B_Ggrasp = plant.GetBodyByName("body", grasp)
    B_Gpregrasp = plant.GetBodyByName("body", pregrasp)

    # Set the pregrasp to be green and slightly transparent.
    inspector = scene_graph.model_inspector()
    for body_index in plant.GetBodyIndices(pregrasp):
        SetColor(
            scene_graph,
            [0, 0.6, 0, 0.5],
            plant.get_source_id(),
            inspector.GetGeometries(plant.GetBodyFrameIdOrThrow(body_index)),
        )

    meshcat.Delete()
    meshcat.SetProperty("/Background", "visible", False)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    # TODO(russt): Set a random pose of the object.

    # Get the current object, O, pose
    X_WO = plant.EvalBodyPoseInWorld(plant_context, B_O)

    p_GgraspO = [0, 0.11, 0]
    R_GgraspO = RotationMatrix.MakeXRotation(np.pi / 2.0).multiply(
        RotationMatrix.MakeZRotation(np.pi / 2.0)
    )
    X_GgraspO = RigidTransform(R_GgraspO, p_GgraspO)
    X_OGgrasp = X_GgraspO.inverse()
    X_WGgrasp = X_WO.multiply(X_OGgrasp)

    # pregrasp is negative y in the gripper frame (see the figure!).
    X_GgraspGpregrasp = RigidTransform([0, -0.08, 0])
    X_WGpregrasp = X_WGgrasp @ X_GgraspGpregrasp

    plant.SetFreeBodyPose(plant_context, B_Ggrasp, X_WGgrasp)
    # Open the fingers, too.
    plant.GetJointByName("left_finger_sliding_joint", grasp).set_translation(
        plant_context, -0.054
    )
    plant.GetJointByName("right_finger_sliding_joint", grasp).set_translation(
        plant_context, 0.054
    )

    plant.SetFreeBodyPose(plant_context, B_Gpregrasp, X_WGpregrasp)
    # Open the fingers, too.
    plant.GetJointByName("left_finger_sliding_joint", pregrasp).set_translation(
        plant_context, -0.054
    )
    plant.GetJointByName("right_finger_sliding_joint", pregrasp).set_translation(
        plant_context, 0.054
    )

    diagram.ForcedPublish(context)


grasp_poses_example()