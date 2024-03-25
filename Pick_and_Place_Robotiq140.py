import mpld3
import numpy as np
from matplotlib import pyplot as plt
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    AngleAxis,
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    MeshcatVisualizer,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Parser,
    PiecewisePolynomial,
    PiecewisePose,
    Quaternion,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
    TrajectorySource,
)

from manipulation import running_as_notebook
from manipulation.station import MakeHardwareStation, load_scenario
from manipulation.utils import RenderDiagram

if running_as_notebook:
    mpld3.enable_notebook()


###


meshcat = StartMeshcat()