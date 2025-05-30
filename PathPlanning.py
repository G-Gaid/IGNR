import logging
import os
import time
from typing import Annotated, List, Tuple
import vtk
import numpy as np
import SimpleITK as sitk
import sitkUtils
import slicer
import qt
import ctk
from slicer.util import VTKObservationMixin
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import (
    ScriptedLoadableModule,
    ScriptedLoadableModuleWidget,
    ScriptedLoadableModuleLogic,
    ScriptedLoadableModuleTest,
)
from slicer.parameterNodeWrapper import parameterNodeWrapper, WithinRange
from slicer import vtkMRMLScalarVolumeNode, vtkMRMLMarkupsFiducialNode

#
# Module declaration
#

class PathPlanning_copy(ScriptedLoadableModule):
    """IGNR PathPlanning_copy module."""
    def __init__(self, parent):
        super().__init__(parent)
        self.parent.title = _("PathPlanning_copy")
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "IGNR")]
        self.parent.dependencies = []
        self.parent.contributors = ["George Gaid (King's College London)"]
        self.parent.helpText = _("""
This module implements Path Planning for the IGNR Final project.
It allows users to define entry/target points and critical/target volumes,
to thereby compute the optimal trajectories.
""")

        self.parent.acknowledgementText = _("""
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.  Rachel Sparks has modified this, 
for part of Image-guide Navigation for Robotics taught through King's College London.
""")


def registerSampleData():
    """Add sample datasets to the Sample Data module."""
    from SampleData import SampleDataLogic
    iconsPath = os.path.join(os.path.dirname(__file__), "Resources", "Icons")
    SampleDataLogic.registerCustomSampleDataSource(
        category="PathPlanning_copy",
        sampleName="ExampleBrain",
        thumbnailFileName=os.path.join(iconsPath, "ExampleBrain.png"),
        uris="https://example.com/ExampleBrain.nrrd",
        fileNames="ExampleBrain.nrrd",
        checksums="SHA256:...",
        nodeNames="Example Brain Volume",
    )


#
# Parameter node
#

@parameterNodeWrapper
class PathPlanning_copyParameterNode:
    """Holds all user parameters for PathPlanning_copy."""
    inputEntryFiducials:  vtkMRMLMarkupsFiducialNode
    inputTargetFiducials: vtkMRMLMarkupsFiducialNode
    inputTargetVolume:    vtkMRMLScalarVolumeNode
    inputCriticalVolume:  vtkMRMLScalarVolumeNode
    lengthThreshold:      Annotated[float, WithinRange(0,1000)] = 150.0
    outputFiducials:      vtkMRMLMarkupsFiducialNode


#
# Widget
#

class PathPlanning_copyWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """GUI for the PathPlanning_copy module."""
    def setup(self):
        super().setup()

        # Parameters section
        collapsible = ctk.ctkCollapsibleButton()
        collapsible.text = _("Parameters")
        self.layout.addWidget(collapsible)
        formLayout = qt.QFormLayout(collapsible)

        # Entry points selector
        self.entrySelector = slicer.qMRMLNodeComboBox()
        self.entrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.entrySelector.selectNodeUponCreation = True
        self.entrySelector.noneEnabled = False
        self.entrySelector.setMRMLScene(slicer.mrmlScene)
        formLayout.addRow(_("Entry Points:"), self.entrySelector)

        # Target points selector
        self.targetSelector = slicer.qMRMLNodeComboBox()
        self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.targetSelector.selectNodeUponCreation = True
        self.targetSelector.noneEnabled = False
        self.targetSelector.setMRMLScene(slicer.mrmlScene)
        formLayout.addRow(_("Target Points:"), self.targetSelector)

        # Target volume selector
        self.targetVolSelector = slicer.qMRMLNodeComboBox()
        self.targetVolSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
        self.targetVolSelector.selectNodeUponCreation = True
        self.targetVolSelector.noneEnabled = False
        self.targetVolSelector.setMRMLScene(slicer.mrmlScene)
        formLayout.addRow(_("Target Volume:"), self.targetVolSelector)

        # Critical volume selector
        self.critVolSelector = slicer.qMRMLNodeComboBox()
        self.critVolSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
        self.critVolSelector.selectNodeUponCreation = True
        self.critVolSelector.noneEnabled = False
        self.critVolSelector.setMRMLScene(slicer.mrmlScene)
        formLayout.addRow(_("Critical Volume:"), self.critVolSelector)

        # Output trajectory selector
        self.outputSelector = slicer.qMRMLNodeComboBox()
        self.outputSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.outputSelector.selectNodeUponCreation = True
        self.outputSelector.noneEnabled = False
        self.outputSelector.setMRMLScene(slicer.mrmlScene)
        formLayout.addRow(_("Output Trajectory:"), self.outputSelector)

        # Length threshold
        self.lengthSlider = qt.QDoubleSpinBox()
        self.lengthSlider.minimum = 0.0
        self.lengthSlider.maximum = 1000.0
        self.lengthSlider.value = 150.0
        formLayout.addRow(_("Length Threshold (mm):"), self.lengthSlider)

        # Apply button
        self.applyButton = qt.QPushButton(_("Apply"))
        self.applyButton.enabled = False
        formLayout.addWidget(self.applyButton)

        # Connect signals
        selectors = [
            self.entrySelector, self.targetSelector,
            self.targetVolSelector, self.critVolSelector,
            self.outputSelector
        ]
        for sel in selectors:
            sel.currentNodeChanged.connect(self._checkCanApply)
        self.applyButton.clicked.connect(self.onApplyButton)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent,   self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent,     self.onSceneEndClose)

        # Logic + parameter node
        self.logic = PathPlanning_copyLogic()
        self.initializeParameterNode()

    def initializeParameterNode(self):
        """Instantiate parameter node and set defaults."""
        self._parameterNode = self.logic.getParameterNode()
        p = self._parameterNode
        defaults = [
            (self.entrySelector, "inputEntryFiducials"),
            (self.targetSelector, "inputTargetFiducials"),
            (self.targetVolSelector, "inputTargetVolume"),
            (self.critVolSelector, "inputCriticalVolume"),
        ]
        for widget, attr in defaults:
            if not getattr(p, attr):
                node = slicer.mrmlScene.GetFirstNodeByClass(widget.nodeTypes[0])
                if node:
                    setattr(p, attr, node)
        if not p.outputFiducials:
            out = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
            out.SetName("PlannedTrajectories")
            p.outputFiducials = out
        self._checkCanApply()

    def _checkCanApply(self, *args):
        """Enable Apply only when all inputs/outputs set."""
        p = self._parameterNode
        ready = all([
            p and p.inputEntryFiducials and p.inputTargetFiducials,
            p.inputTargetVolume and p.inputCriticalVolume,
            p.outputFiducials
        ])
        self.applyButton.enabled = ready

    def onApplyButton(self):
        """Gather parameters and run logic."""
        p = self._parameterNode
        self.logic.SetEntryFiducials(p.inputEntryFiducials)
        self.logic.SetTargetFiducials(p.inputTargetFiducials)
        self.logic.SetTargetVolume(p.inputTargetVolume)
        self.logic.SetCriticalVolume(p.inputCriticalVolume)
        self.logic.SetLengthThreshold(self.lengthSlider.value)
        self.logic.SetOutputFiducials(p.outputFiducials)
        startTime = time.time()
        success = self.logic.Run()
        elapsed = time.time() - startTime
        logging.info(f"Path planning completed in {elapsed:.2f}s")
        if not success:
            slicer.util.errorDisplay(_("Path planning failed; see log."))

    def onSceneStartClose(self, *args):
        """Reset parameter node when scene closes."""
        self._parameterNode = None

    def onSceneEndClose(self, *args):
        """Re-initialize when scene opens."""
        if self.parent.isEntered:
            self.initializeParameterNode()


#
# Logic
#

class PathPlanning_copyLogic(ScriptedLoadableModuleLogic):
    """Core algorithm: enumerate, filter, rank by Danielsson distance."""
    def __init__(self):
        super().__init__()
        self._entryFid = None
        self._targetFid = None
        self._targetVol = None
        self._critVol = None
        self._lengthThresh = None
        self._outputFid = None
        self._distanceVol = None

    def getParameterNode(self):
        return PathPlanning_copyParameterNode(super().getParameterNode())

    # Setters
    def SetEntryFiducials(self, node):
        self._entryFid = node

    def SetTargetFiducials(self, node):
        self._targetFid = node

    def SetTargetVolume(self, node):
        self._targetVol = node

    def SetCriticalVolume(self, node):
        self._critVol = node

    def SetLengthThreshold(self, t):
        self._lengthThresh = t

    def SetOutputFiducials(self, node):
        self._outputFid = node

    def isValidInputOutputData(self) -> bool:
        """Ensure all required nodes are present and distinct."""
        if not all([
            self._entryFid, self._targetFid,
            self._targetVol, self._critVol,
            self._outputFid
        ]):
            logging.debug("Invalid I/O: some nodes are missing")
            return False
        if self._outputFid.GetID() in (
            self._entryFid.GetID(), self._targetFid.GetID()
        ):
            logging.debug("Invalid I/O: output fiducial collides with input")
            return False
        return True

    def Run(self) -> bool:
        """Enumerate entry–target pairs, apply constraints, pick best by clearance."""
        if not self.isValidInputOutputData():
            return False
        if not (
            self._targetVol.GetImageData()
            and self._critVol.GetImageData()
        ):
            logging.error("Volume image data missing")
            return False

        # Compute Danielsson distance map
        self._distanceVol = ComputeDistanceImageFromLabelMap().Execute(self._critVol)

        candidates: List[Tuple[float, Tuple[float,float,float], Tuple[float,float,float]]] = []
        for e, t in self._allPairs():
            if not self._inTarget(t):
                continue
            if not self._collisionCheck(e, t):
                continue
            dist = self._minDistance(e, t)
            if dist <= 0 or self._euclidean(e, t) > self._lengthThresh:
                continue
            candidates.append((dist, e, t))

        if not candidates:
            logging.warning("No valid trajectories found")
            return False

        # Choose maximum-clearance path
        _, bestE, bestT = max(candidates, key=lambda x: x[0])
        self._outputFid.RemoveAllControlPoints()
        self._outputFid.AddControlPoint(*bestE)
        self._outputFid.AddControlPoint(*bestT)
        logging.info("Selected best trajectory")
        return True

    def _allPairs(self) -> List[Tuple[Tuple[float,float,float], Tuple[float,float,float]]]:
        pts = []
        for i in range(self._entryFid.GetNumberOfControlPoints()):
            e = [0.0, 0.0, 0.0]
            self._entryFid.GetNthControlPointPosition(i, e)
            for j in range(self._targetFid.GetNumberOfControlPoints()):
                t = [0.0, 0.0, 0.0]
                self._targetFid.GetNthControlPointPosition(j, t)
                pts.append((tuple(e), tuple(t)))
        return pts

    def _euclidean(self, p1, p2) -> float:
        return vtk.vtkMath.Distance2BetweenPoints(p1, p2) ** 0.5

    def _inTarget(self, pt) -> bool:
        tmpIn = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        tmpOut = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        tmpIn.AddControlPoint(pt)
        PickPointsMatrix().run(self._targetVol, tmpIn, tmpOut)
        inside = tmpOut.GetNumberOfControlPoints() > 0
        slicer.mrmlScene.RemoveNode(tmpIn)
        slicer.mrmlScene.RemoveNode(tmpOut)
        return inside

    def _collisionCheck(self, p1, p2) -> bool:
        mc = vtk.vtkDiscreteMarchingCubes()
        mc.SetInputData(self._critVol.GetImageData())
        mc.SetValue(0, 1)
        mc.Update()
        tree = vtk.vtkOBBTree()
        tree.SetDataSet(mc.GetOutput())
        tree.BuildLocator()
        pts = vtk.vtkPoints()
        ids = vtk.vtkIdList()
        hit = tree.IntersectWithLine(p1, p2, pts, ids) != 0
        return not hit

    def _minDistance(self, p1, p2, samples: int = 50) -> float:
        sitkImg = sitkUtils.PullVolumeFromSlicer(self._distanceVol)
        arr = sitk.GetArrayFromImage(sitkImg)
        tfm = vtk.vtkMatrix4x4()
        inv = vtk.vtkMatrix4x4()
        self._distanceVol.GetIJKToRASMatrix(tfm)
        vtk.vtkMatrix4x4.Invert(tfm, inv)
        minDist = float("inf")
        for k in range(samples + 1):
            alpha = k / samples
            ras = [p1[i] + alpha * (p2[i] - p1[i]) for i in range(3)] + [1.0]
            ijk = inv.MultiplyPoint(ras)
            i, j, k_ = map(int, map(round, ijk[:3]))
            if not (
                0 <= k_ < arr.shape[0]
                and 0 <= j < arr.shape[1]
                and 0 <= i < arr.shape[2]
            ):
                return -1.0
            minDist = min(minDist, float(arr[k_, j, i]))
        return minDist


#
# Helpers
#

class ComputeDistanceImageFromLabelMap:
    """Compute Danielsson distance map from a binary mask volume."""
    def Execute(self, vol: vtkMRMLScalarVolumeNode) -> vtkMRMLScalarVolumeNode:
        sitkImg = sitkUtils.PullVolumeFromSlicer(vol)
        dmap = sitk.DanielssonDistanceMapImageFilter()
        outSitk = dmap.Execute(sitkImg)
        return sitkUtils.PushVolumeToSlicer(
            outSitk, None, vol.GetName() + "_dist"
        )


class PickPointsMatrix:
    """Mask-based point inclusion via numpy sampling."""
    def run(self, vol, inF, outF):
        outF.RemoveAllControlPoints()
        img = vol.GetImageData()
        dims = img.GetDimensions()
        scalars = img.GetPointData().GetScalars()
        arr = vtk.util.numpy_support.vtk_to_numpy(scalars).reshape(
            dims[2], dims[1], dims[0]
        )
        mat = vtk.vtkMatrix4x4()
        vol.GetRASToIJKMatrix(mat)
        for idx in range(inF.GetNumberOfControlPoints()):
            ras = [0.0, 0.0, 0.0]
            inF.GetNthControlPointPosition(idx, ras)
            ijk = mat.MultiplyPoint(ras + [1.0])
            x, y, z = map(int, map(round, ijk[:3]))
            if (
                0 <= x < dims[0]
                and 0 <= y < dims[1]
                and 0 <= z < dims[2]
                and arr[z, y, x] == 1
            ):
                outF.AddControlPoint(*ras)


#
# Testing
#

class PathPlanning_copyTest(ScriptedLoadableModuleTest):
    """Integration tests for PathPlanning_copy."""
    def setUp(self):
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        self.setUp()
        self.test_distance_map()
        self.setUp()
        self.test_pickpoints()
        self.setUp()
        self.test_full_smoke()

    def test_distance_map(self):
        self.delayDisplay("Testing Danielsson distance map")
        import numpy as np
        arr = np.zeros((3, 3, 3), np.uint8)
        arr[1, 1, 1] = 1
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        vtk_img = vtk.vtkImageData()
        vtk_img.SetDimensions(3, 3, 3)
        vtk_img.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
        vtk_arr = vtk.util.numpy_support.numpy_to_vtk(
            arr.flatten(), True, vtk.VTK_UNSIGNED_CHAR
        )
        vtk_img.GetPointData().SetScalars(vtk_arr)
        node.SetAndObserveImageData(vtk_img)
        dm = ComputeDistanceImageFromLabelMap().Execute(node)
        out = sitk.GetArrayFromImage(sitkUtils.PullVolumeFromSlicer(dm))
        self.assertEqual(int(out[1, 1, 1]), 0)
        self.assertEqual(int(out[1, 1, 0]), 1)

    def test_pickpoints(self):
        self.delayDisplay("Testing PickPointsMatrix")
        import numpy as np
        arr = np.zeros((3, 3, 3), np.uint8)
        arr[1, 1, 1] = 1
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        vtk_img = vtk.vtkImageData()
        vtk_img.SetDimensions(3, 3, 3)
        vtk_img.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
        vtk_arr = vtk.util.numpy_support.numpy_to_vtk(
            arr.flatten(), True, vtk.VTK_UNSIGNED_CHAR
        )
        vtk_img.GetPointData().SetScalars(vtk_arr)
        node.SetAndObserveImageData(vtk_img)
        fid_in = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        fid_in.AddControlPoint(1, 1, 1)
        fid_in.AddControlPoint(0, 0, 0)
        fid_out = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        PickPointsMatrix().run(node, fid_in, fid_out)
        self.assertEqual(fid_out.GetNumberOfControlPoints(), 1)

    def test_full_smoke(self):
        self.delayDisplay("Testing Run() smoke")
        logic = PathPlanning_copyLogic()
        e = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        e.AddControlPoint(0, 0, 0)
        t = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        t.AddControlPoint(4, 4, 4)
        vol = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        out = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        logic.SetEntryFiducials(e)
        logic.SetTargetFiducials(t)
        logic.SetTargetVolume(vol)
        logic.SetCriticalVolume(vol)
        logic.SetLengthThreshold(100.0)
        logic.SetOutputFiducials(out)
        self.assertFalse(logic.Run())
