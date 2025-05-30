import unittest
import vtk
import numpy as np
import SimpleITK as sitk
import sitkUtils
import slicer
import PathPlanning_copy
from PathPlanning_copy import (
    PathPlanning_copyLogic,
    ComputeDistanceImageFromLabelMap,
    PickPointsMatrix
)


class TestPathPlanningCopy(unittest.TestCase):
    """Sanity‐check tests for the PathPlanning_copy module."""

    def setUp(self):
        slicer.mrmlScene.Clear(0)

    def test_api_surface(self):
        """Ensure key classes exist in the module."""
        for name in ("PathPlanning_copyLogic",
                     "ComputeDistanceImageFromLabelMap",
                     "PickPointsMatrix"):
            self.assertTrue(
                hasattr(PathPlanning_copy, name),
                f"Missing {name} in PathPlanning_copy"
            )

    def test_distance_map(self):
        """Compute a 3×3×3 Danielsson map and verify distances."""
        # create a labelmap with a single '1' at center
        arr = np.zeros((3,3,3), np.uint8)
        arr[1,1,1] = 1
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        img = vtk.vtkImageData()
        img.SetDimensions(3,3,3)
        img.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
        vtk_arr = vtk.util.numpy_support.numpy_to_vtk(
            arr.flatten(), deep=True, array_type=vtk.VTK_UNSIGNED_CHAR
        )
        img.GetPointData().SetScalars(vtk_arr)
        node.SetAndObserveImageData(img)

        # run the filter
        dm_node = ComputeDistanceImageFromLabelMap().Execute(node)
        sitk_img = sitkUtils.PullVolumeFromSlicer(dm_node)
        out = sitk.GetArrayFromImage(sitk_img)

        # center should be 0, one voxel away should be 1
        self.assertEqual(int(out[1,1,1]), 0)
        self.assertEqual(int(out[1,1,0]), 1)

    def test_pickpoints_matrix(self):
        """Verify PickPointsMatrix keeps only points inside the mask."""
        arr = np.zeros((3,3,3), np.uint8)
        arr[1,1,1] = 1
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        img = vtk.vtkImageData()
        img.SetDimensions(3,3,3)
        img.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
        vtk_arr = vtk.util.numpy_support.numpy_to_vtk(
            arr.flatten(), deep=True, array_type=vtk.VTK_UNSIGNED_CHAR
        )
        img.GetPointData().SetScalars(vtk_arr)
        node.SetAndObserveImageData(img)

        # set up fiducials: one inside, one outside
        fid_in = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        fid_in.AddControlPoint(1,1,1)
        fid_in.AddControlPoint(0,0,0)
        fid_out = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")

        PickPointsMatrix().run(node, fid_in, fid_out)
        self.assertEqual(fid_out.GetNumberOfControlPoints(), 1)
        # ensure the one kept is at (1,1,1)
        pos = [0.0,0.0,0.0]
        fid_out.GetNthControlPointPosition(0, pos)
        self.assertTupleEqual(tuple(pos), (1.0,1.0,1.0))

    def test_run_smoke(self):
        """Smoke‐test Run(): should return False with no valid image data."""
        logic = PathPlanning_copyLogic()

        # minimal fiducials
        e = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        e.AddControlPoint(0,0,0)
        t = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        t.AddControlPoint(0,0,0)

        # empty volumes (no image data)
        vol = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")

        out = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")

        logic.SetEntryFiducials(e)
        logic.SetTargetFiducials(t)
        logic.SetTargetVolume(vol)
        logic.SetCriticalVolume(vol)
        logic.SetLengthThreshold(100.0)
        logic.SetOutputFiducials(out)

        result = logic.Run()
        self.assertFalse(result, "Run() should fail with missing image data")


if __name__ == "__main__":
    unittest.main()
