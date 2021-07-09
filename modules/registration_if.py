from abc import ABC, abstractmethod
from modules.self_logger import SelfLogger
from modules.self_point_cloud import SelfPointCloud
import open3d as o3d


class RegistrationIF(ABC):
    def __init__(self, file_a, file_b, voxel_size):
        self._logger = SelfLogger.get_logger(__file__)
        self.voxel_size = voxel_size
        self.cloud_a = SelfPointCloud(file_a, voxel_size)
        self.cloud_b = SelfPointCloud(file_b, voxel_size)

        self.result_ransac = self.execute_global_registration(self.cloud_a, self.cloud_b)
        self.result_icp = self.refine_registration(self.cloud_a, self.cloud_b)
        o3d.visualization.draw_geometries(
            [self.cloud_a.points.transform(self.result_icp.transformation), self.cloud_b.points])

    def execute_global_registration(self, source: SelfPointCloud, target: SelfPointCloud):
        distance_threshold = self.voxel_size * 1.5
        self._logger.debug(f":: RANSAC registration on downsampled point clouds.")
        self._logger.debug(f"   Since the downsampling voxel size is {self.voxel_size:.3f},")
        self._logger.debug(f"   we use a liberal distance threshold {distance_threshold:.3f}.")
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source.points_down, target.points_down, source.fpfh_down, target.fpfh_down, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result

    @abstractmethod
    def refine_registration(self, source: SelfPointCloud, target: SelfPointCloud):
        pass
