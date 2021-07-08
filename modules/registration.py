import open3d as o3d
from modules.cloud_reader import CloudReader
from modules.self_logger import SelfLogger


class Registration:
    def __init__(self, file_a, file_b, voxel_size):
        self._logger = SelfLogger.get_logger(__file__)
        self.voxel_size = voxel_size
        self.cloud_a = CloudReader(file_a, voxel_size)
        self.cloud_b = CloudReader(file_b, voxel_size)

        self.result_ransac = self.execute_global_registration(
            self.cloud_a.points_down, self.cloud_b.points_down,
            self.cloud_a.fpfh, self.cloud_b.fpfh)

        self.result_icp = self.refine_registration(self.cloud_a.points, self.cloud_b.points)

        o3d.visualization.draw_geometries(
            [self.cloud_a.points.transform(self.result_icp.transformation), self.cloud_b.points])

    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh):
        distance_threshold = self.voxel_size * 1.5
        self._logger.debug(f":: RANSAC registration on downsampled point clouds.")
        self._logger.debug(f"   Since the downsampling voxel size is {self.voxel_size:.3f},")
        self._logger.debug(f"   we use a liberal distance threshold {distance_threshold:.3f}.")
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result

    def refine_registration(self, cloud_a, cloud_b):
        distance_threshold = self.voxel_size * 0.4
        self._logger.debug(f":: Point-to-plane ICP registration is applied on original point")
        self._logger.debug(f"   clouds to refine the alignment. This time we use a strict")
        self._logger.debug(f"   distance threshold {distance_threshold:.3f}.")
        result = o3d.pipelines.registration.registration_icp(
            cloud_a, cloud_b, distance_threshold, self.result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return result


def main():
    Registration("../data/a.xyz", "../data/b.xyz", 0.05)


if __name__ == "__main__":
    main()
