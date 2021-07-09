import open3d as o3d
from modules.self_point_cloud import SelfPointCloud
from modules.self_logger import SelfLogger
from modules.registration_if import RegistrationIF


class Registration(RegistrationIF):
    def __init__(self, file_a, file_b, voxel_size):
        super(Registration, self).__init__(file_a, file_b, voxel_size)

    def refine_registration(self, source: SelfPointCloud, target: SelfPointCloud):
        distance_threshold = self.voxel_size * 0.4
        self._logger.debug(f":: Point-to-plane ICP registration is applied on original point")
        self._logger.debug(f"   clouds to refine the alignment. This time we use a strict")
        self._logger.debug(f"   distance threshold {distance_threshold:.3f}.")
        result = o3d.pipelines.registration.registration_icp(
            source.points, target.points, distance_threshold, self.result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return result


def main():
    Registration("../data/a.xyz", "../data/b.xyz", 0.05)


if __name__ == "__main__":
    main()
