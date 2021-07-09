import dask.dataframe as dd
import open3d as o3d
from modules.self_logger import SelfLogger


class SelfPointCloud:
    def __init__(self, file_path, voxel_size):
        self._logger = SelfLogger.get_logger(__file__)
        self._file_path = file_path
        self.voxel_size = voxel_size

        self.points = self._read_xyz()
        self.points_down = self._down_sampling(self.points)
        self.fpfh_down = self._calc_fpfh(self.points_down)
        self.tree = o3d.geometry.KDTreeFlann(self.points)

    def _read_xyz(self):
        df = dd.read_csv(self._file_path, names=["x", "y", "z", "r", "g", "b"]).compute()
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(df[["x", "y", "z"]].to_numpy())
        cloud.colors = o3d.utility.Vector3dVector(df[["r", "g", "b"]].to_numpy() / 256)
        return cloud

    def _down_sampling(self, cloud):
        self._logger.debug(f"Down sample with a voxel size {self.voxel_size:.3f}.")
        cloud_down = cloud.voxel_down_sample(self.voxel_size)

        radius_normal = self.voxel_size * 2
        self._logger.debug(f"Estimate normal with search radius {radius_normal:.3f}.")
        cloud_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        return cloud_down

    def _calc_fpfh(self, cloud_down):
        radius_feature = self.voxel_size * 5
        self._logger.debug(f":: Compute FPFH feature with search radius {radius_feature:.3f}.")
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            cloud_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return fpfh


def main():
    cloud_a = SelfPointCloud("../data/a.xyz", 0.05)
    cloud_b = SelfPointCloud("../data/b.xyz", 0.05)
    o3d.visualization.draw_geometries([cloud_a.points, cloud_b.points])


if __name__ == "__main__":
    main()
