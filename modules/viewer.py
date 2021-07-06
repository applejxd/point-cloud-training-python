import open3d as o3d
import pandas as pd
import dask.dataframe as dd


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Down sample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


class CloudCompare:
    def __init__(self):
        self.voxel_size = 0.05
        self.cloud_a, self.cloud_b = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
        self.cloud_a_down, self.cloud_a_fpfh = None, None
        self.cloud_b_down, self.cloud_b_fpfh = None, None
        self.read_xyz()

        self.result_ransac = self.execute_global_registration(
            self.cloud_a_down, self.cloud_b_down, self.cloud_a_fpfh, self.cloud_b_fpfh)
        print(self.result_ransac)
        # o3d.visualization.draw_geometries(
        #     [self.cloud_a.transform(self.result_ransac.transformation), self.cloud_b])

        result_icp = self.refine_registration(self.cloud_a, self.cloud_b)
        o3d.visualization.draw_geometries(
            [self.cloud_a.transform(result_icp.transformation), self.cloud_b])

    def pre_process(self, cloud, df):
        cloud.points = o3d.utility.Vector3dVector(df[["x", "y", "z"]].to_numpy())
        cloud.colors = o3d.utility.Vector3dVector(df[["r", "g", "b"]].to_numpy() / 256)
        cloud_down, fpfh = preprocess_point_cloud(cloud, self.voxel_size)
        return cloud, cloud_down, fpfh

    def read_xyz(self):
        a_df: pd.DataFrame = dd.read_csv(
            "../data/a.xyz", names=["x", "y", "z", "r", "g", "b"]).compute()
        b_df: pd.DataFrame = dd.read_csv(
            "../data/b.xyz", names=["x", "y", "z", "r", "g", "b"]).compute()

        self.cloud_a, self.cloud_a_down, self.cloud_a_fpfh = self.pre_process(self.cloud_a, a_df)
        self.cloud_b, self.cloud_b_down, self.cloud_b_fpfh = self.pre_process(self.cloud_b, b_df)

        o3d.visualization.draw_geometries([self.cloud_a, self.cloud_b])

    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh):
        distance_threshold = self.voxel_size * 1.5
        print(":: RANSAC registration on downsampled point clouds.")
        print("   Since the downsampling voxel size is %.3f," % self.voxel_size)
        print("   we use a liberal distance threshold %.3f." % distance_threshold)
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
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_icp(
            cloud_a, cloud_b, distance_threshold, self.result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return result


if __name__ == "__main__":
    CloudCompare()
