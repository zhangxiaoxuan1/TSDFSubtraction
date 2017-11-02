class MarchingCubes
{
public:
    static void marchingCube(int matrixSize[3], std::vector<std::vector<std::vector<short int>>>& grid,
                             std::vector<std::vector<std::vector<float>>>& tsdfGrid, int index,pcl::PointCloud<pcl::PointXYZRGB>::Ptr componentCloud);
};
