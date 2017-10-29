// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
void connectedComponents(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int startPoint[3], int matrixSize[3])
{
    // Set up grid in heap
    // 'e': empty; 'o': occupied, 'v': visited
    std::vector<std::vector<std::vector<char>>> grid;
    grid.resize((unsigned) matrixSize[0]);
    for (int i = 0; i < matrixSize[0]; i++) {
        grid[i].resize((unsigned) matrixSize[1]);

        for (int j = 0; j < matrixSize[1]; j++) {
            grid[i][j].resize((unsigned) matrixSize[2]);

            for (int k = 0; k < matrixSize[2];k++) {
                grid[i][j][k] = 'e';
            }
        }
    }
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it!= cloud->end(); it++){
        // std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
    }
}

int main (int argc, char * argv[])
{
    float tsdfval1 = 0;
    float tsdfval2 = 0;
    int minX = 512,maxX = -1,minY = 512,maxY = -1,minZ = 512,maxZ = -1;
    std::string tsdfDirectory = ".";

    // Loading files
    if (argc != 2) {
		std::cout << "usage: ./tsdf <TSDF binary file directory>."
                  << std::endl
                  << "File should be named tsdf.bin and tsdf2.bin. Default is current folder."
                  << std::endl;
	} else {
		tsdfDirectory = std::string(argv[1]);
	}
    std::string tsdfName = tsdfDirectory + "/tsdf.bin";
    std::string tsdfName2 = tsdfDirectory + "/tsdf2.bin";
    FILE * fp = fopen(tsdfName.c_str(), "r");
    FILE * fp2 = fopen(tsdfName2.c_str(), "r");
    if(!fp || !fp2){
        std::cerr << "Files not found. Please check your file names and directories." << std::endl;
        return 1;
    }


    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud);
    int count = 0;
    for (int i = 0; i < 512; i++) {
        for (int j = 0; j < 512; j++){
            for (int k = 0; k < 512; k++){
                if(fread((void*)(&tsdfval1), sizeof(tsdfval1), 1, fp)) {
                    // Naively add point if value in the first cloud is positive & in the second cloud is negative.
                    if(fread((void*)(&tsdfval2), sizeof(tsdfval2), 1, fp2)){
                        if(tsdfval1 > 0 && tsdfval2 < 0){
                            cloud.push_back(pcl::PointXYZ(i, j, k));
                            minX = (i < minX) ? i : minX;
                            minY = (j < minY) ? j : minY;
                            minZ = (k < minZ) ? k : minZ;
                            maxX = (i > maxX) ? i : maxX;
                            maxY = (j > maxY) ? j : maxY;
                            maxZ = (k > maxZ) ? k : maxZ;
                            count ++;
                        }
                    } else {
                        std::cerr << "tsdf2.bin is corrupted. TSDF format should be a 512*512*512 float array." << std::endl;
                        return 1;
                    }
                } else {
                    std::cerr << "tsdf.bin is corrupted. TSDF format should be a 512*512*512 float array." << std::endl;
                    return 1;
                }
            }
        }
    }
    std::cout << "TSDF point cloud generated." << std::endl;

    // Calculate size of the matrix to perform operations on
    int startPoint[3] = {minX, minY, minZ};
    int matrixSize[3] = {maxX-minX+1, maxY-minY+1, maxZ-minZ+1};
    // Run connected component
    connectedComponents(ptrCloud, startPoint, matrixSize);

    /**
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(ptrCloud);
    while (!viewer.wasStopped ())
    {

    }
     **/
    return (0);
}
