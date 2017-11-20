//
// Created by motion on 10/28/17.
//

// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char * argv[])
{
    float tsdfval = 0;
    float tsdfArr[512];
    int radius = 25;
    int location[3] = {250,235,50};
    double dis;
    double tsdfval_sphere;
    std::string tsdfDirectory = ".";

    // Loading files
    if (argc != 2) {
		std::cout << "usage: ./tsdf <TSDF binary file directory>."
                  << std::endl
                  << "File should be named tsdf.bin. Default is current folder."
                  << std::endl;
	} else {
		tsdfDirectory = std::string(argv[1]);
	}
    std::string tsdfName = tsdfDirectory + "/tsdf.bin";
    std::string destName = tsdfDirectory + "/tsdf_new.bin";
    FILE * fp = fopen(tsdfName.c_str(), "r");
    FILE * wfp = fopen(destName.c_str(), "wb+");
    if(!fp){
        std::cerr << "Files not found. Please check your file names and directories." << std::endl;
        return 1;
    }


    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud);
    for (int i = 0; i < 512; i++) {
        for (int j = 0; j < 512; j++){
            for (int k = 0; k < 512; k++){
                if(fread((void*)(&tsdfval), sizeof(tsdfval), 1, fp)) {
                    dis = pow((i-location[0])*(i-location[0])+(j-location[1])*(j-location[1])+(k-location[2])*(k-location[2]),0.5);
                    tsdfval_sphere = dis/radius - 1;
                    tsdfArr[k] = std::min(tsdfval,(float) tsdfval_sphere);
                } else {
                    std::cerr << "tsdf.bin is corrupted. TSDF format should be a 512*512*512 float array." << std::endl;
                    return 1;
                }
            }
            fwrite(tsdfArr, sizeof(float) * 512, 1, wfp);
        }
    }
    std::cout << "new TSDF file generated." << std::endl;
    return 0;
}
