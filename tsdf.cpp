// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char * argv[])
{
  std::string tsdfDirectory = ".";
  if (argc != 2) {
		std::cout << "usage: ./tsdf <TSDF binary file directory>. File should be named tsdf.bin. Default is current folder."
				<< std::endl;
	} else {
		tsdfDirectory = std::string(argv[1]);
	}
  std::string tsdfName = tsdfDirectory + "/tsdf.bin";
  FILE * fp = fopen(tsdfName.c_str(), "r");
  if(!fp){
    std::cerr << "File not found" << std::endl;
    return 1;
  }
}
