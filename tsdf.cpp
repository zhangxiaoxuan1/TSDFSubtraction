// include required libraries
#include <functional>
#include <queue>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "marching_cubes.h"

uint8_t rgbTable[20][3] = {{230, 25, 75},{60, 180, 75},{255, 225, 25},{0, 130, 200},{245, 130, 48},{145, 30, 180},
                       {70, 240, 240},{240, 50, 230},{210, 245, 60},{250, 190, 190},{0, 128, 128},{230, 190, 255},
                       {170, 110, 40},{255, 250, 200},{128, 0, 0},{170, 255, 195},{128, 128, 0},{255, 215, 180},
                       {0, 0, 128},{128, 128, 128}};
// Red, green,yellow,blue,orange,purple,teal,pink,yellow/green,light pink,green/blue,pink/purple,brown,light yellow,
// dark brown,light green,brown/green,pink/yellow,dark blue, grey
float threshold = 0.7;

int dfs (std::vector<std::vector<std::vector<short int>>>& grid,int i, int j, int k, short int index)
{
    if (i < 0 || i >= grid.size() || j < 0 || j >= grid[0].size() || k < 0 || k >= grid[0][0].size() ){
        return 0;
    }


    // Only count if not visited
    if(grid[i][j][k] != 1){
        return 0;
    }
    // Change value of the cell
    grid[i][j][k] = index;
    // Call dfs to iterate through its neighbors
    return 1 + dfs(grid,i,j,k-1,index) + dfs(grid,i,j,k+1,index) + dfs(grid,i,j-1,k,index)
           + dfs(grid,i,j+1,k,index) + dfs(grid,i+1,j,k,index) + dfs(grid,i-1,j,k,index);

}
bool comp (std::vector<int>& i,std::vector<int>& j) { return (i[1]>j[1]); }

void connectedComponents(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int startPoint[3], int matrixSize[3], std::vector<std::vector<std::vector<float>>>& tsdfGrid)
{
    // Set up grid in heap
    // 0: empty; 1: occupied, >1: visited
    std::vector<std::vector<std::vector<short int>>> grid;
    std::vector<std::vector<int>> componentArr;
    grid.resize((unsigned) matrixSize[0]);
    for (int i = 0; i < matrixSize[0]; i++) {
        grid[i].resize((unsigned) matrixSize[1]);

        for (int j = 0; j < matrixSize[1]; j++) {
            grid[i][j].resize((unsigned) matrixSize[2]);

            for (int k = 0; k < matrixSize[2];k++) {
                grid[i][j][k] = 0;
            }
        }
    }


    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it!= cloud->end(); it++){
        grid[it->x - startPoint[0]][it->y - startPoint[1]][it->z - startPoint[2]] = 1;
    }

    std::cout << "Running connected components..." << std::endl;
    int index = 1;
    int count = 0;
    for (int i = 0; i < matrixSize[0]; i++) {
        for (int j = 0; j < matrixSize[1]; j++) {
            for (int k = 0; k < matrixSize[2];k++) {
                if(grid[i][j][k] == 1){
                    index++;
                    std::vector<int> temp;
                    temp.push_back(index);
                    temp.push_back(dfs(grid, i,j,k, index));
                    componentArr.push_back(temp);
                }
            }
        }
    }
    // Sort the array
    std::sort (componentArr.begin(), componentArr.end(), comp);
    int target = componentArr[0][0];

    std::cout << "Finished Connected Components. The top 10 components are:" << componentArr[0][0] << ","
    << componentArr[1][0] << "," << componentArr[2][0] << "," << componentArr[3][0] << "," << componentArr[4][0]
    << "," << componentArr[5][0] << "," << componentArr[6][0] << "," << componentArr[7][0] << "," << componentArr[8][0] << "," << componentArr[9][0]<< std::endl;
    target = 585;
    // Add points to the point cloud for visualization
    pcl::PointCloud<pcl::PointXYZRGB> componentCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrComponentCloud(&componentCloud);
    for (int i = 0; i < matrixSize[0]; i++) {
        for (int j = 0; j < matrixSize[1]; j++) {
            for (int k = 0; k < matrixSize[2];k++) {
                if(grid[i][j][k] > 1){
                    if(grid[i][j][k] == target){
                        pcl::PointXYZRGB point = pcl::PointXYZRGB(255,255,255);
                        point.x = i;
                        point.y = j;
                        point.z = k;
                        componentCloud.push_back(point);

                    }else{
                        pcl::PointXYZRGB point = pcl::PointXYZRGB(rgbTable[grid[i][j][k]%20][0], rgbTable[grid[i][j][k]%20][1], rgbTable[grid[i][j][k]%20][2]);
                        point.x = i;
                        point.y = j;
                        point.z = k;
                        componentCloud.push_back(point);
                    }
                }
            }
        }
    }
    // Run marching cubes
    MarchingCubes::marchingCube(matrixSize,grid,tsdfGrid,target,ptrComponentCloud);
}

float synthesizeTSDF(float v1, float v2)
{
    if(v1 != 0 && v2 != 0){
        return std::max(v2, -v1);
    } else {
        if(v1 != 0){
            // This is to get rid of the scene misalignment
            if(v1 < 0.99){
                return -v1;
            } else {
                return 0;
            }
        }
        else{
            return 0;
        }
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
		std::cout << "The TSDF without the object should be named tsdf.bin and the file with object should be named tsdf2.bin."
                  << std::endl
                  << "Place these two files in this directory or provide its directory as a parameter when running the program."
                  << std::endl << std::endl;
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
                        if(synthesizeTSDF(tsdfval1, tsdfval2) < 0){
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

    // Calculate size of the matrix to perform operations on
    int startPoint[3] = {minX, minY, minZ};
    int matrixSize[3] = {maxX-minX+1, maxY-minY+1, maxZ-minZ+1};

    // Create the TSDF grid for marching cubes
    std::vector<std::vector<std::vector<float>>> tsdfGrid;
    tsdfGrid.resize((unsigned) matrixSize[0]);
    for (int i = 0; i < matrixSize[0]; i++) {
        tsdfGrid[i].resize((unsigned) matrixSize[1]);

        for (int j = 0; j < matrixSize[1]; j++) {
            tsdfGrid[i][j].resize((unsigned) matrixSize[2]);
        }
    }
    // Read the file again for corresponding float values
    FILE * fp3 = fopen(tsdfName.c_str(), "r");
    FILE * fp4 = fopen(tsdfName2.c_str(), "r");
    for (int i = 0; i < 512; i++) {
        for (int j = 0; j < 512; j++){
            for (int k = 0; k < 512; k++){
                if(fread((void*)(&tsdfval1), sizeof(tsdfval1), 1, fp3)) {
                    if (fread((void *) (&tsdfval2), sizeof(tsdfval2), 1, fp4)) {
                        if (i >= minX && i <= maxX && j >= minY && j <= maxY && k >= minZ && k <= maxZ) {
                            tsdfGrid[i - minX][j - minY][k - minZ] = synthesizeTSDF(tsdfval1, tsdfval2);
                        }
                    }
                }
            }
        }
    }
    std::cout << "TSDF point cloud generated. The cloud has " << count << " points." << std::endl;
    // Run connected component
    connectedComponents(ptrCloud, startPoint, matrixSize, tsdfGrid);
    return (0);
}
