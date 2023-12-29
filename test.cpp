#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <boost/regex.hpp>

using namespace std;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;


struct MyPoint {
	PCL_ADD_POINT4D;             // 添加PointXYZ的x、y、z字段
	float intensity;             // 强度字段
	int wall;                    // 在PCD文件中的位置字段
	int tfile;                   // PCD的唯一标识字段

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
		MyPoint(const pcl::PointXYZI& point) : x(point.x), y(point.y), z(point.z) {}
} EIGEN_ALIGN16;                // 使用16字节对齐

// 使用宏注册 MyPoint 结构体
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(int, wall, wall)
	(int, tfile, tfile)
)

MyPoint convertToPointT(const pcl::PointXYZI& pointT) {
	return MyPoint(pointT);
}

// 遍历文件夹并加载点云
void loadPointClouds(const std::string& rootFolder, std::vector<std::vector<PointT>>& allCloud) {
	namespace fs = boost::filesystem;

	// 使用正则表达式定义文件夹名称的匹配模式
	int wall_id = 0;
	boost::regex folderPattern("room.*");
	boost::regex folderPattern1("yangchang.*");
	// 遍历根文件夹下的所有文件夹
	for (fs::directory_iterator roomIt(rootFolder); roomIt != fs::directory_iterator(); ++roomIt) {
		// 判断文件夹名称是否匹配正则表达式

		if (fs::is_directory(roomIt->status()) && boost::regex_match(roomIt->path().filename().string(), folderPattern)) {
			// 如果是文件夹，表示一个房间

			// 遍历房间文件夹下的所有点云文件

			for (fs::directory_iterator cloudIt(roomIt->path()); cloudIt != fs::directory_iterator(); ++cloudIt) {

				if (fs::is_regular_file(cloudIt->status()) && boost::regex_match(cloudIt->path().filename().string(), folderPattern1)) {
					// 如果是点云文件，加载点云
					// 如果是点云文件，加载点云
					try {
						pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>);
						pcl::io::loadPCDFile(cloudIt->path().string(), *tempCloud);
						std::vector<PointT> currentPoints;

						// 提取首尾点并添加到当前点云的 vector 中
						currentPoints.push_back(tempCloud->front());
						currentPoints.push_back(tempCloud->back());

						allCloud.push_back(currentPoints);
					}
					catch (const pcl::PCLException& e) {
						std::cerr << "Error loading PCD file: " << e.what() << std::endl;
					}
				}
			}


		}
	}
}
vector<pcl::PointXYZI> putPoiToVec(string inpath) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI>(inpath, *all_cloud);
	vector<pcl::PointXYZI> all_cloud2;
	for (auto p = all_cloud->begin(); p != all_cloud->end(); p++) {
		all_cloud2.push_back(*p);
	}
	return all_cloud2;
}
double twopointdistant(const PointT& p1, const PointT& p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

//得到单个点网格内点的个数
int getifintopo(const pcl::PointXYZI& p, const  vector<pcl::PointXYZI>& cloud_i) {

	//pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	int count_pose = 0;
	for (int l = 0; l < cloud_i.size(); l++) {
		if (twopointdistant(cloud_i[l], p) < 0.002) {
			count_pose++;
		}
	}
	return count_pose;


}
//当线段的两点之间都有点，需要延长找端点
bool yanchangcount2(vector<pcl::PointXYZI>& loorcloud, vector<pcl::PointXYZI>& cloud) {


	vector<pcl::PointXYZI> line_cloud;
	if (loorcloud.size() != 2) {
		cout << "表示线的点不是两个，错误！  " << endl;
		return false;
	}
	//pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointXYZI a = loorcloud[0];
	pcl::PointXYZI b = loorcloud[1];
	//检查x的顺序,保证a值一定在右边
	if (loorcloud[0].x < loorcloud[1].x) {
		pcl::PointXYZI a = loorcloud[0];
		pcl::PointXYZI b = loorcloud[1];
	}
	else {
		pcl::PointXYZI a = loorcloud[1];
		pcl::PointXYZI b = loorcloud[0];
	}
	double distance = sqrt(pow(a.x + b.x, 2) + pow(a.y + b.y, 2) + pow(a.z + b.z, 2));//计算两点间距
	int line_size = distance / 0.025 + 1;//计算间距点数
	double increment_x = abs((b.x - a.x) / line_size);
	double increment_y = abs((b.y - a.y) / line_size);
	double increment_z = abs((b.z - a.z) / line_size);
	//循环插入点

	//pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
	int n;
	int i = 0;
	int count = 0;
	if (a.y < b.y) {
		n = 2;
	}
	else {
		n = 1;
	}
	int k = pow(-1, n);
	do {
		pcl::PointXYZI temp_point1;
		i--;
		temp_point1.x = a.x + increment_x * (i);
		temp_point1.y = a.y + increment_y * (i)*k;
		temp_point1.z = a.z + increment_z * (i);
		count = getifintopo(temp_point1, cloud);//判断延长一定步长后的点网格内是否有点

		if (count > 0) {

			line_cloud.push_back(temp_point1);
			//line_cloud2->push_back(temp_point1);
		}
	} while (count == 0);

	int j = 0;
	count = 0;
	do {
		pcl::PointXYZI temp_point2;
		j++;
		temp_point2.x = b.x + increment_x * (j);
		temp_point2.y = b.y + increment_y * (j)*k;
		temp_point2.z = b.z + increment_z * (j);
		count = getifintopo(temp_point2, cloud);

		if (count > 0) {

			line_cloud.push_back(temp_point2);
			//line_cloud2->push_back(temp_point2);
		}
	} while (count == 0);
	/*cout << "请输入间断点的文件名" << endl;
	string path; cin >> path;
	pcl::io::savePCDFileASCII(path.c_str(), *line_cloud2);*/

	loorcloud = line_cloud;

	return true;
}
// 合并所有点集到一个大的点云
pcl::PointCloud<PointT>::Ptr mergePointClouds(const std::vector<std::vector<PointT>>& allPoints) {
	pcl::PointCloud<PointT>::Ptr mergedCloud(new pcl::PointCloud<PointT>);

	for (const auto& points : allPoints) {
		for (auto p : points) {
			mergedCloud->push_back(p);
		}
		//mergedCloud->insert(mergedCloud->end(), points.begin(), points.end());


	}

	return mergedCloud;
}

// 将所有的点写入一个 PCD 文件
void writeMergedPointCloud(const std::vector<std::vector<PointT>>& allPoints, const std::string& outputPath) {
	pcl::PointCloud<PointT>::Ptr mergedCloud = mergePointClouds(allPoints);

	// 保存为 PCD 文件
	pcl::io::savePCDFileASCII(outputPath, *mergedCloud);
}
//得到两点之间的点
void gepoint(PointT& a, PointT& b, const pcl::PointCloud<PointT>::Ptr& f) {


	PointT p, q;//起点和终点

	p.x = a.x; p.y = a.y; p.z = a.z;
	q.x = b.x; q.y = b.y; q.z = b.z;
	double distance = sqrt(pow(p.x + q.x, 2) + pow(p.y + q.y, 2) + pow(p.z + q.z, 2));//计算两点间距
	int line_size = distance / 0.05 + 1;//计算间距点数
	double increment_x = (b.x - a.x) / line_size;
	double increment_y = (b.y - a.y) / line_size;
	double increment_z = (b.z - a.z) / line_size;
	for (int i = 0; i <= line_size; ++i) {//循环插入点
		PointT temp_point;
		temp_point.x = a.x + increment_x * i;
		temp_point.y = a.y + increment_y * i;
		temp_point.z = a.z + increment_z * i;
		f->push_back(temp_point);

	}
	// 将PointCloud对象保存到文件中




}
int main() {
	std::vector<std::vector<PointT>> allPoints;

	// 指定根文件夹路径
	std::string rootFolder = "./room";

	// 加载点云
	loadPointClouds(rootFolder, allPoints);//只是每个pcd的首尾两点


	vector<pcl::PointXYZI> ori_cloud = putPoiToVec("10Test.pcd");
	int wall_id = 0;
	//延长

	pcl::PointCloud<MyPoint>::Ptr a(new pcl::PointCloud<MyPoint>);
	for (auto l : allPoints) {
		yanchangcount2(l, ori_cloud);
		MyPoint f = convertToPointT(l[0]);
		MyPoint b = convertToPointT(l[1]);
		f.wall = wall_id;
		b.wall = wall_id;
		wall_id++;
		a->push_back(f);
		a->push_back(b);
		//gepoint(l[0], l[1], a);
	}



	// 保存为 PCD 文件
	pcl::io::savePCDFileASCII("allhourse_wallid.pcd", *a);
	return 0;
}