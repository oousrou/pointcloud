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
	PCL_ADD_POINT4D;             // ���PointXYZ��x��y��z�ֶ�
	float intensity;             // ǿ���ֶ�
	int wall;                    // ��PCD�ļ��е�λ���ֶ�
	int tfile;                   // PCD��Ψһ��ʶ�ֶ�

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ȷ���ڴ����
		MyPoint(const pcl::PointXYZI& point) : x(point.x), y(point.y), z(point.z) {}
} EIGEN_ALIGN16;                // ʹ��16�ֽڶ���

// ʹ�ú�ע�� MyPoint �ṹ��
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

// �����ļ��в����ص���
void loadPointClouds(const std::string& rootFolder, std::vector<std::vector<PointT>>& allCloud) {
	namespace fs = boost::filesystem;

	// ʹ��������ʽ�����ļ������Ƶ�ƥ��ģʽ
	int wall_id = 0;
	boost::regex folderPattern("room.*");
	boost::regex folderPattern1("yangchang.*");
	// �������ļ����µ������ļ���
	for (fs::directory_iterator roomIt(rootFolder); roomIt != fs::directory_iterator(); ++roomIt) {
		// �ж��ļ��������Ƿ�ƥ��������ʽ

		if (fs::is_directory(roomIt->status()) && boost::regex_match(roomIt->path().filename().string(), folderPattern)) {
			// ������ļ��У���ʾһ������

			// ���������ļ����µ����е����ļ�

			for (fs::directory_iterator cloudIt(roomIt->path()); cloudIt != fs::directory_iterator(); ++cloudIt) {

				if (fs::is_regular_file(cloudIt->status()) && boost::regex_match(cloudIt->path().filename().string(), folderPattern1)) {
					// ����ǵ����ļ������ص���
					// ����ǵ����ļ������ص���
					try {
						pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>);
						pcl::io::loadPCDFile(cloudIt->path().string(), *tempCloud);
						std::vector<PointT> currentPoints;

						// ��ȡ��β�㲢��ӵ���ǰ���Ƶ� vector ��
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

//�õ������������ڵ�ĸ���
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
//���߶ε�����֮�䶼�е㣬��Ҫ�ӳ��Ҷ˵�
bool yanchangcount2(vector<pcl::PointXYZI>& loorcloud, vector<pcl::PointXYZI>& cloud) {


	vector<pcl::PointXYZI> line_cloud;
	if (loorcloud.size() != 2) {
		cout << "��ʾ�ߵĵ㲻������������  " << endl;
		return false;
	}
	//pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointXYZI a = loorcloud[0];
	pcl::PointXYZI b = loorcloud[1];
	//���x��˳��,��֤aֵһ�����ұ�
	if (loorcloud[0].x < loorcloud[1].x) {
		pcl::PointXYZI a = loorcloud[0];
		pcl::PointXYZI b = loorcloud[1];
	}
	else {
		pcl::PointXYZI a = loorcloud[1];
		pcl::PointXYZI b = loorcloud[0];
	}
	double distance = sqrt(pow(a.x + b.x, 2) + pow(a.y + b.y, 2) + pow(a.z + b.z, 2));//����������
	int line_size = distance / 0.025 + 1;//���������
	double increment_x = abs((b.x - a.x) / line_size);
	double increment_y = abs((b.y - a.y) / line_size);
	double increment_z = abs((b.z - a.z) / line_size);
	//ѭ�������

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
		count = getifintopo(temp_point1, cloud);//�ж��ӳ�һ��������ĵ��������Ƿ��е�

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
	/*cout << "�������ϵ���ļ���" << endl;
	string path; cin >> path;
	pcl::io::savePCDFileASCII(path.c_str(), *line_cloud2);*/

	loorcloud = line_cloud;

	return true;
}
// �ϲ����е㼯��һ����ĵ���
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

// �����еĵ�д��һ�� PCD �ļ�
void writeMergedPointCloud(const std::vector<std::vector<PointT>>& allPoints, const std::string& outputPath) {
	pcl::PointCloud<PointT>::Ptr mergedCloud = mergePointClouds(allPoints);

	// ����Ϊ PCD �ļ�
	pcl::io::savePCDFileASCII(outputPath, *mergedCloud);
}
//�õ�����֮��ĵ�
void gepoint(PointT& a, PointT& b, const pcl::PointCloud<PointT>::Ptr& f) {


	PointT p, q;//�����յ�

	p.x = a.x; p.y = a.y; p.z = a.z;
	q.x = b.x; q.y = b.y; q.z = b.z;
	double distance = sqrt(pow(p.x + q.x, 2) + pow(p.y + q.y, 2) + pow(p.z + q.z, 2));//����������
	int line_size = distance / 0.05 + 1;//���������
	double increment_x = (b.x - a.x) / line_size;
	double increment_y = (b.y - a.y) / line_size;
	double increment_z = (b.z - a.z) / line_size;
	for (int i = 0; i <= line_size; ++i) {//ѭ�������
		PointT temp_point;
		temp_point.x = a.x + increment_x * i;
		temp_point.y = a.y + increment_y * i;
		temp_point.z = a.z + increment_z * i;
		f->push_back(temp_point);

	}
	// ��PointCloud���󱣴浽�ļ���




}
int main() {
	std::vector<std::vector<PointT>> allPoints;

	// ָ�����ļ���·��
	std::string rootFolder = "./room";

	// ���ص���
	loadPointClouds(rootFolder, allPoints);//ֻ��ÿ��pcd����β����


	vector<pcl::PointXYZI> ori_cloud = putPoiToVec("10Test.pcd");
	int wall_id = 0;
	//�ӳ�

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



	// ����Ϊ PCD �ļ�
	pcl::io::savePCDFileASCII("allhourse_wallid.pcd", *a);
	return 0;
}