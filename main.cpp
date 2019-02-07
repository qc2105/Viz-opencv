#include "utils.hpp"

#include <opencv2/opencv.hpp>

#include <opencv2/viz/types.hpp>
#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>

using namespace cv;
using namespace std;

#define CV_VERSION_NUMBER       \
    CVAUX_STR(CV_MAJOR_VERSION) \
    CVAUX_STR(CV_MINOR_VERSION) \
    CVAUX_STR(CV_SUBMINOR_VERSION)

void launching_Viz()
{
    // Create a window
    viz::Viz3d myWindow("Viz Demo");

    // Start event loop
    myWindow.spin();

    // Event loop is over when pressed q, Q, e, E
    cout << "First event loop is over" << endl;

    // Access window via its name
    viz::Viz3d sameWindow = viz::getWindowByName("Viz Demo");

    // Start event loop
    sameWindow.spin();

    // Event loop is over when pressed q, Q, e, E
    cout << "Second event loop is over" << endl;

    // Event loop is over when pressed q, Q, e, E
    // Start event loop once for 1 millisecond
    sameWindow.spinOnce(1, true);
    while (!sameWindow.wasStopped())
    {
        // Interact with window

        // Event loop for 1 millisecond
        sameWindow.spinOnce(1, true);
    }

    // Once more event loop is stopped
    cout << "Last event loop is over" << endl;
}

void Pose_of_a_widget()
{

    viz::Viz3d myWindow("Coordinate Frame");

    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
    axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("Line Widget", axis);

    viz::WCube cube_widget(Point3f(0.5, 0.5, 0.0), Point3f(0.0, 0.0, -0.5), true, viz::Color::blue());
    cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);

    myWindow.showWidget("Cube Widget", cube_widget);

    Mat rot_vec = Mat::zeros(1, 3, CV_32F);
    float translation_phase = 0.0, translation = 0.0;
    while (!myWindow.wasStopped())
    {
        /* Rotation using rodrigues */
        // Rotate around (1,1,1)
        rot_vec.at<float>(0, 0) += CV_PI * 0.01f;
        rot_vec.at<float>(0, 1) += CV_PI * 0.01f;
        rot_vec.at<float>(0, 2) += CV_PI * 0.01f;

        // Shift on (1,1,1)
        translation_phase += CV_PI * 0.01f;
        translation = sin(translation_phase);

        Mat rot_mat;
        Rodrigues(rot_vec, rot_mat);

        // Construct pose
        Affine3f pose(rot_mat, Vec3f(translation, translation, translation));

        myWindow.setWidgetPose("Cube Widget", pose);

        myWindow.spinOnce(1, true);
    }
}

void cvfaces_load(Mat &cloud, vector<viz::WLine> & lines, string &obj_file_path)
{
	ifstream ifs(obj_file_path, ifstream::in);

	string str;
	int n = 0;
	int point1_index;
	int point2_index;
	int point3_index;

	while (ifs.good())
	{
		getline(ifs, str);
		cout << str << endl;

		if (string::npos == str.find("f "))
			continue;
		vector<string> strs;
		split_str(str, strs, ' ');

		point1_index = stof(strs[1]);
		point2_index = stof(strs[2]);
		point3_index = stof(strs[3]);

		Point3f point1 = cloud.at<Vec3f>(0, point1_index - 1);
		Point3f point2 = cloud.at<Vec3f>(0, point2_index - 1);
		Point3f point3 = cloud.at<Vec3f>(0, point3_index - 1);

		cout << point1 << ": " << point2 << ": " << point3;

		viz::WLine line1 = viz::WLine(point1, point2, viz::Color::silver());
		viz::WLine line2 = viz::WLine(point2, point3, viz::Color::silver());
		viz::WLine line3 = viz::WLine(point3, point1, viz::Color::silver());

		lines.push_back(line1);
		lines.push_back(line2);
		lines.push_back(line3);
		
		cout << point1_index << ", " << point2_index << ", " << point3_index << endl;
		n++;
	}

	ifs.close();
}

void cvcloud_load(Mat &cloud, string &obj_file_path)
{
    ifstream ifs(obj_file_path, ifstream::in);

    Point3f *data = cloud.ptr<cv::Point3f>();
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;

    string str;
    int n = 0;
	float maxY = 0.f;
	float maxZ = 0.f;

    while (ifs.good())
    {
        getline(ifs, str);
        cout << str << endl;

        if (string::npos == str.find("v "))
            continue;
        vector<string> strs;
        split_str(str, strs, ' ');

        data->x = stof(strs[1]);
        data->y = stof(strs[2]);
        data->z = stof(strs[3]);
        x += data->x;
        y += data->y;
		if (data->y > maxY)
		{
			maxY = data->y;
		}
        z += data->z;
		if (data->z > maxZ)
		{
			maxZ = data->z;
		}
        cout << data->x << ", " << data->y << ", " << data->z << endl;
        data++;
        n++;
    }

    ifs.close();

	Point3f *ptr = cloud.ptr<cv::Point3f>();
	/*for (int i = 0; i < n; i++)
	{
		ptr->y = maxY - ptr->y;
		ptr->z = maxZ - ptr->z;
		ptr++;
	}*/

    ptr = cloud.ptr<cv::Point3f>();
    for (int i = 0; i < n; i++)
    {
        ptr->x -= x / n;
        ptr->y -= y / n;
        ptr->z -= z / n;
        ptr++;
    }
   
    cloud *= 5.0f;
}

int transformations(string &obj_file_path)
{

    bool camera_pov = true; //false;

    // Create a window
    viz::Viz3d myWindow("Coordinate Frame");

    // Add coordinate axes
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    // Let's assume camera has the following properties
    Vec3d cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);

    // We can get the pose of the cam using makeCameraPose
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    // We can get the transformation matrix from camera coordinate system to global using
    // - makeTransformToGlobal. We need the axes of the camera
    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);

    // Create a cloud widget.
    Mat bunny_cloud(1, nlines(obj_file_path, string("v ")), CV_32FC3);
	vector<viz::WLine> faces;
    cvcloud_load(bunny_cloud, obj_file_path);
	cvfaces_load(bunny_cloud, faces, obj_file_path);
	viz::WLine lineX = viz::WLine(Point3d(0, 0, 0), Point3d(800, 0, 0), viz::Color::red());
	viz::WLine lineY = viz::WLine(Point3d(0, 0, 0), Point3d(0, 800, 0), viz::Color::green());
	viz::WLine lineZ = viz::WLine(Point3d(0, 0, 0), Point3d(0, 0, 800), viz::Color::blue());

    
    viz::WCloud cloud_widget(bunny_cloud, viz::Color::green());

    Mat poly_cloud(1, nlines(obj_file_path, string("f ")), CV_32FC3);
    viz::Mesh mesh = viz::Mesh::load(obj_file_path, 2);
    viz::WMesh mesh_widget(mesh.cloud, mesh.polygons);
    
    // Pose of the widget in camera frame
    Affine3f cloud_pose = Affine3f().translate(Vec3f(0.0f, 0.0f, 3.0f));
    // Pose of the widget in global frame
    Affine3f cloud_pose_global = transform * cloud_pose;

    // Visualize camera frame
    if (!camera_pov)
    {
        viz::WCameraPosition cpw(0.5);                               // Coordinate axes
        viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
        myWindow.showWidget("CPW", cpw, cam_pose);
        myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
    }

    // Visualize widget
    myWindow.showWidget("test", cloud_widget, cloud_pose_global);
    //myWindow.showWidget("bunny", mesh_widget, cloud_pose_global);
	myWindow.showWidget("lineX", lineX, cloud_pose_global);
	myWindow.showWidget("lineY", lineY, cloud_pose_global);
	myWindow.showWidget("lineZ", lineZ, cloud_pose_global);
	for (int n = 0; n < faces.size(); n++)
	{
		myWindow.showWidget(std::to_string(n), faces.at(n), cloud_pose_global);
	}

    // Set the viewer pose to that of camera
    if (camera_pov)
        myWindow.setViewerPose(cam_pose);

    // Start event loop.
    myWindow.spin();

    return 0;
}

int main(int argc, char **argv)
{
    // std::cout << getBuildInformation();

    // launching_Viz();

    // Pose_of_a_widget();

    if (argc != 2)
    {
        cerr << "Usage: " << argv[0] << " <obj_file_path>" << endl;
        exit(-1);
    }

    string obj_file_path = argv[1];
    transformations(obj_file_path);

    return 0;
}
