#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <iostream>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>

using namespace std;
using namespace pcl;

// Cloud file.
string filename;

// Parameter vriables.
float plan_pose_x,plan_pose_y,plan_pose_z,
imageSizeX,imageSizeY;

// Sensor pose.
Eigen::Affine3f sensorPose;

// Both range images.
boost::shared_ptr<pcl::RangeImage> sphe_range_image_ptr(new pcl::RangeImage);
pcl::RangeImage& sph_range_image = *sphe_range_image_ptr;

boost::shared_ptr<pcl::RangeImagePlanar> plan_range_image_ptr(new pcl::RangeImagePlanar);
// Heap range image.
//pcl::RangeImagePlanar& plan_range_image = *plan_range_image_ptr;

// Stack range image
pcl::RangeImagePlanar plan_rangeImage;

// Both visualizers.
pcl::visualization::RangeImageVisualizer*sphe_viewer_ptr = new pcl::visualization::RangeImageVisualizer;
pcl::visualization::RangeImageVisualizer*plan_viewer_ptr  = new pcl::visualization::RangeImageVisualizer;

// Original cloud.
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Original cloud viewer.
boost::shared_ptr<pcl::visualization::PCLVisualizer> cloudviewer
(new pcl::visualization::PCLVisualizer(""));

// Text data display
boost::shared_ptr<pcl::visualization::PCLVisualizer> textdata
(new pcl::visualization::PCLVisualizer(""));

// Borders recognition activation flag.
bool borders = false;

// Pointing arrows activation flag.
bool points = false;

// For spherical range image.
// Angular resolution is the angular distance between pixels.
// Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
// Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
// Center of projection. here, we choose the middle of the image.
float centerX;
float centerY;
// Focal length. The value seen here has been taken from the original depth images.
// It is safe to use the same value vertically and horizontally.
float focalLengthX = 525.000f, focalLengthY = focalLengthX;

// Noise level. If greater than 0, values of neighboring points will be averaged.
// This would set the search radius (i.e., 0.03 == 3cm).
float noiseLevel = 0.1f;
// Minimum range. If set, any point closer to the sensor than this will be ignored.
float minimumRange = 0.0f;


//#define ACTIVE
#ifdef ACTIVE
void create_range_images(float focalLengthY, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Affine3f& sensorPose,
                         float centerX, float minimumRange, float imageSizeX, float centerY, float noiseLevel, float focalLengthX, float imageSizeY)
{
    angularResolutionX = angularResolutionX / 2.0f;
    angularResolutionY = angularResolutionY / 2.0f;

    //    sphe_viewer_ptr->~RangeImageVisualizer();
    //    plan_viewer_ptr->~RangeImageVisualizer();

    //    sphe_viewer_ptr->close();
    //    plan_viewer_ptr->close();
    //    if (sphe_viewer_ptr != NULL) delete sphe_viewer_ptr;
    //    if (plan_viewer_ptr != NULL) delete plan_viewer_ptr;

    //    sphe_viewer_ptr = new pcl::visualization::RangeImageVisualizer;
    //    plan_viewer_ptr = new pcl::visualization::RangeImageVisualizer;

    //    sphe_range_image_ptr->~RangeImage();
    //    plan_range_image_ptr->~RangeImagePlanar();


    //    // Recreate objects for both range images.
    //    sphe_range_image_ptr.reset();
    //    sphe_range_image_ptr.reset( new pcl::RangeImage);
    //    // sph_range_image = *sphe_range_image_ptr;

    //    plan_range_image_ptr.reset();
    //    plan_range_image_ptr.reset(new pcl::RangeImagePlanar);
    //    // plan_range_image = *plan_range_image_ptr;

    //    pcl::RangeImage sph_range_image;
    //    pcl::RangeImagePlanar plan_range_image;

    sphe_range_image_ptr->clear();
    sphe_range_image_ptr->resize(0);
    sphe_range_image_ptr->reset();
    sphe_range_image_ptr->createFromPointCloud (*cloud, pcl::deg2rad (0.5f),pcl::deg2rad (0.5f),
                                                pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                                sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minimumRange, 1);

    plan_range_image_ptr->clear();
    plan_range_image_ptr->resize(0);
    plan_range_image_ptr->reset();
    plan_range_image_ptr->createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
                                                            centerX, centerY, focalLengthX, focalLengthY,
                                                            sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                            noiseLevel, minimumRange);

    //DEACT: visualizeBorders takes this over.
    // sphe_viewer_ptr->showRangeImage(*sphe_range_image_ptr);

    plan_viewer_ptr->showRangeImage(*plan_range_image_ptr);

    if (borders) {
    // -------------------------
    // -----Extract borders-----
    // -------------------------
    pcl::RangeImageBorderExtractor border_extractor (&sph_range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute (border_descriptions);

    sphe_viewer_ptr->visualizeBorders(sph_range_image,
                                                                              -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false, border_descriptions);
    }
}

//void show_range_images()
//{
//    sphe_viewer_ptr->showRangeImage(sph_range_image);
//    plan_viewer_ptr->showRangeImage(plan_rangeImage);
//}

static void pointPickingEventHandler(const pcl::visualization::PointPickingEvent e){
    float x,y,z;

    if (points) {
    e.getPoint(x,y,z);
    PointXYZ p2 =  PointXYZ(x,y,z);
    PointXYZ p1 = PointXYZ(x+100,y+100,z+100);

    // Get random number as string to number arrows.
    // Arrows must have different names.
    int random_number_arrow = rand() % 1000;
    std::ostringstream oss_arrow;
    oss_arrow << random_number_arrow;

    // Get random number as string to number arrows.
    // Arrows must have different names.
    int random_number_text = rand() % 1000;
    std::ostringstream oss_text;
    oss_text << random_number_text;

    // Add text nearby marked point.

    std::ostringstream oss2;
    oss2 << "X: " << x << "Y: " <<  y << "Z: " << z ;

    // Adds an arrow between points p1 and p2 and distance between them.
    cloudviewer->addArrow(p1,p2,0.5, 0.5, 0.0,oss_arrow.str());
    cloudviewer->addText3D(oss2.str(),p2,1.0,1.0,1.0,1.0, oss_text.str());
    }
}


void add_camera_cone(float posez, float posex, float posey, float roll, float pitch, float yaw)
{
    pcl::ModelCoefficients coeff;
    coeff.values.resize(7);
    Eigen::Vector3f cone_apex(posex,posey,posez);
    Eigen::Vector3f axis_direction(roll,pitch,yaw);
    coeff.values[0] = cone_apex.x();    coeff.values[1] = cone_apex.y();    coeff.values[2] = cone_apex.z();
    coeff.values[3] = axis_direction.x();coeff.values[4] = axis_direction.y();coeff.values[5] = axis_direction.z();
    coeff.values[6] = 0.25;
    cloudviewer->removeShape("PlanarRangeImageCone");
    cloudviewer->addCone(coeff,"PlanarRangeImageCone");
}

void get_viewer_camera( float posez,  float pitch,  float yaw,  float posex,  float posey,  float roll)
{
    std::vector<pcl::visualization::Camera> cameras;
    cloudviewer->getCameras(cameras);
    posex = cameras[0].pos[0];
    posey = cameras[0].pos[1];
    posez = cameras[0].pos[2];

    roll     = cameras[0].view[0];
    pitch  = cameras[0].view[1];
    yaw   = cameras[0].view[2];
}

static void set_text_data(float posex,float posey, float posez,
                          float roll, float pitch, float yaw) {
    std::ostringstream oss2;
    oss2 << "POSE X: " << posex << " POSE Y: " <<  posey << " POSE Z: " << posez  << " ROLL: "
         << roll << " PITCH: " << pitch << " YAW: " << yaw;
    textdata->removeText3D("Pose data");
    textdata->addText3D(oss2.str(),PointXYZ(0,0,0),0.1,1.0,1.0,1.0,"Pose data");
}

static void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
    static float posex = 0;
    static float posey = 0;
    static float posez = 0;

    static float roll      = pcl::deg2rad((float)180);
    static float pitch   = pcl::deg2rad((float)0);
    static float yaw    = pcl::deg2rad((float)180);
    const float angle_step = pcl::deg2rad((float)5);
    if (event.keyDown()){
        if (event.getKeySym() == "r") cloudviewer->removeAllShapes();
        if (event.getKeySym() == "x") posex += 0.1;
        if (event.getKeySym() == "y") posey += 0.1;
        if (event.getKeySym() == "z") posez += 0.1;

        if (event.getKeySym() == "X") posex -= 0.1;
        if (event.getKeySym() == "Y") posey -= 0.1;
        if (event.getKeySym() == "Z") posez -= 0.1;

        if (event.getKeySym() == "1") roll     += angle_step;
        if (event.getKeySym() == "2") pitch  += angle_step;
        if (event.getKeySym() == "3") yaw   += angle_step;

        if (event.getKeySym() == "4") roll     -= angle_step;
        if (event.getKeySym() == "5") pitch  -= angle_step;
        if (event.getKeySym() == "6") yaw   -= angle_step;

        if (event.getKeySym() == "b") borders = !borders;
        if (event.getKeySym() == "p") points   = !points;

        // get_viewer_camera(posez, pitch, yaw, posex, posey, roll);

        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
        Eigen::Quaternionf orientation = yawAngle * pitchAngle * rollAngle;

        sensorPose = Eigen::Affine3f(Eigen::Translation3f(posex,posey,posez)*Eigen::Affine3f(orientation));
        create_range_images(focalLengthY, cloud, sensorPose, centerX, minimumRange, imageSizeX, centerY, noiseLevel, focalLengthX, imageSizeY);

        add_camera_cone(posez, posex, posey, roll, pitch, yaw);

        set_text_data(posex,posey,posez,roll,pitch,yaw);

        std::ostringstream oss2;
        oss2 << "X: " << posex << "Y: " <<  posey << "Z: " << posez ;
        cloudviewer->removeText3D(" ");
        cloudviewer->addText3D(oss2.str(),PointXYZ(0,0,0),1.0,1.0,1.0,1.0," ");

    }
}

int
main(int argc, char** argv)
{
    plan_pose_x = atof(((string)argv[10]).c_str());
    plan_pose_y = atof(((string)argv[11]).c_str());
    plan_pose_z = atof(((string)argv[12]).c_str());
    imageSizeX = atof(((string)argv[13]).c_str());
    imageSizeY = atof(((string)argv[14]).c_str());

    if (console::find_argument(argc, argv, "-v") >= 0)
    {
        filename = argv[2];
        io::loadPCDFile<pcl::PointXYZ>(filename.c_str(), *cloud);
    }

    sensorPose = (Eigen::Affine3f)Eigen::Translation3f(plan_pose_x,plan_pose_y, plan_pose_z);

    // Planar range image object.
    //	pcl::RangeImagePlanar rangeImagePlanar;
    //	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
    //			centerX, centerY, focalLengthX, focalLengthX,
    //			sensorPose, pcl::RangeImage::CAMERA_FRAME,
    //            noiseLevel, minimumRange);

    // Maximum horizontal and vertical angles. For example, for a full panoramic scan,
    // the first would be 360º. Choosing values that adjust to the real sensor will
    // decrease the time it takes, but don't worry. If the values are bigger than
    // the real ones, the image will be automatically cropped to discard empty zones.
    float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
    // Sensor pose. Thankfully, the cloud includes the data.

    centerX = imageSizeX / 2.0f;
    centerY = imageSizeY / 2.0f;

    cloud->width = (uint32_t) cloud->points.size();
    cloud->height = 1;

    create_range_images(focalLengthY, cloud, sensorPose, centerX, minimumRange, imageSizeX, centerY, noiseLevel, focalLengthX, imageSizeY);

    //*************************************************************************************************************************
    //------------------------
    //----Show cloud----
    //------------------------
    std::vector< pcl::visualization::Camera > cameras;
    cloudviewer->getCameras(cameras);
    cloudviewer->addCoordinateSystem(1,"CoordinateSystem");
    cloudviewer->addPointCloud<pcl::PointXYZ>(cloud, "Generated cloud");

    //----------------------------------------------
    //----Show spherical range image----
    //----------------------------------------------
    plan_viewer_ptr->registerKeyboardCallback(keyboardEventOccurred);
    cloudviewer->registerPointPickingCallback(pointPickingEventHandler);

    while (!plan_viewer_ptr->wasStopped())
    {
        sphe_viewer_ptr->spinOnce();
        plan_viewer_ptr->spinOnce();
        cloudviewer->spinOnce();
        textdata->spinOnce();
        // Sleep 100ms to go easy on the CPU.
        pcl_sleep(0.001);
    }
}
#endif
