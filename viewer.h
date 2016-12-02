#ifndef VIEWER_H
#define VIEWER_H

/*******Boost*****/
#include <boost/math/special_functions/round.hpp>
#include <boost/thread/thread.hpp>
/*******Eigen*****/
#include <eigen3/Eigen/Geometry>
/*******STD*******/
#include <iostream>
/*******Qt********/
#include <QWidget>
#include <QFileDialog>
#include <QIcon>
#include <QPixmap>
#include <QDesktopWidget>
/*Point Cloud Library*/
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkVersion.h>
#include <vtkInteractorStyleSwitch.h>
/**Clases propias*/
#include "interactor_style_actor.h"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
namespace Ui {
class Viewer;
}

class Viewer : public QWidget
{
    Q_OBJECT

public:
    explicit Viewer(QWidget *parent = 0);
    ~Viewer();
void saveScreenshot();
private:
    Ui::Viewer *ui;

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT::Ptr cloud_;
    PointT p1,p2;
    PointT m_registration_points[2];
    QPixmap originalPixmap;
    int text_id=0;
    int line_id=0;
    int marker_id=0;
    int mesh_id=0;
    int filtering_axis_;
    int color_mode_;
    int number_points;

    void  colorCloudDistances ();
    void fillCloud();

    void mouseEventOccurred (const pcl::visualization::MouseEvent& event, void* viewer_void);
    void keyBoardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    void pickEventLine (const pcl::visualization::PointPickingEvent &event, void* viewer_void);
    void area_picking_callback (const pcl::visualization::AreaPickingEvent &event, void*);

    void interactorInit();
public slots:
    void shootScreen();
    void saveFileButtonPressed ();
    void loadFileButtonPressed ();
    void addLine();
    void addPlane();
    void addSquare();
    void addSphere();
    void deleteDistances();
    void showFullScreen();
    void fVoxelGrid(int value);
    void loadInteractorCamera(bool checked);
    void loadInteractorActor(bool checked);



};

#endif // VIEWER_H
