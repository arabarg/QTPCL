#include "viewer.h"
#include "../build/ui_viewer.h"
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>

Viewer::Viewer(QWidget *parent) :  QWidget(parent),ui(new Ui::Viewer),
  filtering_axis_ (1), //y
  color_mode_ (4), //rainbow
  text_id (1),
  line_id (1)
{


    ui->setupUi(this);

    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());

    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer_->registerMouseCallback (&Viewer::mouseEventOccurred, *this, (void*)&viewer_);

    //ui->comboBox(ui->qvtkWidget);

    ui->groupBox->setParent(ui->qvtkWidget);

    ui->listWidget->setParent(ui->qvtkWidget);
    ui->verticalSlider->setParent(ui->qvtkWidget);
    viewer_->registerPointPickingCallback(&Viewer::pickEventLine, *this, (void*)&viewer_);
    viewer_->registerAreaPickingCallback(&Viewer::area_picking_callback, *this, (void*)&viewer_);

    viewer_->registerKeyboardCallback (&Viewer::keyBoardEventOccurred,*this, (void*)&viewer_);

    cloud_.reset(new PointCloudT);
    fillCloud();
    colorCloudDistances ();
    viewer_->addCoordinateSystem (1.0);
    viewer_->addPointCloud (cloud_, "cloud");
   addPlane(); //NO SE PORQUE NO PINTA EL PLANO


    //**SIGNALS Y SLOTS DEL FORMULARIO*/
    connect (ui->free, SIGNAL(clicked ()), this, SLOT(addSquare()));
    connect (ui->line, SIGNAL(clicked ()), this, SLOT(addLine()));
    connect (ui->circle, SIGNAL(clicked ()), this, SLOT(addSphere()));
    connect (ui->load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
    connect (ui->save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));
    connect (ui->back, SIGNAL(clicked ()), this, SLOT(shootScreen()));
    connect (ui->rubber, SIGNAL(clicked()), this, SLOT(deleteDistances()));
connect(ui->fullScreen, SIGNAL(clicked()), this, SLOT(showFullScreen()));
connect(ui->verticalSlider, SIGNAL(valueChanged(int)), this, SLOT(fVoxelGrid(int)));



}

void Viewer::saveScreenshot(){
    //Viewer* v;
    QString format = "png";

       QString initialPath = QDir::currentPath() + tr("/untitled.") + format;

//QString fileName = QFileDialog::getSaveFileName(this, tr ("Save As"),
             //                                   "/home/",
               //                                 tr ("(*.png)"));

QString fileName = QFileDialog::getSaveFileName(this, tr("Save As"),
                           initialPath,
                           tr("%1 Files (*.%2);;All Files (*)")
                           .arg(QString(format.toUpper()))
                           .arg(QString(format)));

       if (!fileName.isEmpty())
           originalPixmap.save(fileName, format.toAscii());


}
void Viewer::keyBoardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){

  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer_->removeShape (str);
    }
    text_id = 0;
  }
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "s was pressed => removing all lines" << std::endl;

    char line[512];
    for (unsigned int i = 0; i < line_id; ++i)
    {
      sprintf (line, "line#%03d", i);
      viewer_->removeShape (line);
    }
    line_id = 0;
  }
}
void Viewer::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void){
     if(number_points!=0){
       if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
           event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
       {
         char str[512];
         char buffer[512];
         sprintf (str, "text#%03d", text_id++);
         sprintf (buffer, "(%d,%d)", event.getX(), event.getY());
         viewer_->addText (buffer, event.getX(), event.getY(),str);
       }
        }
 }
void Viewer::pickEventLine (const pcl::visualization::PointPickingEvent& event, void* viewer_void){

 int idx = event.getPointIndex ();
 if (idx == -1)return;

 number_points++;

 if(number_points<=2){
    char str[512];
    char buffer[512];
    char line[512];
    char marker[512];
    char mesh[512];
    int i=1;
    float distance;
    PointT picked_point;
    PointT p1, p2;

    /*Detect point and save*/
    event.getPoint(picked_point.x,picked_point.y,picked_point.z);
    m_registration_points[number_points-1]=picked_point;

        /*Point Marker*/
        sprintf(marker, "sphere#%03d", marker_id++);
        viewer_->addSphere (picked_point, 0.01, 1.0, 0.0, 0.0, marker);
//      sprintf (str, "text#%03d", text_id++);
//      sprintf (buffer, "(%lf,%lf,%lf)", picked_point.x , picked_point.y, picked_point.z);
//      viewer_->addText3D (buffer, picked_point, 0.1, 1.0, 0.0, 0.0,str);
        if(number_points==2){
            /*Line*/
            sprintf(line,"line#%03d",line_id++ );
            viewer_->addLine(m_registration_points[0],m_registration_points[1],line);
            /*Distance*/
            distance=euclideanDistance(m_registration_points[0],m_registration_points[1]);
            sprintf (mesh, "mesh#%03d", mesh_id++);
            sprintf (buffer, "(d%i= %f m.)",mesh_id, distance);
            /*Combo*/

            ui->listWidget->addItem(buffer);
            ui->qvtkWidget->update ();
           }
      }

 }


/******************NO NOS VALE PARA ARCHIVOS GRANDES COMO LOS NUESTROS_**********************/
struct callback_args{
   // structure used to pass arguments to the callback function
   PointCloudT::Ptr clicked_points_3d;

 };
void Viewer::area_picking_callback (const pcl::visualization::AreaPickingEvent& event, void* viewer_void){
      std::vector<int> indices;
      struct callback_args* data = (struct callback_args *)viewer_void;
       PointT current_point;
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
      if (event.getPointsIndices(indices))
      {

          for(int i=0; i<indices.size();i++){
               current_point=cloud_->points[indices[i]];


              std::cout<<"indices"<<endl;
              data->clicked_points_3d->points.push_back(current_point);
              std::cout<<"data"<<endl;

          }
          viewer_->removePointCloud("clicked_points");
          std::cout<<"remove"<<endl;
          viewer_->addPointCloud(data->clicked_points_3d, red, "clicked_points");
          viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
        std::cout << "picked " << indices.size () <<std::endl;
         ui->qvtkWidget->update ();
      }
      else
        std::cout << "No valid points selected!" << std::endl;
 }


/*Rellena una nube de puntos y la muestra mientras no se sube ningun archivo*/
void Viewer::fillCloud(){

    cloud_->resize (500);

      // Fill the cloud with random points
      for (size_t i = 0; i < cloud_->points.size (); ++i)
      {
        cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }

}
void Viewer::colorCloudDistances ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  // Set an initial value
  switch (filtering_axis_)
  {
    case 0:  // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1:  // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default:  // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }

  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    switch (filtering_axis_)
    {
      case 0:  // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1:  // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default:  // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
}
void Viewer::addPlane(){
     std::cout << "  a" << std::endl;
    pcl::ModelCoefficients plane_coeff;
    Eigen::Vector4f plane_parameters;
    plane_coeff.values.resize (4);    // We need 4 values
    plane_coeff.values[0] = plane_parameters.x ();
    plane_coeff.values[1] = plane_parameters.y ();
    plane_coeff.values[2] = plane_parameters.z ();
    plane_coeff.values[3] = plane_parameters.w ();

    viewer_->addPlane(plane_coeff);
     ui->qvtkWidget->update ();
}


/*************************SLOTS***********************/
void Viewer::shootScreen(){
      originalPixmap = QPixmap::grabWindow(QApplication::desktop()->winId());
      saveScreenshot();
}
void Viewer::showFullScreen(){
    if(this->isFullScreen())this->setWindowState(Qt::WindowMaximized);
    else
this->setWindowState(Qt::WindowFullScreen);
}
void Viewer::deleteDistances(){
    /*delete lines*/
    char line[512];
    for (unsigned int i = 0; i < line_id; ++i)
    {
      sprintf (line, "line#%03d", i);
      viewer_->removeShape (line);
    }
    line_id = 0;
    /*delete markers*/
    char marker[512];
    for (unsigned int i = 0; i < marker_id; ++i)
    {
      sprintf (marker, "sphere#%03d", i);
      viewer_->removeShape (marker);
    }
    marker_id = 0;
    /*delete distances*/
   ui->listWidget->clear();
    marker_id = 0;
    /*delete free forms*/

    /*delete areas*/

    /*delete angles*/


    /*clean*/
 ui->qvtkWidget->update ();



}
void Viewer::addSquare(){

    PointT p1, p2, p3, p4;
  p1.x=0.0; p1.y=0.0; p1.z=0.0;
          p2.x=0.0; p2.y=4.0; p2.z=0.0;
            p3.x=4.0; p3.y=4.0; p3.z=0.0;
              p4.x=4.0; p4.y=0.0; p4.z=0.0;
     viewer_->addLine(p1,p2,"lin2");
     viewer_->addLine(p2,p3,"lin3");
viewer_->addLine(p3,p4,"lin4");
viewer_->addLine(p4,p1,"lin5");
 ui->qvtkWidget->update ();
}
void Viewer::addSphere(){
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize (3);    // We need 3 values
    circle_coeff.values[0] = 1.0;
    circle_coeff.values[1] = 1.0;
    circle_coeff.values[2] = 10;

    viewer_->addCircle (circle_coeff,  "sphere");
     ui->qvtkWidget->update ();
}
void Viewer::addLine(){
 std::cout<<"addLine"<<std::endl;
number_points=0;
 }
void Viewer::loadFileButtonPressed (){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));
          PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  PointCloudT::Ptr cloud_aux (new PointCloudT);
  PointCloudI::Ptr cloud_bi (new PointCloudI);
 PointCloudI::Ptr cloud_bi_filt (new PointCloudI);
  if (filename.isEmpty ()) return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

/*
 *FILTER: REMOVENANFROMPOINTCLOUD
*/
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  /*
   * FILTER VOXELGRID
   * */
  ui->verticalSlider->setValue(50);
 pcl::copyPointCloud (*cloud_,*cloud_aux);
  pcl::VoxelGrid<PointT> sor;
   sor.setInputCloud (cloud_);
   sor.setLeafSize (0.5f, 0.5f, 0.5f);
   sor.filter (*cloud_filtered);
   pcl::copyPointCloud (*cloud_filtered, *cloud_);

   /*
    * FILTER BILATERAL:: NO FUNCIONA NO SE SI ES POR EL TAMAÑO DE
    * LA IMAGEN
    * */
// [...]
//copyPointCloud(*cloud_,*cloud_bi);

//   pcl::BilateralFilter<PointI> BF;
//  pcl::search::KdTree<PointI>::Ptr tree (new     pcl::search::KdTree<PointI>);

//  BF.setSearchMethod(tree);
//  BF.setHalfSize(0.5);//el tamaño medio de la ventana de filtro bilateral de Gauss para usar.
//  BF.setStdDev (0.01);//el nuevo parámetro de desviación estándar.
//  BF.setInputCloud(cloud_bi);

//  BF.filter(*cloud_bi_filt);

// copyPointCloud(*cloud_bi_filt, *cloud_);


   /*
    * FILTER GRIDMINIMUN
    * */
//  pcl::GridMinimum <pcl::PointXYZRGBA> GridMini(5.2);
//           GridMini.setResolution(5.2);
//           GridMini.setInputCloud(cloud_);
//           GridMini.filter(*cloud_filtered);

//   pcl::copyPointCloud (*cloud_filtered, *cloud_);

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
   pcl::copyPointCloud (*cloud_aux, *cloud_);
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}
void Viewer::saveFileButtonPressed (){
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}
void Viewer::fVoxelGrid(int value){
PointCloudT::Ptr cloud_filtered (new PointCloudT);
PointCloudT::Ptr cloud_aux (new PointCloudT);

 pcl::copyPointCloud (*cloud_,*cloud_aux);
    pcl::VoxelGrid<PointT> sor;
     sor.setInputCloud (cloud_);
     float realvalue=0.01*value;
     sor.setLeafSize (realvalue,realvalue,realvalue);
     sor.filter (*cloud_filtered);
   pcl::copyPointCloud (*cloud_filtered, *cloud_);

   colorCloudDistances ();
    viewer_->updatePointCloud (cloud_, "cloud");
     pcl::copyPointCloud (*cloud_aux, *cloud_);


    ui->qvtkWidget->update ();



}





Viewer::~Viewer()
{
    delete ui;
}
