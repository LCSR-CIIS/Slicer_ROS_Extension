/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// ROS1_Module Logic includes
#include "vtkSlicerROS1_ModuleLogic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>
#include <vtkMRMLLinearTransformNode.h>

// STD includes
#include <cassert>

//ROS include
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

//Boost include
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#define M_TO_MM 1000.0
#define RAD_TO_DEG 180.0/M_PI

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerROS1_ModuleLogic);

//----------------------------------------------------------------------------
vtkSlicerROS1_ModuleLogic::vtkSlicerROS1_ModuleLogic()
{
  //Create ROS Node
  int argc = 1;
  char** argv = new char*;
  const std::string name = GetClassName();
  argv[0] = new char[name.size() + 1];
  strcpy(argv[0], name.c_str());
  ros::init(argc, argv, "SlicerROSNode");
  slicerROSnodePtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));
  //ROS Publisher
  slicerPub = slicerROSnodePtr->advertise<std_msgs::Int64>("/slicerPub",1000);
  //ROS Subscriber 
  slicerSub = slicerROSnodePtr->subscribe("joint_states", 1000, &vtkSlicerROS1_ModuleLogic::slicerSubCallback, this);
  int counter = 0;
  ROS_INFO("Node and publisher created, now publishing data");
  //Load URDF from parameter server
  if (!m_urdfModel.initParam("robot_description"))
  {
    std::cout<<"Failed to parse urdf file"<<std::endl;
  }else{
    std::cout<<"URDF loaded"<<std::endl;
    std::cout<<"Robot name is: "<<m_urdfModel.getName()<<std::endl;
  }
  //Initialize tfListener
  m_tfListener_ptr = new tf::TransformListener();
}

//----------------------------------------------------------------------------
vtkSlicerROS1_ModuleLogic::~vtkSlicerROS1_ModuleLogic()
{
  //ROS Node Handle delete
  if(slicerROSnodePtr != NULL){
    //delete slicerROSnodePtr;
  }
  //Delete tfListener
  if(m_tfListener_ptr != NULL){
    delete m_tfListener_ptr;
  }
}

//----------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
  std::cout<<"Node Registered"<<std::endl;
  LoadURDFModelToScene();
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
  std::cout<<"Node added"<<std::endl;
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}

void vtkSlicerROS1_ModuleLogic::Spin()
{
  //Spin ROS node
  if(ros::ok()){
    ros::spinOnce();
  }
  //update the model
  UpdateRobotModel();
}

void vtkSlicerROS1_ModuleLogic::slicerSubCallback(const std_msgs::Int64::ConstPtr& msg){
  std::cout<<"debug3"<<std::endl;
}

void vtkSlicerROS1_ModuleLogic::LoadURDFModelToScene(){
  //Get link from URDF model
  std::map<std::string, urdf::LinkSharedPtr > links = m_urdfModel.links_;
  std::map<std::string, urdf::LinkSharedPtr >::iterator it;
  //Create a model node for each link
  for (it = links.begin(); it != links.end(); it++)
  {
    std::cout<<"Link name: "<<it->first<<std::endl;
    //Convert Geometry pointer to Mesh pointer
    urdf::GeometrySharedPtr geometry;
    //Link offset XYZ and RPY
    double offsetXYZ[3];
    double offsetRPY[3];
    if(it->second->collision != NULL ){
      geometry = it->second->collision->geometry;
      offsetXYZ[0] = it->second->collision->origin.position.x;
      offsetXYZ[1] = it->second->collision->origin.position.y;
      offsetXYZ[2] = it->second->collision->origin.position.z;
      it->second->collision->origin.rotation.getRPY(offsetRPY[0], offsetRPY[1], offsetRPY[2]);
    }else{
      //If no collision geometry, use visual geometry
      if(it->second->visual != NULL){
        geometry = it->second->visual->geometry;
        offsetXYZ[0] = it->second->visual->origin.position.x;
        offsetXYZ[1] = it->second->visual->origin.position.y;
        offsetXYZ[2] = it->second->visual->origin.position.z;
        it->second->visual->origin.rotation.getRPY(offsetRPY[0], offsetRPY[1], offsetRPY[2]);
      }else{
        geometry = NULL;
      }
    }
    //Check if geometry is NULL
    if (geometry == NULL){
      std::cout<<"No geometry found for link: "<<it->first<<std::endl;
      continue;
    }
    urdf::MeshSharedPtr mesh = boost::dynamic_pointer_cast<urdf::Mesh>(geometry);
    //Print mesh filename
    std::cout<<"Mesh filename: "<<mesh->filename<<std::endl;
    //Get package name from urdf
    std::string packagePath = "/home/hongyi/catkin_ws/src/universal_robot/";
    //if contains "package://" prefix, remove it
    if(mesh->filename.find("package://") != std::string::npos){
      mesh->filename.erase(0,10);
    }
    //Get full path of mesh file
    std::string meshPath = packagePath + mesh->filename;
    //print mesh path
    std::cout<<"Mesh path: "<<meshPath<<std::endl;
    //Get polydata from mesh file
    vtkSmartPointer<vtkPolyData> polyData = ReadPolyData(meshPath.c_str());
    //Transform polydata with offset
    vtkSmartPointer<vtkTransform> offsetTransform = vtkSmartPointer<vtkTransform>::New();
    offsetTransform->Translate(offsetXYZ[0]*M_TO_MM, offsetXYZ[1]*M_TO_MM, offsetXYZ[2]*M_TO_MM);
    offsetTransform->RotateZ(offsetRPY[2]*RAD_TO_DEG);
    offsetTransform->RotateY(offsetRPY[1]*RAD_TO_DEG);
    offsetTransform->RotateX(offsetRPY[0]*RAD_TO_DEG);
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(offsetTransform);
    transformFilter->SetInputData(polyData);
    transformFilter->Update();
    polyData = transformFilter->GetOutput();
    //Set mesh to model node
    vtkNew<vtkMRMLModelNode> m;
    m->SetAndObservePolyData(polyData);
    //Set model node name
    m->SetName(it->first.c_str());
    //Set model node display node
    if (m->GetDisplayNodeID() == NULL){
      vtkNew<vtkMRMLModelDisplayNode> dnode;
      this->GetMRMLScene()->AddNode(dnode.GetPointer());
      dnode->SetName((it->first+"_display").c_str());
      m->SetAndObserveDisplayNodeID(dnode->GetID());
    }
    //Add model node and display node to scene
    if(this->GetMRMLScene() != NULL){
      this->GetMRMLScene()->AddNode(m.GetPointer());
      std::cout << "Model node added to scene" << std::endl;
    }else{
      std::cout<<"MRML Scene is NULL"<<std::endl;
    }
    //add model node to model node vector
    m_modelNodes.push_back(m.GetPointer());
  }
}

//Update model pose
void vtkSlicerROS1_ModuleLogic::UpdateRobotModel(){
  //Get model node from model node vector
  for(int i = 0; i < m_modelNodes.size(); i++){
    vtkMRMLModelNode* modelNode = m_modelNodes[i];
    //Get model node name
    std::string modelName = modelNode->GetName();
    //Get model node pose
    tf::StampedTransform transform;
    try{
      m_tfListener_ptr->lookupTransform(modelName, m_modelNodes[0]->GetName(), ros::Time(0), transform);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //Get 4x4 matrix from tf transform
    vtkSmartPointer<vtkMatrix4x4> matrix = GetMatrixFromTFtransform(transform);
    //Set model node pose
    if(modelNode -> GetParentTransformNode() == NULL){
      vtkNew<vtkMRMLLinearTransformNode> transformNode;
      transformNode->SetName((modelName+"_transform").c_str());
      this->GetMRMLScene()->AddNode(transformNode.GetPointer());
      modelNode->SetAndObserveTransformNodeID(transformNode->GetID());
    }else{
      vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(modelNode->GetParentTransformNode());
      transformNode->SetMatrixTransformFromParent(matrix);
    }
  }
}


//Function that converts STL, DAE or OBJ files to VTK PolyData 
vtkSmartPointer<vtkPolyData> vtkSlicerROS1_ModuleLogic::ReadPolyData(const char* fileName)
{
  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension = vtksys::SystemTools::GetFilenameLastExtension(fileName);
  if (extension == ".stl" || extension == ".STL")
  {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".obj" || extension == ".OBJ")
  {
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else
  {
    vtkErrorMacro("File type not recognized: " << fileName << " only .stl, .STL, .obj, and .OBJ files are supported. Using dummy mesh instead.");
    //Load a dummy mesh
    std::string dummyMeshFileName = "/home/hongyi/SlicerROSExtension-debug/ROS1_Module/Resources/dummy.STL";
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(dummyMeshFileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  //Convert units from meters to millimeters
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Scale(M_TO_MM, M_TO_MM, M_TO_MM);
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputData(polyData);
  transformFilter->Update();
  return transformFilter->GetOutput();
}

//Function to get vtkTransform matrix from tf transform
vtkSmartPointer<vtkMatrix4x4> vtkSlicerROS1_ModuleLogic::GetMatrixFromTFtransform(tf::StampedTransform transform){
  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  matrix->Identity();
  matrix->SetElement(0,0,transform.getBasis()[0][0]);
  matrix->SetElement(0,1,transform.getBasis()[0][1]);
  matrix->SetElement(0,2,transform.getBasis()[0][2]);
  matrix->SetElement(0,3,transform.getOrigin()[0] * M_TO_MM);
  matrix->SetElement(1,0,transform.getBasis()[1][0]);
  matrix->SetElement(1,1,transform.getBasis()[1][1]);
  matrix->SetElement(1,2,transform.getBasis()[1][2]);
  matrix->SetElement(1,3,transform.getOrigin()[1] * M_TO_MM);
  matrix->SetElement(2,0,transform.getBasis()[2][0]);
  matrix->SetElement(2,1,transform.getBasis()[2][1]);
  matrix->SetElement(2,2,transform.getBasis()[2][2]);
  matrix->SetElement(2,3,transform.getOrigin()[2] * M_TO_MM);
  return matrix;
}