#include <btBulletDynamicsCommon.h>
#include "obj_parser.h"
#include "DrawString.hpp"
#include <stdio.h>
//#include <GL/glew.h>
//#include <GL/freeglut.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include "maths_funcs.h"
#include <math.h>
#include <vector> // STL dynamic memory.
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>




static void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType);
GLuint CompileShaders(const char* FragmentShader,const char* VertexShader);
void linkCurrentBuffertoShader(GLuint shaderProgramID);
GLuint generateObjectBuffer(GLfloat vertices[], GLfloat colors[]);
void LoadTexture(const char* texture, int i );




mat4 view = identity_mat4 ();
mat4 persp_proj = identity_mat4 ();
mat4 model = identity_mat4 ();
GLuint shaderProgramIDs[3];

class info{
 public:
  int type;
  std::string typeS;
 };

class Sphere;

 class world{
 public:
   Sphere* sky;
   GLuint counts[100];
   int rotate_y  = 0;
   int rotate_x  = 0;
   int rotate_z  = 0;
   int rotates_x  = 0;
   int translaterZ = 0;
   int translaterX = 0;
   int rotaterz = 0;
   int translationZ = 0;
   int translationY = 0;
   int translationX = 0;
   int texture = 1;
   int missilemove = 1;
   float explosionX = 0;
   float explosionY = 0;
   float explosionZ = 0;
   char pause = 0;
   
   btRigidBody* bodies[100];
   btDiscreteDynamicsWorld* dynamicsWorld;
   static void display();
   static void display1();
   static void display2();
   static void keypress(unsigned char key, int x, int y);
   void CreateTank();
   void CreateGround();
   void CreatePlane();
   void CreateFall();
   void CreateCylinder();
   void physicsinit();
   void PhysicsEnd();
   
 };

world* theWorld;


using namespace std;
//Bare bone- ASSIMP LOADER
// 
// parameters:
// -filename: path to the file you want to load
// -vao: an uninitialised GLuint to use for generating a vertex attribute object
// -vbo: an uninitialised GLuint to use for generating a vertex buffer object
//
// returns:
// -the number of vertices (necessary for glDrawArrays), 0 if failed to load
//
// --example usage:
// GLuint vao,vbo,count;
// count = loadMesh("filename.dae",&vao,&vbo); //the & is vital
// 
// --drawing
// glBindVertexArray(vao);
// glDrawArrays(GL_TRIANGLES,0,count);
// glBindVertexArray(0);
//
// derived from http://nickthecoder.wordpress.com/2013/01/20/mesh-loading-with-assimp/

GLuint vaos[100];
GLuint vbos[100];
int translateX=0.0;
int translateY= 0.0;
int translateZ = 0.0;


GLuint textureID[7];

class Mesh{
public:
  GLuint vao;
  GLuint vbo;
  GLuint count;
  void Print();
  void Draw(int,int,int);
  Mesh();
  Mesh(GLuint,GLuint,GLuint);
};


void Mesh::Draw(int proj_mat_location,int view_mat_location,int matrix_location){
  cout << vao << "yep " << count << endl ;
  glBindVertexArray(4);
  glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
  glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
  glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
  glDrawArrays(GL_TRIANGLES,0,8292);
  
}
Mesh::Mesh(GLuint _vao,GLuint _vbo ,GLuint _count){
  vao = _vao;
  vbo = _vbo;
  count = _count;

}

void Mesh::Print(){
  cout  << "count: " <<count << " vao: " <<vao  << " vbo: " <<vbo << endl;
}

class Sphere{
  vector<Mesh> meshes;
  vector<GLuint> Textures;
  void LoadTexture(string);

public:
  Sphere();
  Sphere(string);
  void Draw(int,int,int);
  GLuint LoadMesh(const char* );
};


void Drawer(int proj_mat_location,int view_mat_location,int matrix_location){
  cout << proj_mat_location << "g: "<< view_mat_location <<"r: " << matrix_location << endl;
  glBindVertexArray(3);  
  glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
  glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
  glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
  glDrawArrays(GL_TRIANGLES,0,8292);
}

void Sphere::Draw(int proj_mat_location,int view_mat_location,int matrix_location){


//Eurofighter
   btTransform trans;
   vec3 Direction1 = heading_to_direction(theWorld->rotate_y ) ;
   vec3 Direction2 = heading_to_direction(theWorld->rotate_x ) ;
   Direction1 = normalise(Direction1);
   Direction1 = vec3(Direction1.v[0],-Direction2.v[0],Direction1.v[2]);
   vec3 Direction = Direction1;
   Direction = normalise(Direction);
   Direction = vec3(theWorld->rotate_z*Direction.v[0],theWorld->rotate_z*Direction.v[1],theWorld->rotate_z*Direction.v[2]); 
   btVector3 btspeed = btVector3(Direction.v[0],Direction.v[1],Direction.v[2]);
   theWorld->bodies[3]->setLinearVelocity(btspeed);
   theWorld->bodies[3]->getMotionState()->getWorldTransform(trans);
   glUseProgram (shaderProgramIDs[0]);
   
   
   view = identity_mat4 ();
   persp_proj = perspective(45.0, (float)5/(float)5, 0.1, 1000.0);
   model = identity_mat4 ();
   mat4 model1 = identity_mat4 ();
   
   view = translate (view, vec3 (theWorld->translationX - trans.getOrigin().getX(), theWorld->translationY - trans.getOrigin().getY(), theWorld->translationZ- trans.getOrigin().getZ()));
   
   
   mat4 local1 = identity_mat4 ();
   local1 = scale (local1, vec3 (0.04, 0.04, 0.04  ));
   local1 = rotate_y_deg (local1, 180); 
   local1 = rotate_x_deg (local1, 15); 
   
   local1 = rotate_x_deg (local1, theWorld->rotate_x); 
   local1 = rotate_y_deg (local1, direction_to_heading(Direction)); 
   view = rotate_y_deg (view, -direction_to_heading(Direction)); 
   view = rotate_x_deg (view, -theWorld->rotate_x); 
   
   
   local1 = translate (local1, vec3 (trans.getOrigin().getX()-theWorld->translationX,trans.getOrigin().getY()- theWorld->translationY, trans.getOrigin().getZ()- theWorld->translationZ));
   view = translate (view, vec3 (0,-3,-15- translateZ));
   model = model1*local1;


  
   for (int i = 3 ; i < meshes.size() ;i++){
     //     meshes[i].Draw(proj_mat_location,view_mat_location,matrix_location);
   }
   
   //     Drawer(proj_mat_location,view_mat_location,matrix_location);   
   
   for (int i = 0; i < meshes.size(); i++){
     glBindVertexArray(meshes[i].vao);  
     glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
     glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
     glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
     glDrawArrays(GL_TRIANGLES,0,meshes[i].count);
    }
   
   

}

GLuint Sphere::LoadMesh(const char *filename)
{

  cout << "Enter" << endl;
  GLuint vao  ,vbo  ;



	const aiScene *scene = aiImportFile(filename,aiProcessPreset_TargetRealtime_Fast);
	if(scene)
	{

		for (int number = 0; number < scene->mNumMeshes;number++){
	      
		  cout << "number: " << number << endl;		 
      		  aiMesh *mesh = scene->mMeshes[number];
      		  unsigned int numVertices = mesh->mNumFaces*3;
		  
		  cout << "numVertices: " << numVertices << endl;		 
		  float *attributes = (float*)malloc(sizeof(float)*8*numVertices);
	     
		  GLuint count = 0;
		  for(unsigned int i = 0 ; i < mesh->mNumFaces ; i++)
		    {
		      const aiFace& face = mesh->mFaces[i];
		      for(unsigned int j = 0 ; j < 3 ; j++)
			{
			  aiVector3D vert = mesh->mVertices[face.mIndices[j]];
			  aiVector3D norm = mesh->mNormals[face.mIndices[j]];
			  aiVector3D uv(0.0f,0.0f,0.0f);
			  if(mesh->HasTextureCoords(0))
			    {
			      uv = mesh->mTextureCoords[0][face.mIndices[j]];
			    }
			  //verts
			  attributes[count] = vert.x;
			  attributes[count+1] = vert.y;
			  attributes[count+2] = vert.z;
			  
			  //normals
			  attributes[count+3] = norm.x;
			  attributes[count+4] = norm.y;
			  attributes[count+5] = norm.z;
			  
			  //uvs
			  attributes[count+6] = uv.x;
			  attributes[count+7] = uv.y;
			  count+=8;
			}
		    }


		  //create vao etc
		  glGenVertexArrays(1,&vao);
		  glBindVertexArray(vao);


		glGenBuffers(1,&vbo);
		glBindBuffer(GL_ARRAY_BUFFER,vbo);
		glBufferData(GL_ARRAY_BUFFER,count*sizeof(float),attributes,GL_STATIC_DRAW);

		//vertices => should be layout = 0 in shader
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)0);

		//normals => should be layout = 1 in shader
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)(sizeof(float)*3));

		//uvs => should be layout = 2 in shader
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)(sizeof(float)*6));
		
		//done
		glBindVertexArray(0);
		free(attributes);
	     	Mesh newer(vao,vbo,numVertices);
		newer.Print();
		meshes.push_back(newer);
		//		return numVertices;
		cout << "vao: " <<vao  << "vbo: " << vbo << endl;
		}
		
	}
	aiReleaseImport(scene);
	return 0;
}



Sphere::Sphere(){
  string Tex = "download.jpg";
  LoadTexture(Tex);
}

Sphere::Sphere(string Tex){
  LoadTexture(Tex);
}


void Sphere::LoadTexture(string Texture){


	int x,y,n;	
	GLuint TexID;
	unsigned char* data = stbi_load(Texture.c_str(), &x, &y, &n, 4);
	glGenTextures(1, &TexID);
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, TexID);
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data); 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	Textures.push_back(TexID);

}




void world::CreateTank(){
  
  btCollisionShape* tankShape = new btSphereShape(20);
  
  
  info* store1 =  new info;
  store1->type = 11;
  void* ptr1 = store1;
  btDefaultMotionState* tankMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 10, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    tankRigidBodyCI(1, tankMotionState, tankShape, btVector3(0, 0, 0));
  tankRigidBodyCI.m_restitution = .9;
  btRigidBody* tankRigidBody = new btRigidBody(tankRigidBodyCI);
  dynamicsWorld->addRigidBody(tankRigidBody,1,10);
  //##############################################################################################################
  
  
  this->bodies[0] = tankRigidBody;
  tankRigidBody->setActivationState(DISABLE_DEACTIVATION);
  this->bodies[0]->setLinearVelocity(btVector3(2,2,0));
  
  this->bodies[0]->setUserPointer(store1);
  
  
}


void world::CreateCylinder(){
  
  btCollisionShape* tankShape = new btCylinderShape(btVector3(100, 2000, 200));
  
  
  info* store5 = new info;
  store5->type = 16;
  store5->typeS = "Cylinder";
  void* ptr5 = store5;
  btDefaultMotionState* tankMotionState = new btDefaultMotionState();
  btRigidBody::btRigidBodyConstructionInfo
    tankRigidBodyCI(0, tankMotionState, tankShape, btVector3(0, 0, 0));
  tankRigidBodyCI.m_restitution = 1;
  btRigidBody* tankRigidBody = new btRigidBody(tankRigidBodyCI);
  dynamicsWorld->addRigidBody(tankRigidBody,1,11);
  //##############################################################################################################
  
  
  this->bodies[4] = tankRigidBody;
  tankRigidBody->setActivationState(DISABLE_DEACTIVATION);
   this->bodies[4]->setUserPointer(store5);
   cout << ((btCylinderShape*)tankShape)->getHalfExtentsWithMargin().getX() << endl;
   cout << ((btCylinderShape*)tankShape)->getHalfExtentsWithoutMargin().getX() << endl;
   cout << ((btCylinderShape*)tankShape)->getName()<< endl;

  
  
}




void world::CreateGround(){
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1, 0), 1);
 
  info* store3 = new info;
  store3->type = 12;
  void* ptr3 = store3;
  
  
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  groundRigidBodyCI.m_restitution = .9;
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody,4,15);
  
  //##############################################################################################################
  
  this->bodies[2] = groundRigidBody;
  
  groundRigidBody->setActivationState(DISABLE_DEACTIVATION); 
  this->bodies[2]->setUserPointer(store3);
}


void world::CreatePlane(){
  
  btCollisionShape* planeShape = new btSphereShape(1);


  
  info* store4 = new info;
  store4->type = 13;
  void* ptr4 = store4;
  
  
  
  btDefaultMotionState* planeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 30, 0)));
  btScalar mass = 1;
  btVector3 planeInertia(0, 0, 0);
  planeShape->calculateLocalInertia(mass, planeInertia);
  btRigidBody::btRigidBodyConstructionInfo planeRigidBodyCI(mass, planeMotionState, planeShape, planeInertia);
  planeRigidBodyCI.m_restitution = 1;
  btRigidBody* planeRigidBody = new btRigidBody(planeRigidBodyCI);
  dynamicsWorld->addRigidBody(planeRigidBody,8,15);
  //##############################################################################################################
  
  
  this->bodies[3] = planeRigidBody; 
  planeRigidBody->setActivationState(DISABLE_DEACTIVATION); 
  this->bodies[3]->setUserPointer(store4);
  
}



void world::CreateFall(){
  
  btCollisionShape* fallShape = new btSphereShape(1);
  
  //##############################################################################################################


  
  info* store2  =  new info;
  store2->type = 10;
  void* ptr2 = store2;
  info* hert = (info*) ptr2;
  //cout << "ddd" <<hert->type <<endl; 
  btScalar mass = 1;
  btVector3 planeInertia(0, 0, 0);
  
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 20, 0)));
  mass = 10;
  fallShape->calculateLocalInertia(mass, planeInertia);
  btRigidBody::btRigidBodyConstructionInfo
    fallRigidBodyCm(mass, fallMotionState, fallShape, btVector3(0, 0, 0));
  fallRigidBodyCm.m_restitution = .9;
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCm);
  dynamicsWorld->addRigidBody(fallRigidBody,2,15);
  
  
  this->bodies[1] = fallRigidBody; 
  fallRigidBody->setActivationState(DISABLE_DEACTIVATION);
  this->bodies[1]->setLinearVelocity(btVector3(.2,2,0));
  this->bodies[1]->setUserPointer(store2);
  
}

void world::physicsinit(){
  
	 //Build the broadphase
     btBroadphaseInterface* broadphase = new btDbvtBroadphase();

     // Set up the collision configuration and dispatcher
     btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
     btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

     // The actual physics solver
     btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

     // The world.
     btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -1, 0));

     // Do_everything_else_here


	 this->dynamicsWorld= dynamicsWorld;
 }

void world::PhysicsEnd(){

	 theWorld->dynamicsWorld->removeRigidBody(theWorld->bodies[0]);
	 delete theWorld->bodies[0]->getMotionState();
	 delete theWorld->bodies[0];

	 theWorld->dynamicsWorld->removeRigidBody(theWorld->bodies[1]);
	 delete theWorld->bodies[1]->getMotionState();
	 delete theWorld->bodies[1];


   theWorld->dynamicsWorld->removeRigidBody(theWorld->bodies[2]);
	 delete theWorld->bodies[2]->getMotionState();
	 delete theWorld->bodies[2];

	 theWorld->dynamicsWorld->removeRigidBody(theWorld->bodies[3]);
	 delete theWorld->bodies[3]->getMotionState();
	 delete theWorld->bodies[3];


 
	 /*
     // Clean up behind ourselves like good little programmers
     delete dynamicsWorld;
     delete solver;
     delete dispatcher;
     delete collisionConfiguration;
     delete broadphase;
	 */
 }


//C reate a NULL-terminated string by reading the provided file
char* readShaderSource(const char* shaderFile) ;

GLuint LoadMesh(const char *filename,GLuint *counter,GLuint *vaor, GLuint *vbor,int meshno);


// Macro for indexing vertex buffer
#define BUFFER_OFFSET(i) ((char *)NULL + (i))









void updateScene() {	

		// Wait until at least 16ms passed since start of last frame (Effectively caps framerate at ~60fps)
	static int  last_time = 0;
	int  curr_time = time(NULL);
	float  delta = (curr_time - last_time) * 0.001f;
	if (delta > 0.03f)
		delta = 0.03f;
	last_time = curr_time;

	// Draw the next frame
	glutPostRedisplay();
}
int hello = 0;

void world::keypress(unsigned char key, int x, int y){

switch(key){
case 'y':
	theWorld->pause = 0;
	break;
case 'h':
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
theWorld->texture  =2;
//cout << "theWorld->texture: " << theWorld->texture << endl;
	break;
case 'b':
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
theWorld->texture  =3;
//cout << "texture: " << theWorld->texture << endl;	break;
case 'n':
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
theWorld->texture = 4;
//cout << "texture: " << theWorld->texture << endl;
	break;
case '9':
theWorld->texture++;
	break;
case '7':
cout << theWorld->translationX++;
	break;
case '8':
cout << theWorld->translationX--;
	break;
case 'c':
cout << theWorld->translationY++;
	break;
case 'v':
cout << theWorld->translationY--;
	break;
case '4':
cout << translateZ++;
	break;
case '5':
cout << translateZ--;
	break;
case '1':
theWorld->translationZ++;
	break;
case '2':
cout << "theWorld->translationZ: " << theWorld->translationZ--;
	break;
case '3':
theWorld->translationZ = -translateZ;
	break;
case 'q':
theWorld->rotates_x++;
	break;
case 'w':
theWorld->rotates_x--;
	break;
case 's':
theWorld->rotate_y++;
	break;

case 'a':
theWorld->rotate_y--;
	break;

case 'x':
theWorld->rotate_z++;
	break;

case 'z':
theWorld->rotate_z--;
	break;
case 'i':
  glutDisplayFunc(world::display1);
	//cout<< "translateX" << translateX<< endl;
	//cout<< "translateY" << translateY<< endl;
	//cout<< "translateZ" << translateZ<< endl;
	//cout << "rotate_x" << theWorld->rotate_x << endl;
	//cout << "rotate_y" << theWorld->rotate_y << endl;
	//cout << "rotate_z" << theWorld->rotate_z << endl;
	break;
case 'p':
	theWorld->missilemove= 1;
	//cout << "HElo" << endl;
	break;
case 'l':
	theWorld->missilemove = 0;
	//cout << "HElo" << endl;
	break;
case 'm':
theWorld->explosionX++;
	break;
case 'k':
theWorld->explosionX--;
	break;
}

}




void init2();





void DrawSurface(int proj_mat_location,int view_mat_location,int matrix_location){


  btTransform trans;
  
  mat4 model1 = identity_mat4();
  mat4 local1 = identity_mat4 ();
  mat4 view  = identity_mat4 ();
  //plane
  
  glUseProgram (shaderProgramIDs[0]);
  glBindVertexArray(vaos[1]);
  glBindTexture(GL_TEXTURE_2D, textureID[7]);
  
  	view = translate (view, vec3 (theWorld->translationX , theWorld->translationY - 200, theWorld->translationZ));
  theWorld->bodies[2]->getMotionState()->getWorldTransform(trans);
  
  local1 = scale(local1, vec3(500,500,500)); 
  local1 = translate (local1, vec3(trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ()));
  
  
  // update uniforms & draw
  mat4 model = model1*local1;
  glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
  glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
  glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
  
  glDrawArrays(GL_TRIANGLES,0,theWorld->counts[1]);
}



void UpdateVelocity(){
        btTransform trans;

        theWorld->bodies[0]->getMotionState()->getWorldTransform(trans);
	btVector3 first= trans.getOrigin(); 
       theWorld->bodies[3]->getMotionState()->getWorldTransform(trans);
	btVector3 second = trans.getOrigin(); 
	second = second - first;
	btVector3 tankvelocity = 0.1*second;//btVector3(2,0,0);
	theWorld->bodies[0]->setLinearVelocity(tankvelocity);
}



void DrawTank(int proj_mat_location,int view_mat_location,int matrix_location){

  UpdateVelocity();

   btTransform trans;
   theWorld->bodies[0]->getMotionState()->getWorldTransform(trans);
   mat4 local1 = identity_mat4 ();
   mat4 model1 = identity_mat4 ();
   local1 = translate (local1, vec3 (trans.getOrigin().getX(),trans.getOrigin().getY(), trans.getOrigin().getZ()));
   glUseProgram (shaderProgramIDs[0]);
   glBindTexture(GL_TEXTURE_2D, textureID[1]);
   for (int i = 4; i < 21 ; i++ )	{
     if (i == 20){
       glBindTexture(GL_TEXTURE_2D, textureID[0]);
     }
     glBindVertexArray(vaos[i]);
     
	
     // update uniforms & draw
     mat4 model = model1*local1;
     
     glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
     glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
     glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
     glDrawArrays(GL_TRIANGLES,0,theWorld->counts[i]);	
   }
   
}

void DrawEuroFighter(int proj_mat_location,int view_mat_location,int matrix_location){

//Eurofighter
   btTransform trans;
   vec3 Direction1 = heading_to_direction(theWorld->rotate_y ) ;
   vec3 Direction2 = heading_to_direction(theWorld->rotate_x ) ;
   Direction1 = normalise(Direction1);
   Direction1 = vec3(Direction1.v[0],-Direction2.v[0],Direction1.v[2]);
   vec3 Direction = Direction1;
   Direction = normalise(Direction);
   Direction = vec3(theWorld->rotate_z*Direction.v[0],theWorld->rotate_z*Direction.v[1],theWorld->rotate_z*Direction.v[2]); 
   btVector3 btspeed = btVector3(Direction.v[0],Direction.v[1],Direction.v[2]);
   theWorld->bodies[3]->setLinearVelocity(btspeed);
   theWorld->bodies[3]->getMotionState()->getWorldTransform(trans);
   glUseProgram (shaderProgramIDs[0]);
   
   
   mat4 view = identity_mat4 ();
   persp_proj = perspective(45.0, (float)5/(float)5, 0.1, 1000.0);
   mat4 model = identity_mat4 ();
   mat4 model1 = identity_mat4 ();
   
   view = translate (view, vec3 (theWorld->translationX - trans.getOrigin().getX(), theWorld->translationY - trans.getOrigin().getY(), theWorld->translationZ- trans.getOrigin().getZ()));
   
   
   mat4 local1 = identity_mat4 ();
   local1 = scale (local1, vec3 (0.04, 0.04, 0.04  ));
   local1 = rotate_y_deg (local1, 180); 
   local1 = rotate_x_deg (local1, 15); 
   
   local1 = rotate_x_deg (local1, theWorld->rotate_x); 
   local1 = rotate_y_deg (local1, direction_to_heading(Direction)); 
   view = rotate_y_deg (view, -direction_to_heading(Direction)); 
   view = rotate_x_deg (view, -theWorld->rotate_x); 
   
   
   local1 = translate (local1, vec3 (trans.getOrigin().getX()-theWorld->translationX,trans.getOrigin().getY()- theWorld->translationY, trans.getOrigin().getZ()- theWorld->translationZ));
   view = translate (view, vec3 (0,-3,-15));
   model = model1*local1;

   

   for (int i = 0; i < 4; i++){
     cout << "vaos: " << vaos[28 +i] << endl;
     glBindVertexArray(vaos[28 + i]);
     glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
     glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
     glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
     glDrawArrays(GL_TRIANGLES,0,theWorld->counts[28 + i]);

   }
   
   
}


/*
#################################################################3
##################################################################
######################################################################

DISPLAY FUNCTIONS


#########################################################################3
######################################################################3
*/




void world::display(){
  glEnable (GL_DEPTH_TEST); // enable depth-testing
  glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_BLEND);					// Disable Blend
  glClearColor (0.4f, 0.4f, 0.7f, 1.0f);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




//##############################################################################################
//Score Board, must be drawn last if blending 
char h[]  = "Loading";
 DrawString(h,7,300,450,theWorld->counts,vaos,vbos);

//cout <<"max: " << GL_MAX_TEXTURE_SIZE << endl;

 glUseProgram (shaderProgramIDs[1]);
 glBindVertexArray(vaos[21]);
 glBindTexture(GL_TEXTURE_2D, textureID[5]);
 glDrawArrays(GL_TRIANGLES,0,theWorld->counts[21]);
 glBindVertexArray(0);
 glutSwapBuffers();

	// Set up your objects and shaders
 init2();
 glutDisplayFunc(world::display1);
}





float here;

void world::display1(){
	glDisable(GL_BLEND);	
	glEnable (GL_DEPTH_TEST); 
	glDepthFunc (GL_LESS); 					// Disable Blending
	glClear(GL_COLOR_BUFFER_BIT);

	glClearColor (1.0f, 1.0f, 1.0f, 1.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
here = here + 0.2;
if (here == 720)
	here = 0;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


	int matrix_location = glGetUniformLocation (shaderProgramIDs[0], "model");
	int view_mat_location = glGetUniformLocation (shaderProgramIDs[0], "view");
	int proj_mat_location = glGetUniformLocation (shaderProgramIDs[0], "proj");
	
 	int width = 5;
	int height = 5;	// Root of the Hierarchy

	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 1000.0);
	mat4 view = identity_mat4 ();
	mat4 model = identity_mat4 ();
	mat4 model1 = identity_mat4 ();
	


	view = rotate_x_deg(view, 15);
	view = translate (view, vec3 (-5, -2, -50));
	// update uniforms & draw


	if(theWorld->rotate_z > 0){
	mat4 local1 = identity_mat4 ();
	local1 = rotate_y_deg(local1,here/0.5);
	local1 = scale(local1, vec3 (0.3, 0.3, 0.3  ));
//Tank
glUseProgram (shaderProgramIDs[0]);

	glBindTexture(GL_TEXTURE_2D, textureID[1]);
for (int i = 4; i < 21 ; i++ )	{
	if (i == 20){
		glBindTexture(GL_TEXTURE_2D, textureID[0]);
	}
	glBindVertexArray(vaos[i]);
	// update uniforms & draw
	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
	glDrawArrays(GL_TRIANGLES,0,theWorld->counts[i]);	
	}
   
}else{

//################################################################################
//Eurofighter

	mat4 local1 = identity_mat4 ();
	local1 = rotate_y_deg(local1,here/0.5);
	local1 = scale(local1, vec3 (0.2, 0.2, 0.2  ));
	local1 = translate(local1, vec3 (-3.0, 0.0, 0.0  ));
glUseProgram (shaderProgramIDs[0]);

      	model = model1*local1;

for (int i = 0; i < 4; i++){
	glBindVertexArray(vaos[28 + i]);

	// update uniforms & draw


	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,theWorld->counts[28 + i]);

	}

}
//##############################################################################################
//Writing, must be drawn last if blending 
char str[] = "Select Vehicle";
 DrawString(str,14,200,550,theWorld->counts,vaos,vbos );

 
 glBindTexture(GL_TEXTURE_2D, textureID[5]);
 glUseProgram (shaderProgramIDs[1]);
 glBindVertexArray(vaos[21]);
 glDrawArrays(GL_TRIANGLES,0,theWorld->counts[21]);
 glBindVertexArray(0);
 
 if(theWorld->explosionX > 1)
   glutDisplayFunc(world::display2);
 glutSwapBuffers();
}




void world::display2(){
     

	//explosionsarray[explosions]->DrawExplosion();

	
	// NB: Make the call to draw the geometry in the currently activated vertex buffer. This is where the GPU starts to work!
	glEnable (GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"
	glClear(GL_COLOR_BUFFER_BIT);					// Disable Depth Testing
//	glBlendFunc(GL_SRC_ALPHA,GL_ONE);				// Type Of Blending To Perform
	glDisable(GL_BLEND);					// Disable Blend
	glClearColor (1.0f, 1.0f, 1.0f, 1.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

       

//####################     BULLET   ####################################################


        theWorld->dynamicsWorld->stepSimulation(1 / 60.f, 0);

        btTransform trans;
	//        theWorld->bodies[0]->getMotionState()->getWorldTransform(trans);
	   //     std:://cout << "sphere height: " << trans.getOrigin().getY() ;
	    //    std:://cout << "sphere height: " << trans.getOrigin().getX() ;
	   //     std:://cout << "sphere height: " << trans.getOrigin().getZ() ;

          theWorld->bodies[3]->getMotionState()->getWorldTransform(trans);
	   //     std:://cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

	  
   int numManifolds = theWorld->dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  theWorld->dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

	//btRigidBody* obA2;
	//btRigidBody* obB2;

	//obA2->upcast(obA);
	//obB2->upcast(obB);
        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
		btManifoldPoint& pt = contactManifold->getContactPoint(j);
		if (pt.getDistance()<0.f)
            	{
                	const btVector3& ptA = pt.getPositionWorldOnA();
               		const btVector3& ptB = pt.getPositionWorldOnB();
                	const btVector3& normalOnB = pt.m_normalWorldOnB;
			cout << "X: " << ptA.getX() << " Y: " <<ptA.getY()<< " Z: "  <<ptA.getZ()  << " Distance:"<< pt.getDistance() << endl;
			if (1)
			{
				
				theWorld->explosionX =ptA.getX();
				theWorld->explosionY =ptA.getY();
				theWorld->explosionZ =ptA.getZ();
				
			}
			info* hell = (info*)obA->getUserPointer();
						cout << "cjcjcj: " <<  (int) hell->type << "fe"  << endl;
			hell = (info*)obB->getUserPointer();
						cout << "cjcjcj: " <<  (int)hell->type << "fe"  << endl;
			//char h = getchar();
            	}
            
        }
    }


//####################     DRAWING   ####################################################






//Declare your uniform variables that will be used in your shader
	int matrix_location = glGetUniformLocation (shaderProgramIDs[0], "model");
	int view_mat_location = glGetUniformLocation (shaderProgramIDs[0], "view");
	int proj_mat_location = glGetUniformLocation (shaderProgramIDs[0], "proj");
	int Ambient_location =  glGetUniformLocation (shaderProgramIDs[2], "Ambient");
	int Diffuse_location =  glGetUniformLocation (shaderProgramIDs[2], "Diffuse");
	int Specular_location =  glGetUniformLocation (shaderProgramIDs[0], "Specular");
	theWorld->rotate_x  =+ theWorld->rotates_x;
 	int width = 5;
	int height = 5;	// Root of the Hierarchy
	mat4 local1 = identity_mat4 ();
	






//##############################################################################################

//DrawEuroFighter(proj_mat_location,view_mat_location, matrix_location);

	theWorld->sky->Draw(proj_mat_location,view_mat_location, matrix_location);

// glViewport(0, 0, 400, 300);
 DrawSurface(proj_mat_location,view_mat_location, matrix_location);

//##############################################################################################



//	DrawTank(proj_mat_location,view_mat_location, matrix_location);

//Score Board, must be drawn last if blending 
char h[]  = "Lives: ";
 DrawString(h,7,0,0,theWorld->counts,vaos,vbos);

	glBindTexture(GL_TEXTURE_2D, textureID[5]);

glUseProgram (shaderProgramIDs[1]);
	glBindVertexArray(vaos[21]);
	

	glDrawArrays(GL_TRIANGLES,0,theWorld->counts[21]);
	glBindVertexArray(0);

    glutSwapBuffers();
}


/*
#################################################################3
##################################################################
######################################################################

Open screen DISPLAY


#########################################################################3
######################################################################3
*/






void init(){
//###########################################################################################
	int x,y,n;	
	unsigned char* data = stbi_load("download.jpg", &x, &y, &n, 4);
	glGenTextures(1, &textureID[5]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[5]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	 shaderProgramIDs[1] = CompileShaders("FragmentShader2.txt","VertexShader2.txt");	
}





void init2()
{

  const char* textures1[] = {
    "stridsvagn103/Body_d4.jpg",
    "stridsvagn103/Details_d.jpg",
    "dark.jpg",
    "dark1.jpg",
    "dark2.jpg",
    "dark2.jpg",
    "maxresdefault.jpg",
    "earth.png",
    "light.jpg",
    "light1.jpg",
    "light2.jpg",
    "light3.jpg",
    "light4.jpg",
    "light.jpg",
    "sky.jpg",
  };


  for (int i =0; i < 15; i++){
    if(i != 5 && i != 13 ){
      cout << textures1[i]  << endl;
    LoadTexture(textures1[i],i);
    }
  }


  const char* meshes1[] ={
	  "plane.obj",
	  "sphere.obj",
	  "Missile.obj",
	  "stridsvagn103.obj",
	  "skybox.obj",
	  "building1.obj",
	  "building2.obj",
	  "building3.obj",
	  "building4.obj",
	  "building5.obj",
	  "Models/eurofighter obj/eurofighter obj.obj",
	  "skydome.obj",
	};
	//	char** temp = &meshes;
	for (int i = 0 ; i < 12; i++){
	  LoadMesh(meshes1[i],(theWorld->counts+1),&vaos[1],&vbos[1],0);
	}

 

// Set up the shaders*/
	 shaderProgramIDs[0] = CompileShaders("FragmentShader1.txt","VertexShader1.txt");
	 shaderProgramIDs[2] = CompileShaders("FragmentShader3.txt","VertexShader3.txt");




}

int main(int argc, char** argv){
  //	glewExperimental = GL_TRUE; 
  
  theWorld = new world;

//theWorld  = new world;
	// Check for any errors

 // Set up the window
 glutInit(&argc, argv);
 glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB);
 glutInitWindowSize(1000, 1000);
 glutCreateWindow("My Game");
 // Tell glut where the display function is
 glutDisplayFunc(world::display);
 glutIdleFunc(updateScene);
 glutKeyboardFunc(world::keypress);
 // A call to glewInit() must be done after glut is initialized!
 glewExperimental = GL_TRUE;
 GLenum res = glewInit();

    if (res != GLEW_OK) {
      fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
      return 1;
    }


  theWorld->sky = new Sphere();
  theWorld->sky->LoadMesh("Models/eurofighter obj/eurofighter obj.obj");

    theWorld->physicsinit();
    //    theWorld->CreateTank();
    theWorld->CreatePlane();
    theWorld->CreateGround();
    //    theWorld->CreateFall();
    theWorld->CreateCylinder();
    init();
	// Begin infinite event loop
//while(1)
{
//	glutMainLoopEvent();

}
	glutMainLoop();

	//cout << "End!!!!" << endl;
    return 0;
}









// Shader Functions- click on + to expand
#pragma region SHADER_FUNCTIONS
static void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType)
{
	// create a shader object
    GLuint ShaderObj = glCreateShader(ShaderType);

    if (ShaderObj == 0) {
        fprintf(stderr, "Error creating shader type %d\n", ShaderType);
        exit(0);
    }

	const char* pShaderSource = readShaderSource( pShaderText);
	// Bind the source code to the shader, this happens before compilation
	glShaderSource(ShaderObj, 1, (const GLchar**)&pShaderSource, NULL);
	// compile the shader and check for errors
    glCompileShader(ShaderObj);
    GLint success;
	// check for shader related errors using glGetShaderiv
    glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar InfoLog[1024];
        glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
        fprintf(stderr, "Error compiling shader type %d: '%s'\n", ShaderType, InfoLog);
        exit(1);
    }
	// Attach the compiled shader object to the program object
    glAttachShader(ShaderProgram, ShaderObj);
}

GLuint CompileShaders(const char* FragmentShader,const char* VertexShader)
{
	//Start the process of setting up our shaders by creating a program ID
	//Note: we will link all the shaders together into this ID
    GLuint shaderProgramID = glCreateProgram();
    if (shaderProgramID == 0) {
        fprintf(stderr, "Error creating shader program\n");
        exit(1);
    }

	// Create two shader objects, otene for the vertex, and one for the fragment shader

    AddShader(shaderProgramID, FragmentShader, GL_FRAGMENT_SHADER);
    AddShader(shaderProgramID, VertexShader, GL_VERTEX_SHADER);

    GLint Success = 0;
    GLchar ErrorLog[1024] = { 0 };


	// After compiling all shader objects and attaching them to the program, we can finally link it
    glLinkProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
    glGetProgramiv(shaderProgramID, GL_LINK_STATUS, &Success);
	if (Success == 0) {
		glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
		fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
        exit(1);
	}

	// program has been successfully linked but needs to be validated to check whether the program can execute given the current pipeline state
    glValidateProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
    glGetProgramiv(shaderProgramID, GL_VALIDATE_STATUS, &Success);
    if (!Success) {
        glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
        exit(1);
    }
	// Finally, use the linked shader program
	// Note: this program will stay in effect for all draw calls until you replace it with another or explicitly disable its use
    glUseProgram(shaderProgramID);
	return shaderProgramID;
}
#pragma endregion SHADER_FUNCTIONS

// VBO Functions - click on + to expand
#pragma region VBO_FUNCTIONS
GLuint generateObjectBuffer(GLfloat vertices[], GLfloat colors[]) {
	GLuint numVertices = 833;
	// Genderate 1 generic buffer object, called VBO
	GLuint VBO;
 	glGenBuffers(1, &VBO);
	// In OpenGL, we bind (make active) the handle to a target name and then execute commands on that target
	// Buffer will contain an array of vertices 
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// After binding, we now fill our object with data, everything in "Vertices" goes to the GPU
	glBufferData(GL_ARRAY_BUFFER, numVertices*7*sizeof(GLfloat), NULL, GL_STATIC_DRAW);
	// if you have more data besides vertices (e.g., vertex colours or normals), use glBufferSubData to tell the buffer when the vertices array ends and when the colors start
	glBufferSubData (GL_ARRAY_BUFFER, 0, numVertices*3*sizeof(GLfloat), vertices);
	glBufferSubData (GL_ARRAY_BUFFER, numVertices*3*sizeof(GLfloat), numVertices*4*sizeof(GLfloat), colors);
return VBO;
}


void linkCurrentBuffertoShader(GLuint shaderProgramID){
	GLuint numVertices = 833;
	// find the location of the variables that we will be using in the shader program
	GLuint positionID = glGetAttribLocation(shaderProgramID, "vPosition");
	GLuint colorID = glGetAttribLocation(shaderProgramID, "vColor");
	// Have to enable this
	glEnableVertexAttribArray(positionID);
	// Tell it where to find the position data in the currently active buffer (at index positionID)
    glVertexAttribPointer(positionID, 3, GL_FLOAT, GL_FALSE, 0, 0);
	// Similarly, for the color data.
	glEnableVertexAttribArray(colorID);
	glVertexAttribPointer(colorID, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(numVertices*3*sizeof(GLfloat)));
}
#pragma endregion VBO_FUNCTIONS



// Create a NULL-terminated string by reading the provided file
char* readShaderSource(const char* shaderFile) {   
    FILE* fp = fopen(shaderFile, "rb"); //!->Why does binary flag "RB" work and not "R"... wierd msvc thing?

    if ( fp == NULL ) { return NULL; }

    fseek(fp, 0L, SEEK_END);
    long size = ftell(fp);

    fseek(fp, 0L, SEEK_SET);
    char* buf = new char[size + 1];
    fread(buf, 1, size, fp);
    buf[size] = '\0';

    fclose(fp);

    return buf;
}


GLuint LoadMesh(const char *filename,GLuint *counter,GLuint *vaor, GLuint *vbor,int meshno)
{


  GLuint *vao ,*vbo ;
  static int mesh_count;



	const aiScene *scene = aiImportFile(filename,aiProcessPreset_TargetRealtime_Fast);
	if(scene)
	{

		for (int number = 0; number < scene->mNumMeshes;number++){
	       
		  if (number != 15){
		    cout << "number: " << number << endl;		  		 
      		  aiMesh *mesh = scene->mMeshes[number];
      		  unsigned int numVertices = mesh->mNumFaces*3;
		  cout << "numVetices: " << numVertices << endl;
		  float *attributes = (float*)malloc(sizeof(float)*8*numVertices);
		  vao  = vaor + mesh_count;
		  vbo = vbor + mesh_count;
		  
		  GLuint count = 0;
		  for(unsigned int i = 0 ; i < mesh->mNumFaces ; i++)
		    {
		      const aiFace& face = mesh->mFaces[i];
		      for(unsigned int j = 0 ; j < 3 ; j++)
			{
			  aiVector3D vert = mesh->mVertices[face.mIndices[j]];
			  aiVector3D norm = mesh->mNormals[face.mIndices[j]];
			  aiVector3D uv(0.0f,0.0f,0.0f);
			  if(mesh->HasTextureCoords(0))
			    {
			      uv = mesh->mTextureCoords[0][face.mIndices[j]];
			    }
			  //verts
			  attributes[count] = vert.x;
			  attributes[count+1] = vert.y;
			  attributes[count+2] = vert.z;
			  
			  //normals
			  attributes[count+3] = norm.x;
			  attributes[count+4] = norm.y;
			  attributes[count+5] = norm.z;
			  
			  //uvs
			  attributes[count+6] = uv.x;
			  attributes[count+7] = uv.y;
			  count+=8;
			}
		    }
		  //create vao etc
		  glGenVertexArrays(1,vao);
		  glBindVertexArray(*vao);


		glGenBuffers(1,vbo);
		glBindBuffer(GL_ARRAY_BUFFER,*vbo);
		glBufferData(GL_ARRAY_BUFFER,count*sizeof(float),attributes,GL_STATIC_DRAW);

		//vertices => should be layout = 0 in shader
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)0);

		//normals => should be layout = 1 in shader
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)(sizeof(float)*3));

		//uvs => should be layout = 2 in shader
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(float)*8,(GLvoid*)(sizeof(float)*6));
		
		//done
		glBindVertexArray(0);
		free(attributes);
		*(counter + mesh_count++) = numVertices;
		//		return numVertices;
		}else{mesh_count++;}
		}
		
	}
	aiReleaseImport(scene);
	
	return 0;
}



void LoadTexture(const char* texture, int i ){

	int x ,y,n;
	unsigned char *data = stbi_load(texture, &x, &y, &n, 4);
	// Create 3 vertices that make up a triangle that fits on the viewport 
	

	glGenTextures(1, &textureID[i]);
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[i]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	

}

