#include <btBulletDynamicsCommon.h>
#include "obj_parser.h"
#include <stdio.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include "maths_funcs.h"
#include <math.h>
#include <vector> // STL dynamic memory.
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
//Bare bones - ASSIMP LOADER
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

#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

using namespace std;

GLuint vaos[100];
GLuint vbos[100];
GLuint counts[100];
int translateX=0.0;
int translateY= 0.0;
int translateZ = 0.0;

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
GLuint textureID[5];
void display();
void display1();

mat4 view = identity_mat4 ();
mat4 persp_proj = identity_mat4 ();
mat4 model = identity_mat4 ();


 


class world{
public:
	btRigidBody* bodies[100];
    btDiscreteDynamicsWorld* dynamicsWorld;
};

world theWorld;



void physicsinit(){

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
 btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

        btCollisionShape* fallShape = new btSphereShape(1);
        btCollisionShape* tankShape = new btSphereShape(1);
        btCollisionShape* planeShape = new btSphereShape(1);

//##############################################################################################################


        btDefaultMotionState* tankMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 10, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                tankRigidBodyCI(1, tankMotionState, tankShape, btVector3(0, 0, 0));
tankRigidBodyCI.m_restitution = .9;
        btRigidBody* tankRigidBody = new btRigidBody(tankRigidBodyCI);
        dynamicsWorld->addRigidBody(tankRigidBody,8,8);
//##############################################################################################################


        btDefaultMotionState* planeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 200, 0)));
        btScalar mass = 1;
        btVector3 planeInertia(0, 0, 0);
        planeShape->calculateLocalInertia(mass, planeInertia);
        btRigidBody::btRigidBodyConstructionInfo planeRigidBodyCI(mass, planeMotionState, planeShape, planeInertia);
planeRigidBodyCI.m_restitution = .9;
        btRigidBody* planeRigidBody = new btRigidBody(planeRigidBodyCI);
        dynamicsWorld->addRigidBody(planeRigidBody,8,8);
//##############################################################################################################

       btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
groundRigidBodyCI.m_restitution = .9;
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(groundRigidBody,8,8);

//##############################################################################################################

       btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 30, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                fallRigidBodyCI(1, fallMotionState, fallShape, btVector3(0, 0, 0));
fallRigidBodyCI.m_restitution = .9;
        btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
        dynamicsWorld->addRigidBody(fallRigidBody,8,8);


    	theWorld.dynamicsWorld= dynamicsWorld;
	theWorld.bodies[0] = tankRigidBody;
	theWorld.bodies[1] = fallRigidBody; 
	theWorld.bodies[2] = groundRigidBody;
	theWorld.bodies[3] = planeRigidBody; 
	

	fallRigidBody->setActivationState(DISABLE_DEACTIVATION);
	groundRigidBody->setActivationState(DISABLE_DEACTIVATION); 
	tankRigidBody->setActivationState(DISABLE_DEACTIVATION);
	planeRigidBody->setActivationState(DISABLE_DEACTIVATION); 


	theWorld.bodies[0]->setLinearVelocity(btVector3(2,2,0));
	theWorld.bodies[1]->setLinearVelocity(btVector3(.2,2,0));
}

void PhysicsEnd(){

        theWorld.dynamicsWorld->removeRigidBody(theWorld.bodies[0]);
        delete theWorld.bodies[0]->getMotionState();
        delete theWorld.bodies[0];

        theWorld.dynamicsWorld->removeRigidBody(theWorld.bodies[1]);
        delete theWorld.bodies[1]->getMotionState();
        delete theWorld.bodies[1];
  

  theWorld.dynamicsWorld->removeRigidBody(theWorld.bodies[2]);
        delete theWorld.bodies[2]->getMotionState();
        delete theWorld.bodies[2];

        theWorld.dynamicsWorld->removeRigidBody(theWorld.bodies[3]);
        delete theWorld.bodies[3]->getMotionState();
        delete theWorld.bodies[3];


/*

    // Clean up behind ourselves like good little programmers
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
*/
}




void DrawString(char* words, int lenght1,int x,int y ){


float *attributes = (float*)malloc(sizeof(float)*24*lenght1);
float size = 28	;
char* text =words;
for ( unsigned int i=0 ; i<lenght1 ; i++ ){
 
    vec2 vertex_up_left    = vec2( x+i*size     , y+size );
    vec2 vertex_up_right   = vec2( x+i*size+size, y+size );
    vec2 vertex_down_right = vec2( x+i*size+size, y      );
    vec2 vertex_down_left  = vec2( x+i*size     , y      );
 
    attributes[24*i] =vertex_up_left.v[0]   ;
    attributes[24*i+1] =vertex_up_left.v[1] ;

    attributes[24*i+4] =vertex_down_left.v[0]    ;
    attributes[24*i+5] =vertex_down_left.v[1]  ;

    attributes[24*i+8] =vertex_up_right.v[0]   ;
    attributes[24*i+9] =vertex_up_right.v[1] ;

    attributes[24*i+12] =vertex_down_right.v[0]   ;
    attributes[24*i+13] =vertex_down_right.v[1] ;

    attributes[24*i+16] =vertex_up_right.v[0]   ;
    attributes[24*i+17] =vertex_up_right.v[1] ;


    attributes[24*i+ 20] =vertex_down_left.v[0]   ;
    attributes[24*i+21] =vertex_down_left.v[1] ;
}


for ( unsigned int i=0 ; i<lenght1 ; i++ ){
 
char character = text[i];
float uv_x = (character%16)/16.0f;
float uv_y = (character/16)/16.0f;

 	vec2 vertex_up_left    = vec2( uv_x           , 1.0f - uv_y );
   	vec2 vertex_up_right   = vec2( uv_x+1.0f/16.0f, 1.0f - uv_y );
   	vec2 vertex_down_right = vec2( uv_x+1.0f/16.0f, 1.0f - (uv_y + 1.0f/16.0f) );
   	vec2 vertex_down_left  = vec2( uv_x           , 1.0f - (uv_y + 1.0f/16.0f) );
 
 
 

    attributes[24*i+2] =vertex_up_left.v[0]   ;
    attributes[24*i+3] = vertex_up_left.v[1] ;
     
    attributes[24*i+6] =vertex_down_left.v[0]   ;
    attributes[24*i+7] =vertex_down_left.v[1] ;


    attributes[24*i+ 10] =vertex_up_right.v[0]   ;
    attributes[24*i+11] =vertex_up_right.v[1]  ;

    attributes[24*i+14] =vertex_down_right.v[0]    ;
    attributes[24*i+15] =vertex_down_right.v[1]  ;


    attributes[24*i+18] =vertex_up_right.v[0]   ;
    attributes[24*i+19] =vertex_up_right.v[1] ;

   attributes[24*i + 22] =vertex_down_left.v[0]   ;
    attributes[24*i+23] =vertex_down_left.v[1] ;
/*


   attributes[24*i+2] =0  ;
    attributes[24*i+3] = 1 ;
     

    attributes[24*i+6] =0   ;
    attributes[24*i+7] =0 ;


    attributes[24*i+ 10] =1   ;
    attributes[24*i+11] =1 ;

    attributes[24*i+14] =1    ;
    attributes[24*i+15] =0  ;


    attributes[24*i+18] =1   ;

    attributes[24*i+19] =1 ;

   attributes[24*i + 22] =0   ;
    attributes[24*i+23] =0 ;
*/

}

		glGenVertexArrays(1,&vaos[21]);
		glBindVertexArray(vaos[21]);


		glGenBuffers(1,&vbos[21]);
		glBindBuffer(GL_ARRAY_BUFFER,vbos[21]);
		glBufferData(GL_ARRAY_BUFFER,24*lenght1*sizeof(float),attributes,GL_STATIC_DRAW);

		//vertices => should be layout = 0 in shader
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,sizeof(float)*4,(GLvoid*)0);

		//normals => should be layout = 1 in shader
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,sizeof(float)*4,(GLvoid*)(sizeof(float)*2));

free(attributes);
counts[21] = 6*lenght1;
}

GLuint loadMesh(const char *filename, GLuint *vao, GLuint *vbo,int meshno)
{
	const aiScene *scene = aiImportFile(filename,aiProcessPreset_TargetRealtime_Fast);
	if(scene)
	{

		aiMesh *mesh = scene->mMeshes[meshno];

		unsigned int numVertices = mesh->mNumFaces*3;

		float *attributes = (float*)malloc(sizeof(float)*8*numVertices);
	
		GLuint count = 0;
		std::cout << "meshes: " << scene->mNumMeshes << std::endl;
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
		aiReleaseImport(scene);
		return numVertices;
	}
	return 0;
}

// Macro for indexing vertex buffer
#define BUFFER_OFFSET(i) ((char *)NULL + (i))


GLuint shaderProgramIDs[2];


// Vertex Shader (for convenience, it is defined in the main here, but we will be using text files for shaders in future)
// Note: Input to this shader is the vertex positions that we specified for the triangle. 
// Note: gl_Position is a special built-in variable that is supposed to contain the vertex position (in X, Y, Z, W)
// Since our triangle vertices were specified as vec3, we just set W to 1.0.

static const char* pVS = "                                                    \n\
#version 400\n\
\n\
\n\
layout (location =0) in vec3 vertex_position; \n\
layout (location =1)  in vec3 vertex_normal; 				\n\
layout (location =2) in vec2 uvs;     \n\
\n\
out vec3 LightIntensity;\n\
out vec2 UV;   \n\
\n\
vec4 LightPosition = vec4 (100.0, 10.0, 4.0, 1.0); // Light position in world coords.\n\
vec3 Kd = vec3 (0.8, 0.8, 0.8); // green diffuse surface reflectance \n\
vec3 Ld = vec3 (1.0, 1.0, 1.0); // Light source intensity \n\
vec3 Ka = vec3 (0.3, 0.3, 0.3); \n\
vec3 Ks  = vec3 (1.0, 1.0, 1.0); \n\
vec3 globalAmbient = vec3 (0.6, 0.6, 0.6); \n\
\n\
\n\
uniform sampler2D myTextureSampler;      \n\
//uniform sampler2D myTextureSampler2;      \n\
uniform mat4 view;\n\
uniform mat4 proj;\n\
uniform mat4 model;\n\
\n\
void main(){\n\
\n\
//vec3 Kd = normalize ( texture( myTextureSampler, uvs).rgb );  \n\
//vec3 Ka = normalize ( texture( myTextureSampler, uvs).rgb );  \n\
//vec3 Kd = vec3 (vertex_position.x, vertex_position.y, 0.0); // green diffuse surface reflectance \n\
  mat4 ModelViewMatrix = view * model;\n\
  mat3 NormalMatrix =  mat3(ModelViewMatrix);\n\
  // Convert normal and position to eye coords\n\
  // Normal in view space\n\
  vec3 tnorm = normalize( NormalMatrix * vertex_normal);\n\
  // Position in view space\n\
  vec4 eyeCoords = ModelViewMatrix * vec4(vertex_position,1.0);\n\
  //normalised vector towards the light source\n\
 vec3 s = normalize(vec3(LightPosition - eyeCoords));\n\
 //vec3 s = normalize(vec3(LightPosition - vec4(vertex_position,1.0)));\n\
\n\
\n\
  // Compute the specular term\n\
\n\
  vec3 V = normalize(vec3(eyeCoords - LightPosition));//formally V\n\
\n\
  vec3 H = normalize(vec3(vec4(s,1.0) -eyeCoords));\n\
\n\
  float specularLight = pow(max(dot(tnorm, H), 0), 2);\n\
\n\
  if (max( dot( s, tnorm ), 0.0 ) <= 0) specularLight = 0;\n\
\n\
\n\
  \n\
  // The diffuse shading equation, dot product gives us the cosine of angle between the vectors +  Ks * Ld * specularLight\n\
  LightIntensity = globalAmbient*Ka + Ld*Kd * max( dot( s, tnorm ), 0.0 )+  Ks * Ld * specularLight ;\n\
  \n\
  // Convert position to clip coordinates and pass along\n\
 gl_Position = proj * view * model * vec4(vertex_position,0.1);\n\
//LightIntensity =  vertex_position;\n\
  	UV = uvs ; \n\
}";




// Fragment Shader
// Note: no input in this shader, it just outputs the colour of all fragments, in this case set to red (format: R, G, B, A).
static const char* pFS = "                                              \n\
#version 430\n\
\n\
in vec3 LightIntensity;\n\
in vec2 UV  ;   \n\
out vec3 color;                \n\
uniform sampler2D myTextureSampler;      \n\
//uniform sampler2D myTextureSampler2;      \n\
void main(){\n\
	vec2  uvr = vec2(UV.x,1-UV.y); \n\
//	gl_FragColor = vec4 (LightIntensity, 1.0);\n\
color = LightIntensity*texture( myTextureSampler, uvr ).bgr;  \n\
	vec4 text =  texture( myTextureSampler, uvr ).rgba;\n\
\n\
//	gl_FragColor =  vec4(LightIntensity*text.rgb,text.a);\n\
//color  = vec3(UV,UV.y);  \n\
//gl_FragColor = vec4 (UV,1.0, 1.0);  \n\
}";


static const char* pVS1 = "                                              \n\
#version 400\n\
layout (location =0)in vec2 vertexPosition_screenspace;\n\
layout (location =1)in vec2 vertexUV;\n\
out vec2 UV;\n\
void main(){\n\
 \n\
    // Output position of the vertex, in clip space\n\
    // map [0..800][0..600] to [-1..1][-1..1]\n\
    vec2 vertexPosition_homoneneousspace = vertexPosition_screenspace - vec2(400,300); //[0..800][0..600] -> [-400..400][-300..300]\n\
    vertexPosition_homoneneousspace /= vec2(400,300);\n\
    gl_Position =  vec4(vertexPosition_homoneneousspace,-1,1);\n\
 \n\
    // UV of the vertex. No special space for this one.\n\
    UV = vertexUV;\n\
}";

static const char* pFS1 = "                                              \n\
#version 400\n\
out vec3 color;\n\
in vec2 UV;\n\
uniform sampler2D myTextureSampler;      \n\
void main(){ \n\
vec2 uvs = vec2(UV.x,1 - UV.y);\n\
gl_FragColor = texture( myTextureSampler, uvs ).bgra;  \n\
//  color = texture( myTextureSampler, uvs ).rgb; \n\
  //color = vec3(1.0f,0.75f,0.5f); \n\
}";




static const char* pVS2 = "                                                    \n\
#version 400\n\
\n\
\n\
layout (location =0) in vec4 vertex_position; \n\
layout (location =1)  in vec3 vertex_normal; 				\n\
layout (location =2) in vec2 uvs;     \n\
\n\
out vec3 LightIntensity;\n\
out vec2 UV;   \n\
\n\
vec4 LightPosition = vec4 (100.0, 10.0, 4.0, 1.0); // Light position in world coords.\n\
vec3 Kd = vec3 (0.8, 0.8, 0.8); // green diffuse surface reflectance \n\
vec3 Ld = vec3 (1.0, 1.0, 1.0); // Light source intensity \n\
\n\
vec3 Ka = vec3 (0.3, 0.3, 0.3); \n\
vec3 Ks  = vec3 (1.0, 1.0, 1.0); \n\
vec3 globalAmbient = vec3 (0.8, 0.8, 0.8); \n\
\n\
\n\
\n\
uniform sampler2D myTextureSampler;      \n\
//uniform sampler2D myTextureSampler2;      \n\
uniform mat4 view;\n\
uniform mat4 proj;\n\
uniform mat4 model;\n\
\n\
void main(){\n\
\n\
\n\
//vec3 Kd = normalize ( texture( myTextureSampler, uvs).rgb );  \n\
//vec3 Ka = normalize ( texture( myTextureSampler, uvs).rgb );  \n\
//vec3 Kd = vec3 (vertex_position.x, vertex_position.y, 0.0); // green diffuse surface reflectance \n\
\n\
  mat4 ModelViewMatrix = view * model;\n\
  mat3 NormalMatrix =  mat3(ModelViewMatrix);\n\
  // Convert normal and position to eye coords\n\
  // Normal in view space\n\
  vec3 tnorm = normalize( NormalMatrix * vertex_normal);\n\
\n\
  // Position in view space\n\
  vec4 eyeCoords = ModelViewMatrix *vertex_position;\n\
  //normalised vector towards the light source\n\
 vec3 s = normalize(vec3(LightPosition - eyeCoords));\n\
 //vec3 s = normalize(vec3(LightPosition -vertex_position));\n\
\n\
\n\
  // Compute the specular term\n\
\n\
\n\
  vec3 V = normalize(vec3(eyeCoords - LightPosition));//formally V\n\
\n\
  vec3 H = normalize(vec3(vec4(s,1.0) -eyeCoords));\n\
\n\
\n\
  float specularLight = pow(max(dot(tnorm, H), 0), 2);\n\
\n\
  if (max( dot( s, tnorm ), 0.0 ) <= 0) specularLight = 0;\n\
\n\
\n\
\n\
  \n\
  // The diffuse shading equation, dot product gives us the cosine of angle between the vectors +  Ks * Ld * specularLight\n\
\n\
  LightIntensity = globalAmbient*Ka + Ld*Kd * max( dot( s, tnorm ), 0.0 )+  Ks * Ld * specularLight ;\n\
  \n\
  // Convert position to clip coordinates and pass along\n\
 gl_Position =   vec4(proj * view * model *vertex_position);\n\
//  gl_Position =  vec4(vertex_position.x,vertex_position.y,0,1.0);\n\
//LightIntensity =  vertex_position;\n\
  	UV = uvs ; \n\
}";




static const char* pFS2 = "                                              \n\
#version 430\n\
\n\
\n\
in vec3 LightIntensity;\n\
in vec2 UV  ;   \n\
out vec3 color;                \n\
uniform sampler2D myTextureSampler;      \n\
//uniform sampler2D myTextureSampler2;      \n\
void main(){\n\
	vec2  uvr = vec2(UV.x,UV.y); \n\
 	vec3 text = texture( myTextureSampler,uvr ).bgr;\n\
	float alpha = 0.0; \n\
  if (text.x + text.y + text.z > 0.3) alpha = 1.0;\n\
//color = vec3(1.0,1.0,1.0);  \n\
gl_FragColor = vec4(text,alpha);  \n\
//	gl_FragColor =  LightIntensity;\n\
//color = texture( myTextureSampler, uvr ).bgr;  \n\
//gl_FragColor = vec4 (UV,0.0, 1.0);  \n\
}";































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
	// Bind the source code to the shader, this happens before compilation
	glShaderSource(ShaderObj, 1, (const GLchar**)&pShaderText, NULL);
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

	// Create two shader objects, one for the vertex, and one for the fragment shader

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

int missilemove = 0;

void keypress(unsigned char key, int x, int y){

switch(key){
case 'y':
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
texture = 1;
cout << "texture: " << texture << endl;
	break;
case 'h':
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
texture  =2;
cout << "texture: " << texture << endl;
	break;
case 'b':
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
texture  =3;
cout << "texture: " << texture << endl;	break;
case 'n':
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
texture = 4;
cout << "texture: " << texture << endl;
	break;
case '9':
texture++;
	break;
case '7':
translationX++;
	break;
case '8':
translationX--;
	break;
case 'c':
translationY++;
	break;
case 'v':
translationY--;
	break;
case '4':
translateZ++;
	break;
case '5':
translateZ--;
	break;
case '1':
translationZ++;
	break;
case '2':
cout << "translationZ: " << translationZ--;
	break;
case '3':
translationZ = -translateZ;
	break;
case 'q':
rotates_x++;
	break;
case 'w':
rotates_x--;
	break;
case 's':
rotate_y++;
	break;

case 'a':
rotate_y--;
	break;

case 'x':
rotate_z++;
	break;

case 'z':
rotate_z--;
	break;
case 'i':
	glutDisplayFunc(display1);
	cout<< "translateX" << translateX<< endl;
	cout<< "translateY" << translateY<< endl;
	cout<< "translateZ" << translateZ<< endl;
	cout << "rotate_x" << rotate_x << endl;
	cout << "rotate_y" << rotate_y << endl;
	cout << "rotate_z" << rotate_z << endl;
	break;
case 'p':
//	btTransform trans1;
//	btTransform trans2;
//	theWorld.bodies[0]->getMotionState()->getWorldTransform(trans1);
//	theWorld.bodies[2]->getMotionState()->getWorldTransform(trans2);	
//	theWorld.bodies[2]->setLinearVelocity((trans2.getOrigin() - trans1.getOrigin()).normalize);
	missilemove= 1;
	cout << "HElo" << endl;
	break;
case 'l':
	missilemove = 0;
	break;
case 'm':
translaterZ++;
	break;
case 'k':
translaterZ--;
	break;
}

}




void init2();
void display2(){
	glEnable (GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"
	glClear(GL_COLOR_BUFFER_BIT);
//	glDisable(GL_DEPTH_TEST);					// Disable Depth Testing
//	glEnable(GL_BLEND);						// Enable Blending
//	glBlendFunc(GL_SRC_ALPHA,GL_ONE);				// Type Of Blending To Perform
	glDisable(GL_BLEND);					// Disable Blend
	glClearColor (0.2f, 0.2f, 0.7f, 0.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




//##############################################################################################
//Score Board, must be drawn last if blending 
char h[]  = "Loading";
DrawString(h,7,250,250);

cout <<"max: " << GL_MAX_TEXTURE_SIZE << endl;
glUseProgram (shaderProgramIDs[1]);
	glBindVertexArray(vaos[21]);
	
	glBindTexture(GL_TEXTURE_2D, textureID[5]);
	glDrawArrays(GL_TRIANGLES,0,counts[21]);
	glBindVertexArray(0);

    glutSwapBuffers();
	// Set up your objects and shaders
	init2();
	glutDisplayFunc(display);
}





class particle						// Create A Structure For Particle
{
public:
particle()
{
active = true;
life = 1.0;
	fade = 0.01;					// Fade Speed
	r =0;						// Red Value
	g=0;						// Green Value
	b=0;						// Blue Value
	x=((float) rand() / (RAND_MAX))-0.5;		// X Position
	y= ((float) rand() / (RAND_MAX))-0.5;		// Y Position
	z=((float) rand() / (RAND_MAX))-0.5;						// Z Position
	xi = 0.0 ;		// X Direction
	yi  = 0.0;		// Y Direction
	zi = 0.0;		// Z Direction
	xg = 0.0;						// X Gravity
	yg = -0.0;						// Y Gravity
	zg = 0.0;						// Z Gravity
}
public:
	bool	active;					// Active (Yes/No)
	float	life;					// Particle Life
	float	fade;					// Fade Speed
	float	r;						// Red Value
	float	g;						// Green Value
	float	b;						// Blue Value
	float	x;						// X Position
	float	y;						// Y Position
	float	z;						// Z Position
	float	xi;						// X Direction
	float	yi;						// Y Direction
	float	zi;						// Z Direction
	float	xg;						// X Gravity
	float	yg;						// Y Gravity
	float	zg;						// Z Gravity
};	

#define num_particles 5
class explosion{
public:
int life;
explosion(){draws = 0;dead = 0;life = 100;}
int dead;
particle particles[num_particles];
void DrawExplosion();
int draws;
};


void explosion::DrawExplosion(){
float *attribute = (float*)malloc(sizeof(float)*54*num_particles);
for (int i = 0; i < num_particles; i++)
{
    attribute[54*i] = particles[i].x  -0.5;
    attribute[54*i+1] = particles[i].y -0.5;
    attribute[54*i+2] = particles[i].z;
    attribute[54*i+3] = 1.0 /*particles[i].life*/ ;
 attribute[54*i+4] = particles[i].x ;
 attribute[54*i+5] = particles[i].y ;
 attribute[54*i+6] = particles[i].z ;
attribute[54*i+7] = 0.0f;
attribute[54*i+8] = 0.0f;

attribute[54*i + 9] = particles[i].x +0.5 ;
 attribute[54*i+10] = particles[i].y -0.5 ;
    attribute[54*i+11] = particles[i].z;
    attribute[54*i+12] =1.0 ;
attribute[54*i+13] = particles[i].x ;
 attribute[54*i+14] = particles[i].y;
 attribute[54*i+15] = particles[i].z;
    attribute[54*i+16] =0.0f;
attribute[54*i+17] = 1.0f; 

 attribute[54*i+18] = particles[i].x - 0.5  ;
attribute[54*i +19] =  particles[i].y + 0.5;
 attribute[54*i+20] =particles[i].z;
    attribute[54*i+21] =1.0;
attribute[54*i+22] = particles[i].x ;
attribute[54*i+23] = particles[i].y;
 attribute[54*i+24] = particles[i].z ;
    attribute[54*i+25] = 1.0f;
    attribute[54*i+26] = 0.0f;

 attribute[54*i +27] = particles[i].x  +0.5;
    attribute[54*i+28] = particles[i].y -0.5;
    attribute[54*i+29] = particles[i].z;
    attribute[54*i+30] = 1.0 /*particles[i].life*/ ;
 attribute[54*i+31] = particles[i].x ;
 attribute[54*i+32] = particles[i].y ;
 attribute[54*i+33] = particles[i].z ;
attribute[54*i+34] = 0.0f;
attribute[54*i+35] = 1.0f;



attribute[54*i + 36] = particles[i].x - 0.5 ;
 attribute[54*i+37] = particles[i].y  +0.5 ;
    attribute[54*i+38] = particles[i].z;
    attribute[54*i+39] =1.0 ;
attribute[54*i+40] = particles[i].x ;
 attribute[54*i+41] = particles[i].y ;
 attribute[54*i+42] = particles[i].z ;
    attribute[54*i+43] = 1.0f;
attribute[54*i+44] = 0.0f;

 attribute[54*i+45] = particles[i].x +0.5  ;
attribute[54*i +46] =  particles[i].y +0.5;
 attribute[54*i+47] =particles[i].z ;
    attribute[54*i+48] =1.0;
attribute[54*i+49] = particles[i].x ;
attribute[54*i+50] = particles[i].y ;
 attribute[54*i+51] = particles[i].z ;
    attribute[54*i+52] = 1.0f;
    attribute[54*i+53] = 1.0f;


//cout << "particle life:" << particles[i].life << endl;

}
//  attribute[3*i+2] = ((float) rand() / (RAND_MAX)) ;
counts[22] = 6*num_particles;

		glGenVertexArrays(1,&vaos[22]);
		glBindVertexArray(vaos[22]);


		glGenBuffers(1,&vbos[22]);
		glBindBuffer(GL_ARRAY_BUFFER,vbos[22]);
		glBufferData(GL_ARRAY_BUFFER,54*num_particles*sizeof(float),attribute,GL_STATIC_DRAW);

		//vertices => should be layout = 0 in shader
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,4,GL_FLOAT,GL_FALSE,sizeof(float)*9,(GLvoid*)0);

		//normals => should be layout = 1 in shader
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(float)*9,(GLvoid*)(sizeof(float)*4));

		//uvs => should be layout = 2 in shader
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(float)*9,(GLvoid*)(sizeof(float)*7));

free(attribute);
for (int i = 0; i < num_particles ; i++)
{
    particles[i].x  = particles[i].xi -particles[i].xg + particles[i].x ;
    particles[i].y  = particles[i].yi -particles[i].yg + particles[i].y ;
    particles[i].z  = particles[i].zi -particles[i].zg + particles[i].z ;
   // particles[i].xi  = particles[i].xi -particles[i].xg*particles[i].life ;
    //particles[i].yi  = particles[i].yi -particles[i].yg*particles[i].life ;
    //particles[i].zi  = particles[i].zi -particles[i].zg*particles[i].life ;
    particles[i].life  = particles[i].life - particles[i].fade ;
 if(particles[i].life == 0)
		cout << dead++;
}

life = life ;
}

explosion* explosionList;
int explosions = 0;
explosion* explosionsarray[100];


/*
#################################################################3
##################################################################
######################################################################

PRIMARY DISPLAY


#########################################################################3
######################################################################3
*/



void display(){
     


	if (explosions < 2){
		explosion* theexplosion  = new explosion();
		//char h = getchar();
		explosionsarray[explosions] =  theexplosion;
		cout << explosions++;
	}
	//explosionsarray[explosions]->DrawExplosion();

	
	// NB: Make the call to draw the geometry in the currently activated vertex buffer. This is where the GPU starts to work!
	glEnable (GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"
	glClear(GL_COLOR_BUFFER_BIT);
//	glDisable(GL_DEPTH_TEST);					// Disable Depth Testing
//	glEnable(GL_BLEND);						// Enable Blending
//	glBlendFunc(GL_SRC_ALPHA,GL_ONE);				// Type Of Blending To Perform
	glDisable(GL_BLEND);					// Disable Blend
	glClearColor (1.0f, 1.0f, 1.0f, 1.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


//####################     BULLET   ####################################################


        theWorld.dynamicsWorld->stepSimulation(1 / 60.f, 0);

        btTransform trans;
        theWorld.bodies[0]->getMotionState()->getWorldTransform(trans);
	   //     std::cout << "sphere height: " << trans.getOrigin().getY() ;
	    //    std::cout << "sphere height: " << trans.getOrigin().getX() ;
	   //     std::cout << "sphere height: " << trans.getOrigin().getZ() ;

          theWorld.bodies[3]->getMotionState()->getWorldTransform(trans);
	   //     std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;


   int numManifolds = theWorld.dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  theWorld.dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
 	//	cout << "Collision: " << j<<endl; 
		//char h = getchar();
            
        }
    }


//####################     DRAWING   ####################################################



	glBindTexture(GL_TEXTURE_2D, textureID[7]);
	glUseProgram (shaderProgramIDs[0]);
	glBindVertexArray(vaos[1]);
//Declare your uniform variables that will be used in your shader
	int matrix_location = glGetUniformLocation (shaderProgramIDs[0], "model");
	int view_mat_location = glGetUniformLocation (shaderProgramIDs[0], "view");
	int proj_mat_location = glGetUniformLocation (shaderProgramIDs[0], "proj");
	
	rotate_x  =+ rotates_x;
 	int width = 5;
	int height = 5;	// Root of the Hierarchy
	mat4 local1 = identity_mat4 ();



       theWorld.bodies[2]->getMotionState()->getWorldTransform(trans);


	local1 = scale(local1, vec3(100,100,100)); 
	local1 = translate (local1, vec3(trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ()));

	view = identity_mat4 ();
	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 1000.0);
	model = identity_mat4 ();
	mat4 model1 = identity_mat4 ();
	model1 = translate (model1, vec3 (translationX, translationY, translationZ));
	model1 = rotate_y_deg (model1, rotate_y); 
	model1 = rotate_z_deg (model1, rotate_z); 
	model1 = rotate_x_deg (model1, rotate_x); 
//	model1 = translate (model1, vec3 (0, 0, translationZ));
//	model1 = scale(model1, vec3(100,100,100)); 
//	view = translate (view, vec3 (0, -2, translateZ));
	view = rotate_x_deg (view, 20); 
	view = translate (view, vec3 (translateX, translateY -10, translateZ-400));
	// update uniforms & draw
	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[1]);

//##############################################################################################
// SKYBOX
	glBindTexture(GL_TEXTURE_2D, textureID[6]);
	glUseProgram (shaderProgramIDs[0]);
	glBindVertexArray(vaos[23]);
//Declare your uniform variables that will be used in your shader
	local1 = identity_mat4 ();
	local1 = scale(local1, vec3(100,100,100)); 
	local1 = translate (local1, vec3 (0, -20, translateZ  ));

	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
	glDrawArrays(GL_TRIANGLES,0,counts[23]);

//##############################################################################################


//DRAW TANK

        theWorld.bodies[0]->getMotionState()->getWorldTransform(trans);
	theWorld.bodies[0]->setLinearVelocity(btVector3(2,1,0));
	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (trans.getOrigin().getX(),trans.getOrigin().getY(), trans.getOrigin().getZ()));
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
	glDrawArrays(GL_TRIANGLES,0,counts[i]);	
}



//################################################################################
//Eurofighter

        theWorld.bodies[3]->getMotionState()->getWorldTransform(trans);
glUseProgram (shaderProgramIDs[0]);


	local1 = identity_mat4 ();
	// update uniforms & draw
	local1 = scale (local1, vec3 (0.1, 0.1, 0.1  ));
	local1 = rotate_y_deg (local1, 180); 
	local1 = rotate_x_deg (local1, 15); 
	local1 = translate (local1, vec3 (trans.getOrigin().getX(),trans.getOrigin().getY(), trans.getOrigin().getZ()));
	local1 = translate (local1, vec3 (0, 0, -17  ));
for (int i = 0; i < 4; i++){
	glBindVertexArray(vaos[30 + i]);

	model = model1*local1;


	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[30 + i]);

}


//##############################################################################################
// Missile
	//glBindTexture(GL_TEXTURE_2D, textureID[6]);


        theWorld.bodies[1]->getMotionState()->getWorldTransform(trans);
	if (missilemove == 1){
		theWorld.bodies[1]->setLinearVelocity(btVector3(-trans.getOrigin().getX(),-trans.getOrigin().getY(),-trans.getOrigin().getZ()));
	cout << "X: " << trans.getOrigin().getX()<< "Y: " <<trans.getOrigin().getY() <<"Y: " << trans.getOrigin().getZ() << endl;
	}

	glUseProgram (shaderProgramIDs[0]);
	glBindVertexArray(vaos[29]);
//Declare your uniform variables that will be used in your shader
	local1 = identity_mat4 ();
	local1 = scale(local1, vec3(0.05,0.05,0.05)); 
	local1 = translate (local1, vec3 (trans.getOrigin().getX(),trans.getOrigin().getY(), trans.getOrigin().getZ()));

	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);
	glDrawArrays(GL_TRIANGLES,0,counts[29]);


//################################################################################
//Yak
/*
glUseProgram (shaderProgramIDs[0]);


for (int i = 0 ; i < 5; i++){
	glBindVertexArray(vaos[24 + i ]);
	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (0, 0, 0));

//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 

//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ
	// update uniforms & draw
	model = model1*local1;

	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[24 + i]);
}

//################################################################################
//Buildings
/*
glUseProgram (shaderProgramIDs[0]);


for (int i = 0 ; i < 5; i++){
	glBindVertexArray(vaos[24 + i ]);
	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (0, 0, 0));

//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 

//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ
	// update uniforms & draw
	model = model1*local1;


	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[24 + i]);
}
*/
//###################################################################################
	
// explosion
	glDisable(GL_DEPTH_TEST);					// Disable Depth Testing
	glEnable(GL_BLEND);						// Enable Blending
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);				// Type Of Blending To Perform
	//glEnable (GL_DEPTH_TEST); // enable depth-testing
	//glClear ( GL_DEPTH_BUFFER_BIT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	local1 = identity_mat4 ();
	local1 = scale(local1, vec3(100,100,1)); 
	local1 = translate (local1, vec3 (0, 30, 35));
	glUseProgram (shaderProgramIDs[2]);
//cout << "explosions: " << explosions<< endl;
for(int i = 0; i < explosions-1 ; i++){
//cout << "within: " <<i <<  endl;
explosionList =explosionsarray[i];
//cout << "within: " <<i <<  endl;
explosionList->DrawExplosion();
//cout << "within: " <<i <<  endl;
//explosionList->DrawExplosion();
if (explosionList->life > 0){

	glBindVertexArray(vaos[22]);/*
	if (explosionList->life >175){
		glBindTexture(GL_TEXTURE_2D, textureID[2]);
	cout << "bound" <<endl;}
	else if (explosionList->life >50)
		glBindTexture(GL_TEXTURE_2D, textureID[3]);
	else if (explosionList->life >25)
		glBindTexture(GL_TEXTURE_2D, textureID[4]);
	else {	glBindTexture(GL_TEXTURE_2D, textureID[6]);}
*/
glBindTexture(GL_TEXTURE_2D, textureID[6]);
	local1 = translate (local1, vec3 (0.0, 0, 0));
//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 


	// update uniforms & draw
	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[22]);
}
}


//##############################################################################################
//Score Board, must be drawn last if blending 
char h[]  = "Lives: ";
DrawString(h,7,0,0);

	glBindTexture(GL_TEXTURE_2D, textureID[5]);

glUseProgram (shaderProgramIDs[1]);
	glBindVertexArray(vaos[21]);
	

	glDrawArrays(GL_TRIANGLES,0,counts[21]);
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
float here;

void display1(){
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


if(rotate_z >0){
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
	glDrawArrays(GL_TRIANGLES,0,counts[i]);	
	}
   
}else{

//################################################################################
//Eurofighter

	mat4 local1 = identity_mat4 ();
	local1 = rotate_y_deg(local1,here/0.5);
	local1 = scale(local1, vec3 (0.2, 0.2, 0.2  ));
	local1 = translate(local1, vec3 (-3.0, 0.0, 0.0  ));
glUseProgram (shaderProgramIDs[0]);

for (int i = 0; i < 4; i++){
	glBindVertexArray(vaos[30 + i]);

	// update uniforms & draw
	model = model1*local1;


	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,counts[30 + i]);

	}

}
//##############################################################################################
//Writing, must be drawn last if blending 
char str[14] = "Select Vehicle";
DrawString(str,14,200,550 );




	glBindTexture(GL_TEXTURE_2D, textureID[5]);

glUseProgram (shaderProgramIDs[1]);
	glBindVertexArray(vaos[21]);
	

	glDrawArrays(GL_TRIANGLES,0,counts[21]);
	glBindVertexArray(0);

if(here > 1000)
	glutDisplayFunc(display);

 glutSwapBuffers();
}




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
	 shaderProgramIDs[1] = CompileShaders(pFS1,pVS1);	
}


void init2()
{
	int x ,y,n;


	unsigned char *data = stbi_load("stridsvagn103/Body_d4.jpg", &x, &y, &n, 4);
	// Create 3 vertices that make up a triangle that fits on the viewport 
	


	glGenTextures(1, &textureID[0]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	

//###########################################################################################
	data = stbi_load("stridsvagn103/Details_d.jpg", &x, &y, &n, 4);
	glGenTextures(1, &textureID[1]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	

//###########################################################################################
	data = stbi_load("explosion.png", &x, &y, &n, 4);
	glGenTextures(1, &textureID[2]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

//###########################################################################################
	data = stbi_load("darksmoke.jpg", &x, &y, &n, 4);
	glGenTextures(1, &textureID[3]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	


//###########################################################################################
	data = stbi_load("lightsmoke.jpeg", &x, &y, &n, 4);
	glGenTextures(1, &textureID[4]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[4]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
	


//###########################################################################################
	data = stbi_load("maxresdefault.jpg", &x, &y, &n, 4);
	glGenTextures(1, &textureID[6]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[6]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
	

//###########################################################################################
	data = stbi_load("earth.png", &x, &y, &n, 4);
	glGenTextures(1, &textureID[7]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[7]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	

//###########################################################################################
	data = stbi_load("alpha_explosion.png", &x, &y, &n, 4);
	glGenTextures(1, &textureID[8]);
 	
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, x, y, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
 	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	




        counts[1] = loadMesh("plane.obj",&vaos[1],&vbos[1],0); //the & is vital
	cout << "count: " << counts[1] << endl ;
        counts[2] = loadMesh("sphere.obj",&vaos[2],&vbos[2],0); //the & is vital
	cout << "count: " << counts[2] << endl ;
        counts[3] = loadMesh("Missile.obj",&vaos[3],&vbos[3],0); //the & is vital
	cout << "count: " << counts[3] << endl ;
        counts[23] = loadMesh("skybox.obj",&vaos[23],&vbos[23],0); //the & is vital
	cout << "count: " << counts[23] << endl ;

	for (int i = 0; i < 17; i++)
		{
		if (i != 15)
			{
			counts[4 + i] = loadMesh("stridsvagn103.obj",&vaos[4+i],&vbos[4 + i],i); //the & is vital
			cout << "count: " << counts[4 + i] << endl ;
			}		
		}



        counts[23] = loadMesh("skybox.obj",&vaos[23],&vbos[23],0); //the & is vital
	cout << "count: " << counts[23] << endl ;
       counts[24] = loadMesh("building1.obj",&vaos[24],&vbos[24],0); //the & is vital
	cout << "count: " << counts[24] << endl ;
       counts[25] = loadMesh("building2.obj",&vaos[25],&vbos[25],0); //the & is vital
	cout << "count: " << counts[25] << endl ;
        counts[26] = loadMesh("building3.obj",&vaos[26],&vbos[26],0); //the & is vital
	cout << "count: " << counts[26] << endl ;
        counts[27] = loadMesh("building4.obj",&vaos[27],&vbos[27],0); //the & is vital
	cout << "count: " << counts[27] << endl ;
        counts[28] = loadMesh("building5.obj",&vaos[28],&vbos[28],0); //the & is vital
	cout << "count: " << counts[28] << endl ;
        counts[29] = loadMesh("Missile.obj",&vaos[29],&vbos[29],0); //the & is vital
	cout << "count: " << counts[29] << endl ;

	for (int i = 0; i < 4; i++)
		{
		counts[30 + i] = loadMesh("Models/eurofighter obj/eurofighter obj.obj",&vaos[30+i],&vbos[30 + i],i); //the & is vital
		cout << "count: " << counts[30 + i] << endl ;		
		}





// Set up the shaders*/
	 shaderProgramIDs[0] = CompileShaders(pFS,pVS);

	 shaderProgramIDs[2] = CompileShaders(pFS2,pVS2);
	cout << shaderProgramIDs[0] << endl << shaderProgramIDs[1]  << endl; 



}

int main(int argc, char** argv){
//	glewExperimental = GL_TRUE; 

//	GLuint vao,vbo,count;
//        count = loadMesh("mon.obj",&vao,&vbo); //the & is vital
//	cout << "count: " << count << endl;

	// Set up the window
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB);
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("Hello Triangle");
	// Tell glut where the display function is
	glutDisplayFunc(display2);
	glutIdleFunc(updateScene);
    glutKeyboardFunc(keypress);

	 // A call to glewInit() must be done after glut is initialized!
glewExperimental = GL_TRUE;
    GLenum res = glewInit();

//theWorld  = new world;
	// Check for any errors
    if (res != GLEW_OK) {
      fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
      return 1;
    }
physicsinit();
init();
	// Begin infinite event loop
//while(1)
{
//	glutMainLoopEvent();

}
	glutMainLoop();
    return 0;
}











