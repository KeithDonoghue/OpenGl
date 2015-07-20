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
//#include "stb_image.h"




void DrawString(char* words, int lenght1,int x,int y ,GLuint *counts,GLuint *vaos, GLuint *vbos){


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
