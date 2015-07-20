#include <iostream>
#include "maths_funcs.h"
#include <math.h>
#include <GL/glew.h>
#include <GL/freeglut.h>



using namespace std;






void display(){

	glClear(GL_COLOR_BUFFER_BIT);
	// NB: Make the call to draw the geometry in the currently activated vertex buffer. This is where the GPU starts to work!
	glBindVertexArray(vao3);
	glEnable (GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"
	glClearColor (0.5f, 0.5f, 0.5f, 1.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram (shaderProgramID);

//Declare your uniform variables that will be used in your shader
	int matrix_location = glGetUniformLocation (shaderProgramID, "model");
	int view_mat_location = glGetUniformLocation (shaderProgramID, "view");
	int proj_mat_location = glGetUniformLocation (shaderProgramID, "proj");
	
	rotate_x  =+ rotates_x;
 	int width = 5;
	int height = 5;	// Root of the Hierarchy
	mat4 local1 = identity_mat4 ();
	//local1 = scale(local1, vec3(100,100,100)); 
	local1 = translate (local1, vec3 (0, -2, translateZ));

	mat4 view = identity_mat4 ();
	mat4 persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);
	mat4 model = identity_mat4 ();
	mat4 model1 = identity_mat4 ();
	model1 = translate (model1, vec3 (translationX, 0, translationZ));
	model1 = rotate_y_deg (model1, rotate_y); 
	model1 = rotate_z_deg (model1, rotate_z); 
	model1 = rotate_x_deg (model1, rotate_x); 
//	model1 = translate (model1, vec3 (0, 0, translationZ));
//	model1 = scale(model1, vec3(100,100,100)); 
//	view = translate (view, vec3 (0, -2, translateZ));
	view = translate (view, vec3 (translateX, translateY, translateZ));
	// update uniforms & draw
	model = model1*local1;
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);
	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);
	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count3);

//##############################################################################################
	
	glBindVertexArray(vao4);
	view = identity_mat4 ();
	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);

	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (0, 0, 0));

//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 
//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ));

	// update uniforms & draw
	model = model1;
	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count4);	
/*
//################################################################################


	glBindVertexArray(vao5);
	view = identity_mat4 ();
	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);
	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (5, 0, 0));

//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 

//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ
	// update uniforms & draw
	model = model1*local1;

	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count5);


//###################################################################################

	glBindVertexArray(vao6);
	view = identity_mat4 ();

	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);

	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (5, 0, 5));


//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 
//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ));


	// update uniforms & draw
	model = model1*local1;
	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count6);



//###################################################################################

	glBindVertexArray(vao7);
	view = identity_mat4 ();

	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);

	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (10, 0, 5));



//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 
//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ));

	// update uniforms & draw
	model = model1*local1;
	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);


	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count7);

//################################################################################################

	glBindVertexArray(vao8);
	view = identity_mat4 ();

	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);

	local1 = identity_mat4 ();
	local1 = translate (local1, vec3 (10, 0, 10));



//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 
//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ));
	// update uniforms & draw
	model = model1*local1;
	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count8);

//###########################################################################################


	glBindVertexArray(vao2);
	view = identity_mat4 ();
	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);

	local1 = identity_mat4 ();
//	local1 = scale(local1, vec3(0.01,0.01,0.01)); 
//	model = translate (model, vec3 (0, -2, translateZ));
//	view = translate (view, vec3 (0, -2, translateZ));

	local1 = translate (local1, vec3 (translaterX, 0, translaterZ));
	// update uniforms & draw
	model = model1*local1;
	view = translate (view, vec3 (translateX, translateY, translateZ));
	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count2);

	rotaterz = rotaterz +1;

	glBindVertexArray(vao3);
	view = identity_mat4 ();
	persp_proj = perspective(45.0, (float)width/(float)height, 0.1, 100.0);
	mat4 local = identity_mat4 ();

//	model = translate (model, vec3 (0, -2, translateZ)); 
	local = rotate_y_deg (local, rotaterz); 
	local = translate (local, vec3 (-2, 7, 2));
//	view = translate (view, vec3 (0, -2, translateZ));

	view = translate (view, vec3 (translateX, translateY, translateZ));
	// update uniforms & draw
	model = model1*local1*local;

	glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, persp_proj.m);

	glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, view.m);

	glUniformMatrix4fv (matrix_location, 1, GL_FALSE, model.m);

	glDrawArrays(GL_TRIANGLES,0,count3);*/
	glBindVertexArray(0);
	//glDrawArrays(GL_TRIANGLES, 0, count1);
    glutSwapBuffers();
}


