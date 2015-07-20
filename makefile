ALL: 
	g++ -o main main.cpp obj_parser.cpp  maths_funcs.cpp DrawString.cpp -lGL -lGLU -lglut -lGLEW -lassimp -fpermissive

both:
	g++ -g -o main main.cpp obj_parser.cpp  maths_funcs.cpp DrawString.cpp  -lGL -lGLU -lglut -lGLEW -lassimp -lBulletDynamics -lLinearMath -lBulletCollision -std=c++0x -fpermissive
	./main 2b

run: 
	./main 

