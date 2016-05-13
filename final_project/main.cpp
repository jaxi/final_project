//
//  main.cpp
//  final_project
//
//  Created by 何 靖恺 on 22/11/2012.
//  Copyright (c) 2012 何 靖恺. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/glxew.h>
#include <OpenGL/OpenGL.h>
#include <CoreGraphics/CGImage.h>
#include <GLUT/GLUT.h>
#include <time.h>
#include <math.h>
#include <queue>
#include <stack>

using namespace std;

//==================================================

// Algorithm Part

//==================================================


const double PI = 3.141592 ;
const int mazeSize = 51 ;
bool maze[mazeSize][mazeSize] ;

int dir[4][2] = { // four directions in all
    {0,1} ,
    {0,-1} ,
    {1,0} ,
    {-1,0}
} ;


//==============
//Generate Maze
//==============

void initMaze(){  // set all false except for the start point
    
    for (int i = 0; i < 50; ++ i) {
        for (int j = 0 ; j < 50 ; ++ j){
            maze[i][j] = false ;
        }
    }
    maze[0][0] = true ;
}

int perm[24][4] ;
void initSearchPerm(){  //generate walk directions permutation by brute force
    
    int cnt = 0 ;
    for (int i = 0; i < 4; ++ i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0 ; k < 4; ++k) {
                for (int l = 0; l < 4; ++l) {
                    if(i != j && i!= k && i != l && j != k && j != l && k != l){
                        
                        perm[cnt][0] = i ;
                        perm[cnt][1] = j ;
                        perm[cnt][2] = k ;
                        perm[cnt][3] = l ;
                        ++ cnt ;
                    }
                }
            }
        }
    }
}

void debugperm(){ 
    for(int i = 0 ; i < 24 ; ++ i){
        printf("%d %d %d %d\n" , perm[i][0] , perm[i][1] , perm[i][2] , perm[i][3]) ;
    }
}

bool canWalk(int x , int y){ //test whether a position is visiable.
    
    if ( maze[x][y] == true){
        return false ;
    }
    int walktimes = 0 ;
    int requiretimes = 3 ;
    for ( int i = 0 ; i < 4 ; ++ i){
        int tmpx = x + dir[i][0];
        int tmpy = y + dir[i][1] ;
        if (tmpx >= 0 && tmpx < mazeSize && tmpy >=0 && tmpy < mazeSize ){
            if( maze[tmpx][tmpy] == false ){
                ++ walktimes ;
            }
        }else{
            -- requiretimes ;
        }
    }
    if (walktimes == requiretimes ){
        maze[x][y] = true ;
        return true ;
    }else{
        return false ;
    }
}

void debugcanwalk(int x , int y){
    printf("%d\n" , canWalk(x , y)) ;
}

void DFS(int x , int y){  //Deep first algorithm, to generate the random maze
    
    int order = rand() % 24 ;
    for (int i = 0; i < 4; ++ i) {
        int tmpx = x + dir[perm[order][i]][0] ;
        int tmpy = y + dir[perm[order][i]][1] ;
        if (tmpx >= 0 && tmpx < mazeSize && tmpy >= 0 && tmpy < mazeSize){
            if (canWalk(tmpx, tmpy)) {
                DFS(tmpx, tmpy);
            }
        }
    }
}

void debugDFS(){
    for (int i = 0; i < mazeSize ; ++ i) {
        for (int j = 0; j < mazeSize; ++j) {
            printf("%d" , maze[i][j]) ;
        }
        printf("\n") ;
    }
}

void makeMaze(){ // Totally initial + generate the maze
    
    initSearchPerm() ;
    initMaze() ;
    DFS(0, 0) ;
}


//==============
// Generate Route of Robot
//==============


struct Position {
    int x , y ;
};

bool robotVisited[mazeSize][mazeSize] ;
int previous[mazeSize][mazeSize][2] ;

queue<Position> routeQueue ; //STL Queue, for threads safe

void routeinit(){
    
    for (int i = 0; i < mazeSize; ++ i) {
        for (int j = 0; j < mazeSize; ++ j) {
            robotVisited[i][j] = false ;
            previous[i][j][0] = -1 ;
            previous[i][j][0] = -1 ;
        }
    }
}

void routeBFS(){ // get the robot's route by breadth first search
    
    routeinit() ;
    
    Position current ;
    current.x = 0 ;
    current.y = 0 ;
    
    
    routeQueue.push(current) ;
    
    while (routeQueue.empty() == false) {

        current = routeQueue.front() ;
        robotVisited[current.x][current.y] = true ;
        
        routeQueue.pop() ;
        
        for (int i =0 ; i < 4; ++ i) {
            
            int tmpx = current.x + dir[i][0] ;
            int tmpy = current.y + dir[i][1] ;
            
            if (tmpx >= 0 && tmpx < mazeSize && tmpy >= 0 && tmpy < mazeSize) {
                if (robotVisited[tmpx][tmpy] == false  && maze[tmpx][tmpy] == true ) {
                    Position tmp ;
                    tmp.x = tmpx ;
                    tmp.y = tmpy ;
                    
                    previous[tmp.x][tmp.y][0] = current.x ;
                    previous[tmp.x][tmp.y][1] = current.y ;
                    
                    routeQueue.push(tmp) ;
                }
            }
        }
    }
}

int routes[mazeSize * mazeSize][2] ;
int routesLength = 0 ;

void getRobotRoutes() {

    routeBFS() ;
    int distx , disty ;
    for (int i = mazeSize - 1; i >= 0; -- i) {
        if (maze[i][mazeSize - 1] == true) {
            distx = i ;
            disty = mazeSize - 1 ;
            break ;
        }
    }
    int currentx = distx ;
    int currenty = disty ;
    routesLength = 0 ;
    
    while (currentx != -1 && currenty != -1) {
        
        routes[routesLength][0] = currentx ;
        routes[routesLength][1] = currenty ;
        routesLength ++ ;

        int tmpx  = previous[currentx][currenty][0] ;
        int tmpy  = previous[currentx][currenty][1] ;
        currentx = tmpx ;
        currenty = tmpy ;
    }
    
    reverse(routes, routes + routesLength) ;
}

void routesDebug() {
    
    getRobotRoutes() ;
    for (int i = 0; i < routesLength; ++ i) {
        cout << routes[i][0] << "  " << routes[i][1] << endl ;
    }
    cout << routesLength << endl ;

}



//==================================================

// Graphic Part

//==================================================


GLfloat sceneSize = 510;
GLfloat mapsize = 51 ;
int anglealpha = 0 ;
int anglebeta = 30 ;
GLfloat anglelength = 50 ;
GLfloat anglelengthmax = 400 ;
GLfloat anglelengthmin = 0 ;
GLuint treeID ;


GLfloat no_mat[] = { 0.0 , 0.0 , 0.0 , 1.0 } ;
GLfloat mat_ambient [] = { 0.7 , 0.7 , 0.7 , 1.0 } ;
GLfloat mat_ambient_color [] = { 0.3 , 0.3 , 0.3 , 1.0 } ;
GLfloat mat_diffuse [] = { 0.1 , 0.5 , 0.8 , 1.0 } ;
GLfloat mat_specular [] = { 1.0 , 1.0 , 1.0 , 1.0 } ;
GLfloat no_shininess [] = { 0.0 } ;
GLfloat low_shininess [] = {5.0 } ;
GLfloat high_shininess [] = { 100.0 } ;
GLfloat mat_emission [] = { 0.3 , 0.2 , 0.2 , 0.0 } ;

void generate(GLfloat *ambient , GLfloat *specular ,
              GLfloat *shininess , GLfloat * emission){ // material settings for each object
    
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient) ;
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse) ;
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular) ;
    glMaterialfv(GL_FRONT, GL_SHININESS, shininess) ;
    glMaterialfv(GL_FRONT, GL_EMISSION, emission) ;
}

//================
// Shader Part
//================

GLuint shader , program ;
GLint compiled , linked ;

// the two print program below is totally copied from the Example in lecture notes.

/* Print shader info log to screen. Use after a compile to see any
 compilation errors */
void printShaderInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;
    
	/* Get shader info log */
	glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);
    
    if (infologLength > 0)
    {
        infoLog = (char *)malloc(infologLength);
        glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n",infoLog);
        free(infoLog);
    }
}

/* Print progam info log. Call after linking to check for link errors */
void printProgramInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;
    
	glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);
    
    if (infologLength > 0)
    {
        infoLog = (char *)malloc(infologLength);
        glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n",infoLog);
        free(infoLog);
    }
}

/* 
    CAUTIOUS !!!!
    the shader code below got idea from the site http://www.ozone3d.net/tutorials/glsl_fog/
 */

char vertexShaderSrc[] = { // program for vertex shader
    
    "varying float fogFactor; "
    
    
    "void main(void)"
    "{"
    "gl_Position = ftransform();"
    "gl_TexCoord[0] = gl_MultiTexCoord0;"
    
    "const float LOG2 = 1.442695;" // macro like variable
    "float fogDensity = 0.001;"
    
    "gl_FogFragCoord = abs(gl_Position.z);"
    
    "fogFactor = exp2( -fogDensity * fogDensity * gl_FogFragCoord * gl_FogFragCoord * LOG2 ); " //set the factor of fog
    "fogFactor = clamp(fogFactor, 0.0, 1.0);  "   
    
    "}"
} ;

char fragmentShaderSrc[] = { // program for fragment shader
    
    "uniform sampler2D Texture0;"
    
    "varying float fogFactor;"
    "void main (void)"
    "{"
    "    vec4 fogColor    = vec4(0.0,0.4,0.6,1.0);"
    "    vec4 finalColor = texture2D(Texture0, gl_TexCoord[0].xy);"
    "    gl_FragColor     = mix(fogColor, finalColor, fogFactor );"
    "}"
    
};

GLuint v , f , p ;

// this program is modified from the program in the lecture note's example

void setShaders() {
    
    glewExperimental = GL_TRUE ;
    GLenum err =glewInit() ;
    if (err != GLEW_OK) {
        cout << "glewInit failed, aborting." << endl ;
    }
    
    v = glCreateShader(GL_VERTEX_SHADER) ;
    f = glCreateShader(GL_FRAGMENT_SHADER) ;
    
    const char *vs = vertexShaderSrc ;
    const char *fs = fragmentShaderSrc ;
    
    glShaderSource(v , 1 , &vs , NULL) ;
    glShaderSource(f , 1 , &fs , NULL) ;
    
    glCompileShader(v) ;
    glCompileShader(f) ;
    
    printShaderInfoLog(v) ;
    printShaderInfoLog(f) ;
    
    p = glCreateProgram() ;
    
    glAttachShader(p , v) ;
    glAttachShader(p , f) ;
    
    
    glLinkProgram(p) ;

    printProgramInfoLog(p) ;
    
    glUseProgram(p) ;
}


//================
// Cylinder Part
//================

void drawCycle(GLfloat radius , int slice ){
    GLfloat vertex[4] ;
    GLfloat normal[4] ;
    GLfloat delta_angle = 2 * PI / slice ;
    
    normal[0] = 0.0 ;
    normal[1] = 1.0 ;
    normal[2] = 0.0 ;
    normal[3] = 1.0 ;
    
    glBegin(GL_TRIANGLE_FAN) ;
    vertex[0] = 0.0 ;
    vertex[1] = 0.0 ;
    vertex[2] = 0.0 ;
    vertex[3] = 1.0 ;
    
    //glNormal3fv(normal) ;
    glVertex3fv(vertex) ;
    
    for (int i = 0; i < slice ; ++ i) {
        vertex[0] = radius * cos(delta_angle * i) ;
        vertex[1] = 0.0 ;
        vertex[2] = radius * sin(delta_angle * i) ;
        vertex[3] = 1.0 ;
        glVertex3fv(vertex) ;
    }
    
    vertex[0]= radius ;
    vertex[1] = 0.0 ;
    vertex[2] = 0.0 ;
    vertex[3] = 1.0 ;
    glVertex3fv(vertex) ;
    glEnd() ;
}

void drawCylinder(GLfloat base , GLfloat top , GLfloat height , int slice){
    glPushMatrix() ;
    
    glPushMatrix() ;
    glRotatef(-90, 1, 0, 0) ;
    gluCylinder(gluNewQuadric(), base, top, height, 30, 30) ;
    glPopMatrix() ;
    
    glPushMatrix() ;
    drawCycle(base, 30) ;
    glPopMatrix() ;
    
    glPushMatrix() ;
    glTranslatef(0.0, height, 0.0) ;
    drawCycle(top, 30) ;
    glPopMatrix() ;
    
    glPopMatrix() ;
}

//================
// cube part
//================

GLfloat vertex[8][4] = {
    { -0.5 , -0.5 , 0.5 , 1 } ,
    { 0.5 , -0.5 , 0.5 , 1 } ,
    { 0.5 , -0.5 , -0.5 , 1 } ,
    { -0.5 , -0.5 , -0.5 , 1 } ,
    { -0.5 , 0.5 , 0.5 , 1 } ,
    { 0.5 , 0.5 , 0.5 , 1 } ,
    {0.5 , 0.5 , -0.5 , 1 } ,
    { -0.5 , 0.5 , -0.5 }
} ;

GLfloat normaldir[6][4] = {
    { 0 , -1 ,0 , 1 } ,
    { 0 , 1, 0 , 1 } ,
    { 0 , 0 , 1 , 1 } ,
    { 1 , 0 , 0 , 1 } ,
    { 0 , 0 , -1 , 1 } ,
    { -1 , 0 , 0 , 1 }
} ;

int pnts[6][4] = {
    { 0 , 1 , 2 , 3 } ,
    { 4,  5 , 6 , 7 } ,
    { 0 , 1 , 5 , 4 } ,
    { 1 , 2 , 6 , 5 } ,
    { 3 , 2 , 6 , 7 } ,
    { 0 , 3 , 7 , 4 }
} ;


void drawCube(GLfloat size) {
    glPushMatrix() ;
    for (int i = 0 ; i < 8; ++ i) {
        for (int j = 0 ; j < 4; ++ j) {
            vertex[i][j] *= size ;
        }
    }
    
    for (int i = 0; i < 6; ++ i) {
        glBegin(GL_POLYGON) ;
        glNormal3fv(normaldir[i]) ;
        for (int j = 0; j < 4; ++ j) {
            glVertex3fv(vertex[pnts[i][j]]) ;
        }
        glEnd() ;
    }
    
    for (int i = 0 ; i < 8; ++ i) {
        for (int j = 0 ; j < 4; ++ j) {
            vertex[i][j] /= size ;
        }
    }
    glPopMatrix() ;
}

//================
// Tree Part
//================

void drawBranch(GLfloat length) {
    drawCylinder(length / 40, length / 320, length, 30) ;
}

GLfloat greenTree[4] = { 0.45 , 0.25 , 0.0 , 1.0 } ;

void drawTree(GLfloat treeLength , int depth) { //by recursive
    glPushMatrix() ;

    drawBranch(treeLength) ;
    if (depth != 0) {
        glPushMatrix() ;
        glTranslatef(0.0, treeLength * 3 / 8, 0.0) ;
        glRotatef(30 + rand() % 30, 0.0, 0.0, 1.0) ;
        drawTree(treeLength / 2, depth - 1) ;
        glPopMatrix() ;
        
        glPushMatrix() ;
        glTranslatef(0.0, treeLength * 7 / 16, 0.0) ;
        glRotatef(100 + rand() % 20, 0.0, 1.0, 0.0) ;
        glRotatef(30 + rand() % 30, 0.0, 0.0, 1.0) ;
        drawTree(treeLength / 2, depth - 1) ;
        glPopMatrix() ;
        
        glPushMatrix() ;
        glTranslatef(0.0, treeLength / 2, 0.0) ;
        glRotatef(200 + rand() % 40, 0.0, 1.0, 0.0) ;
        glRotatef(30 + rand() % 30, 0.0, 0.0, 1.0) ;
        drawTree(treeLength / 2, depth - 1) ;
        glPopMatrix() ;
    }
    glPopMatrix() ;
}



GLuint createTree() { //append the tree to display lists
    GLuint treeDL ;
    
    treeDL = glGenLists(1) ;
    glNewList(treeDL, GL_COMPILE_AND_EXECUTE) ;
    generate(greenTree, mat_ambient_color, no_shininess, mat_emission) ;
    drawTree(mapsize / mazeSize  , 2) ;

    glEndList() ;
    
    return (treeDL) ;
}

//================
// Grass Part
//================

const int grassWidth = 64 ;
const int grassHeight = 64 ;
GLubyte grassImage[grassWidth][grassHeight][4] ;
bool grassJudge[grassWidth][grassHeight] ;

GLuint grassTexName ;

void grassImageInit(){ // inital the pixels of grass image
    
    for (int i = 0; i < grassWidth; ++ i) {
        for (int j = 0; j< grassHeight; ++ j) {
            grassJudge[i][j] = false ;
        }
    }
    for (int i = 0; i < grassHeight; ++ i) {
        grassJudge[i][grassWidth/ 2] = true ;
        
        if( i % 3 == 1){
            for (int j = 0; i - j >= 0 && grassWidth / 2 + j < grassHeight && grassWidth / 2 - j >= 0; ++ j) {
                grassJudge[i - j][grassWidth / 2 + j] = true ;
                grassJudge[i - j][grassWidth / 2 - j] = true ;
            }
        }
    }
}

void drawGrassImage() { // convert the grass image to pixels
    
    grassImageInit() ;
    
    for (int i = 0; i < grassWidth; ++ i) {
        for (int j = 0; j < grassHeight; ++ j) {
            if (grassJudge[i][j] == true) {
                
                grassImage[i][j][0] = 66 ;
                grassImage[i][j][1] = 204 ;
                grassImage[i][j][2] = 33 ;
                grassImage[i][j][3] = 255 ;
                
            }else{
                
                grassImage[i][j][0] = 196 ;
                grassImage[i][j][1] = 230 ;
                grassImage[i][j][2] = 240 ;
                grassImage[i][j][3] = 255 ;
            }
        }
    }
}

void grassInit(){ // texture init
    drawGrassImage() ;
 
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1) ;
    glGenTextures(2, &grassTexName) ;
    glBindTexture(GL_TEXTURE_2D, grassTexName) ;
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) ;
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, grassHeight, grassWidth, 0, GL_RGBA, GL_UNSIGNED_BYTE, grassImage) ;
    
}


void grassImageDebug() {
    grassImageInit() ;
    for (int i = 0; i < grassWidth; ++ i) {
        for (int j = 0; j < grassHeight; ++ j) {
            cout << grassJudge[i][j];
        }
        cout << endl ;
    }
}

GLuint grassID ;

GLuint createGrass() { // draw a piece of grass and map texture on it , append it to the display lists
    GLuint GrassDL ;
    
    GrassDL = glGenLists(3) ;
    GLfloat length = mapsize / mazeSize / 4;
    
    glNewList(GrassDL, GL_COMPILE_AND_EXECUTE) ;
    
    glPushMatrix() ;
    glEnable(GL_TEXTURE_2D) ;
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
    glBindTexture(GL_TEXTURE_2D, grassTexName) ;
    
    for (int i = 0; i < 3; ++ i) {
        glPushMatrix() ;
        glRotatef(120 * i, 0.0, 1.0, 0.0) ;
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(-0.1, 0, 0) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f( 0.1, 0, 0) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f( 0.1, length, 0.0) ;
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(-0.1, length, 0) ;
        
        glEnd() ;
        glPopMatrix() ;
    }
    glDisable(GL_TEXTURE_2D) ;
    glPopMatrix() ;
    
    glEndList() ;
    
    return (GrassDL) ;
}

GLuint grassAreaID ;

GLuint creatGrassArea () { // make grass cluster like , append it to the display list, for performance
    
    GLuint GrassAreaDL ;
    GrassAreaDL = glGenLists(4) ;
    
    GLfloat length = mapsize / mazeSize ;
    GLfloat step = length / 6 ;
    glNewList(GrassAreaDL, GL_COMPILE_AND_EXECUTE) ;
    
    for (GLfloat i = 0; i <= length ; i += step) {
        for (GLfloat j = 0; j <= length; j += step) {
            glPushMatrix() ;
            glTranslatef(i - 0.5 * length, 0.0, j - 0.5 * length) ;
            glRotatef(rand() % 360, 0.0, 1.0, 0.0) ;
            glCallList(grassID) ;
            glPopMatrix() ;
        }
    }
    glEndList() ;
    
    return (GrassAreaDL) ;
}

void grassDebug(){
    grassImageDebug() ;
    gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0) ;
    glPushMatrix() ;
    //glScalef(1 / 2, 1 / 2, 1 / 2) ;
    glCallList(grassID) ;
    glPopMatrix() ;
}

//================
// Robot Part
//================

GLfloat building[4] = { 0.1 , 0.1 , 0.1 , 1 } ;

void drawEye(GLfloat eyeSize , GLfloat eyeLength, GLfloat offset) { // robot's eye
    glPushMatrix() ;
    glTranslatef(offset, 0.0, 0.0) ;
    glRotatef(90, 1.0, 0.0, 0.0) ;
    glTranslatef(0.0, - eyeLength / 2, 0.0) ;
    drawCylinder(eyeSize, eyeSize, eyeLength, 30 ) ;
    glPopMatrix() ;
}

void drawHead(GLfloat headSize , GLfloat neckLength){ //robot's head
    
    glPushMatrix() ;
    
    glPushMatrix() ;
    glTranslatef(0.0, neckLength, 0.0) ;
    drawEye( headSize / 7, headSize * 1.1, -headSize / 2) ;
    drawEye( headSize / 7, headSize * 1.1, headSize / 2) ;
    glScalef(1.0, 0.125, 1.0) ;
    drawCube(headSize) ;
    glPopMatrix() ;
    
    drawCylinder(headSize / 2, headSize / 4, neckLength, 30 ) ;
    glPopMatrix() ;
    
}


void drawLegSide(GLfloat wheelSize , GLfloat wheelLength ,  int wheelsPerSide , GLfloat bodySize) { // OK , it's the wheel
    for (int i = 0 ; i < wheelsPerSide; ++ i) {
        glPushMatrix() ;
        glTranslatef(0.0, 0.0, - wheelSize * (wheelsPerSide -1)/ 2 + wheelSize * i) ;
        glTranslatef(0.0, bodySize * 0.03, 0.0) ;
        glRotatef(90, 0.0, 1.0, 0.0) ;
        glRotatef(90, 1.0, 0.0, 0.0) ;
        glTranslatef(0.0, - wheelLength / 2, 0.0) ;
        drawCylinder(wheelSize, wheelSize * 1.2, wheelLength, 30) ;
        glPopMatrix() ;
    }
}


void drawBody(GLfloat bodySize){
    glPushMatrix() ;
    
    glPushMatrix() ;
    glTranslatef(0.0, -bodySize / 3, 0.0) ;
    
    glPushMatrix() ;
    glRotatef(90, 0.0, 0.0, 1.0) ;
    glTranslatef(0.0, - bodySize * 1.5 / 2, 0.0) ;
    drawCylinder(sqrt(3.0) / 6 * bodySize / 15 * 5 , sqrt(3.0) / 6  * bodySize / 15 * 5 , bodySize * 1.5, 30) ;
    glPopMatrix() ;
    
    for (int i = 0; i < 3; ++ i) {
        glPushMatrix() ;
        glRotatef(i * 120, 1.0, 0.0, 0.0) ;
        glTranslatef(0.0, - (bodySize / 15 * 10 * sqrt(3) / 6 + bodySize / 30) , 0.0) ;
        drawLegSide(bodySize / 15, bodySize * 1.5, 10 , bodySize) ;
        glPopMatrix() ;
    }
    glPopMatrix() ;
    
    drawHead(bodySize / 2, bodySize * 0.8) ;
    drawCube(bodySize) ;
    
    glPushMatrix() ;
    glTranslatef(0.0, bodySize * 0.3, -bodySize * 0.4) ;
    glScalef(0.7, 0.15, 0.4) ;
    drawCube(bodySize) ;
    glPopMatrix() ;
    
    glPopMatrix() ;
    
}




void drawRotbot(){
    generate(building, mat_ambient_color, no_shininess, mat_emission) ;
    drawBody(mapsize / mazeSize * 0.4) ;
}


void RobotDebug(){
    gluLookAt(-3, 3, 3, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0) ;
    drawBody(1) ;
}
//================
// Moving Part
//================

int movingPartPositionX = 0 , movingPartPositionY = 0 ;
GLfloat mpfx = 0.0 , mpfy = 0.0 ;
int headFor = 0 ;
GLfloat lookatatdistance = 3.0 ;


void drawMovingPart(){ // mainly make the robot move
    
    GLfloat length = mapsize / mazeSize ;
    GLfloat x = (mazeSize - 2) / 2 - length * (mpfx - 1) ;
    GLfloat y = length - 0.01 ;
    GLfloat z = (mazeSize - 2) / 2 - length * (mpfy - 1) ;
    
    gluLookAt(x + lookatatdistance , y + lookatatdistance / 2 , z + lookatatdistance,
              x, y, z,
              0.0, 1.0, 0.0) ;
    
    glPushMatrix() ;
    glTranslatef(x, y - 0.3 ,z) ;
    glRotatef(90 * headFor, 0.0, 1.0, 0.0) ;
    drawRotbot() ;
    glPopMatrix() ;
}

//================
// Sky Box Part
//================

const int imageHeight = 64 ;
const int imageWidth = 64 ;
GLubyte cloudImage[imageHeight][imageWidth][4] ;
bool cloudJudge[imageHeight][imageWidth] ;

void drawCloud(){ // generate the cloud texture
    
    int dir[4][2] = {
        {0 , 1} ,
        {1 , 0} ,
        {0 , -1} ,
        {-1 , 0 }
    } ;

    for (int i = 0; i < imageHeight; ++ i) {
        for (int j = 0; j < imageWidth; ++ j) {
            cloudJudge[i][j] = false ;
        }
    }
    
    int cnt = 0 , lopy = imageHeight - 1 , standardcnt = 0 ;
    int posx = 0 , posy = 0 ;
    
    while (lopy > 0) {

        for (int i = 0; i < lopy; ++ i) {
            cloudJudge[posx][posy] = true ;
            posx += dir[cnt][0] ;
            posy += dir[cnt][1] ;
        }

        cnt = (cnt + 1) % 4 ;
        if (standardcnt % 2 == 0 && standardcnt != 0) {
            lopy -= 2 ;
        }
        ++ standardcnt ;
    }
    
    for (int i = 0; i < imageHeight; ++ i) {
        for (int j = 0 ; j < imageWidth;  ++ j) {
            if (cloudJudge[i][j] == false ) {
                
                cloudImage[i][j][0] = 196 ;
                cloudImage[i][j][1] = 230 ;
                cloudImage[i][j][2] = 240 ;
                cloudImage[i][j][3] = 0 ;
                
            }else{
                cloudImage[i][j][0] = 255 ;
                cloudImage[i][j][1] = 255 ;
                cloudImage[i][j][2] = 255 ;
                cloudImage[i][j][3] = 255 ;
            }
        }
    }
}

GLuint skyTexName ;

GLfloat cloudPnts[5][5][2][3] ;

void initCloudplace(GLfloat size){ // random place the cloud.
    
    
    for (int i = 0; i < 5; ++ i) {
        cloudPnts[0][i][0][0] = (GLfloat)(rand() % 100 )/ 100 * size - size / 2 ;
        cloudPnts[0][i][0][1] = (GLfloat)(rand() % 100 )/ 100 * size / 4 ;
        cloudPnts[0][i][0][2] = size / 2 ;
        
        cloudPnts[0][i][1][0] = (GLfloat)(rand() % 100 )/ 100 * size - size / 2 ;
        cloudPnts[0][i][1][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4;
        cloudPnts[0][i][1][2] = size / 2 ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        cloudPnts[1][i][0][0] = size / 2 ;
        cloudPnts[1][i][0][1] = (GLfloat)(rand() % 100 )/ 100 * size / 4 ;
        cloudPnts[1][i][0][2] = (GLfloat)(rand() % 100 )/ 100 * size - size / 2 ;
        
        cloudPnts[1][i][1][0] = size / 2 ;
        cloudPnts[1][i][1][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4 ;
        cloudPnts[1][i][1][2] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        cloudPnts[2][i][0][0] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        cloudPnts[2][i][0][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4 ;
        cloudPnts[2][i][0][2] = - size / 2 ;
        
        cloudPnts[2][i][1][0] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        cloudPnts[2][i][1][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4 ;
        cloudPnts[2][i][1][2] = - size / 2 ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        cloudPnts[3][i][0][0] = - size / 2 ;
        cloudPnts[3][i][0][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4 ;
        cloudPnts[3][i][0][2] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        
        cloudPnts[3][i][1][0] = - size / 2 ;
        cloudPnts[3][i][1][1] = (GLfloat)(rand() % 100 ) / 100 * size / 4 ;
        cloudPnts[3][i][1][2] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        cloudPnts[4][i][0][0] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        cloudPnts[4][i][0][1] = size / 4 ;
        cloudPnts[4][i][0][2] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        
        cloudPnts[4][i][1][0] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
        cloudPnts[4][i][1][1] = size / 4 ;
        cloudPnts[4][i][1][2] = (GLfloat)(rand() % 100 ) / 100 * size - size / 2 ;
    }
}

void SkyInit() { // sky texture settings
    drawCloud() ;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1) ;
    glGenTextures(1, &skyTexName) ;
    glBindTexture(GL_TEXTURE_2D, skyTexName) ;
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) ;
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageHeight, imageWidth, 0, GL_RGBA, GL_UNSIGNED_BYTE, cloudImage) ;
    
    initCloudplace(mapsize * 2) ;
    
}


void drawSkyLine(GLfloat size){ // draw the skyline
    GLfloat vertex[8][4] = {
        {-size / 2, 0 , size / 2 , 1 },
        {size / 2 , 0 , size / 2 , 1},
        {size / 2 , 0 , - size / 2 , 1},
        {-size / 2 , 0 , - size / 2 , 1},
        {-size / 2 , size / 4, size / 2 , 1},
        {size / 2 , size / 4, size / 2 , 1},
        {size / 2 , size / 4, - size / 2 , 1},
        {-size / 2 , size / 4, - size / 2 , 1}
    };
    glPushMatrix() ;
    //generate(no_mat, mat_specular, low_shininess, mat_emission) ;
    glBegin(GL_LINE_LOOP) ;
    glVertex3fv(vertex[0]) ;
    glVertex3fv(vertex[1]) ;
    glVertex3fv(vertex[5]) ;
    glVertex3fv(vertex[4]) ;
    glEnd();
    
    glBegin(GL_LINE_LOOP) ;
    glVertex3fv(vertex[1]) ;
    glVertex3fv(vertex[2]) ;
    glVertex3fv(vertex[6]) ;
    glVertex3fv(vertex[5]) ;
    glEnd();
    
    glBegin(GL_LINE_LOOP) ;
    glVertex3fv(vertex[3]) ;
    glVertex3fv(vertex[2]) ;
    glVertex3fv(vertex[6]) ;
    glVertex3fv(vertex[7]) ;
    glEnd();
    
    glBegin(GL_LINE_LOOP) ;
    glVertex3fv(vertex[0]) ;
    glVertex3fv(vertex[3]) ;
    glVertex3fv(vertex[7]) ;
    glVertex3fv(vertex[4]) ;
    glEnd();
    
    glBegin(GL_LINE_LOOP) ;
    glVertex3fv(vertex[4]) ;
    glVertex3fv(vertex[5]) ;
    glVertex3fv(vertex[6]) ;
    glVertex3fv(vertex[7]) ;
    glEnd();

    glPopMatrix() ;
    
}

void drawCloudOnSky(GLfloat size){ // map the cloud position array to the sky, convert to texture
    

    glPushMatrix() ;
    glEnable(GL_TEXTURE_2D) ;
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
    glBindTexture(GL_TEXTURE_2D, skyTexName) ;
    
    for (int i = 0; i < 5; ++ i) {
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(cloudPnts[0][i][0][0], cloudPnts[0][i][0][1], size / 2) ;
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(cloudPnts[0][i][1][0], cloudPnts[0][i][0][1], size / 2) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f(cloudPnts[0][i][1][0], cloudPnts[0][i][1][1], size / 2) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f(cloudPnts[0][i][0][0], cloudPnts[0][i][1][1], size / 2) ;
        
        glEnd() ;
    }
    
    for (int i = 0; i < 5 ; ++ i) {
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(size / 2, cloudPnts[1][i][0][1], cloudPnts[1][i][0][2]) ;
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(size / 2, cloudPnts[1][i][0][1], cloudPnts[1][i][1][2]) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f(size / 2, cloudPnts[1][i][1][1], cloudPnts[1][i][1][2]) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f(size / 2, cloudPnts[1][i][1][1], cloudPnts[1][i][0][2]) ;
        
        glEnd() ;
    }
    
    for (int i = 0; i < 5; ++i) {
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(cloudPnts[2][i][0][0], cloudPnts[2][i][0][1], -size / 2) ;
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(cloudPnts[2][i][1][0], cloudPnts[2][i][0][1], -size / 2) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f(cloudPnts[2][i][1][0], cloudPnts[2][i][1][1], -size / 2) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f(cloudPnts[2][i][0][0], cloudPnts[2][i][1][1], -size / 2) ;
        
        glEnd() ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(-size / 2, cloudPnts[3][i][0][1], cloudPnts[3][i][0][2]) ;
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(-size / 2, cloudPnts[3][i][0][1], cloudPnts[3][i][1][2]) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f(-size / 2, cloudPnts[3][i][1][1], cloudPnts[3][i][1][2]) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f(-size / 2, cloudPnts[3][i][1][1], cloudPnts[3][i][0][2]) ;
        
        glEnd() ;
    }
    
    for (int i = 0; i < 5; ++ i) {
        glBegin(GL_QUADS) ;
        
        glTexCoord2f(0.0, 0.0) ;
        glVertex3f(cloudPnts[4][i][0][0], size / 4, cloudPnts[4][i][0][2]) ;
        glTexCoord2f(0.0, 1.0) ;
        glVertex3f(cloudPnts[4][i][0][0], size / 4, cloudPnts[4][i][1][2]) ;
        glTexCoord2f(1.0, 1.0) ;
        glVertex3f(cloudPnts[4][i][1][0], size / 4, cloudPnts[4][i][1][2]) ;
        glTexCoord2f(1.0, 0.0) ;
        glVertex3f(cloudPnts[4][i][1][0], size / 4, cloudPnts[4][i][0][2]) ;
        
        glEnd() ;
    }
    
    glDisable(GL_TEXTURE_2D) ;
    glPopMatrix() ;
}

void cloudImageDebug() {
    
    drawCloud() ;
    for (int i = 0; i < imageHeight; ++ i) {
        for (int j = 0; j < imageWidth;  ++ j) {
            cout<<cloudJudge[i][j] ;
        }
        cout<<endl ;
    }
}

//================
// Road Part
//================

GLuint roadTexName ;

const int stoneHeight = 32 ;
const int stoneWidth = 32 ;
bool stoneJudge[stoneHeight][stoneWidth] ;
GLubyte stoneImage[stoneHeight][stoneWidth][4] ;

void initStoneImage(){ // simulating the stone in a 2D-array
    
    for (int i = 0; i < stoneHeight; ++ i) {
        for (int j =0; j < stoneWidth; ++ j) {
            stoneJudge[i][j] = false ;
        }
    }
    
    GLfloat centerx = (GLfloat)stoneHeight / 2 ;
    GLfloat centery = (GLfloat)stoneWidth / 2 ;
    int radius = (int)sqrt(stoneHeight * stoneWidth) / 2 ;

    for (int i =0; i < radius / 2; ++ i) {
        for (int j = 0; j < 360; ++ j) {
            GLfloat x = centerx + i * cos(j * 2 * PI / 360) ;
            GLfloat y = centery + i * sin(j * 2 * PI / 360) ;
            stoneJudge[(int)x][(int)y] = true ;
        }
    }

}

void stoneImageDebug(){
    initStoneImage() ;
    for (int i = 0; i < stoneHeight;  ++ i) {
        for (int j = 0; j < stoneWidth; ++ j) {
            cout<< stoneJudge[i][j] ;
        }
        cout << endl ;
    }
}

void drawStoneImage(){ // mapping the 2D-array to pixels
    initStoneImage() ;
    for (int i = 0; i < stoneHeight; ++ i) {
        for (int j = 0; j < stoneWidth; ++ j) {
            
            if (stoneJudge[i][j] == true) {
                
                stoneImage[i][j][0] = 153 ;
                stoneImage[i][j][1] = 153 ;
                stoneImage[i][j][2] = 153 ;
                stoneImage[i][j][3] = 255 ;
            }else{
                
                stoneImage[i][j][0] = 102 ;
                stoneImage[i][j][1] = 51 ;
                stoneImage[i][j][2] = 0 ;
                stoneImage[i][j][3] = 128 ;
            }
        }
    }
}

void roadInit(){ // init the road texture settinngs
    
    drawStoneImage() ;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1) ;
    glGenTextures(2, &roadTexName) ;
    glBindTexture(GL_TEXTURE_2D, roadTexName) ;
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) ;
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, stoneHeight, stoneWidth, 0, GL_RGBA, GL_UNSIGNED_BYTE, stoneImage) ;

}

GLuint roadID ;

GLuint createRoad() { // append the road to the display lists
    GLuint RoadDL ;
    
    RoadDL = glGenLists(1) ;
    glNewList(RoadDL, GL_COMPILE_AND_EXECUTE) ;
    
    glPushMatrix() ;
    glEnable(GL_TEXTURE_2D) ;
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
    glBindTexture(GL_TEXTURE_2D, roadTexName) ;
    
    
    for (GLfloat i = 0; i < (GLfloat)mapsize - 0.3; i += 0.3) {
        for (GLfloat j = 0; j < (GLfloat)mapsize - 0.3; j += 0.3) {
            
            GLfloat tmpx = i - mapsize / 2;
            GLfloat tmpy = mapsize/ 110 ;
            GLfloat tmpz = j - mapsize / 2 ;
            
            glBegin(GL_QUADS) ;
            
            glTexCoord2f(0.0, 0.0) ;
            glVertex3f(tmpx, tmpy, tmpz) ;
            glTexCoord2f(0.0, 1.0) ;
            glVertex3f(tmpx, tmpy, tmpz + 0.3) ;
            glTexCoord2f(1.0, 1.0) ;
            glVertex3f(tmpx + 0.3, tmpy, tmpz + 0.3) ;
            glTexCoord2f(1.0, 0.0) ;
            glVertex3f(tmpx + 0.3, tmpy, tmpz) ;
            glEnd() ;

        }
    }
    
    glDisable(GL_TEXTURE_2D) ;
    glPopMatrix() ;

    glEndList() ;
    
    return (RoadDL) ;
}


//================
// Wall Part
//================

void drawWall(){
    glPushMatrix() ;
    for (int i = 0; i < 4; ++ i) {
        glPushMatrix() ;
        glRotatef(90 * i, 0.0, 1.0, 0.0) ;
        glTranslatef(0.0 , 0.0, mapsize / 1.99) ;
        glScalef(1.01, 0.025 , 0.005) ;
        drawCube(mapsize) ;
        glPopMatrix() ;
    }
    glPopMatrix() ;
}
//================
// Map Part
//================

GLfloat sea[4] = { 0.156 , 0.588 , 0.784 , 1.0 } ;

void drawMap(){
    GLfloat length = mapsize / mazeSize ;
    glPushMatrix() ;

    drawMovingPart() ;
    drawSkyLine(mazeSize * 2) ;
    drawCloudOnSky(mazeSize * 2) ;
    
    for (int i = 0; i < mazeSize; ++ i) {
        for (int j = 0; j < mazeSize; ++ j) {
            if (maze[i][j] == false) {
                
                glPushMatrix() ;
                glTranslatef(( mazeSize - 2)/ 2 - length * (i - 1), length/ 2  - 0.1 , ( mazeSize - 2)/ 2  - length * ( j - 1)) ;
                
                glPushMatrix() ;
                glScalef(1.0, 1 / (length * 4) , 1.0) ;
                drawCube(length) ; // set the boader
                glPopMatrix() ;
                
                glCallList(treeID) ;
                glCallList(grassAreaID) ;
                glPopMatrix() ;
                
            }
        }
    }
    glCallList(roadID) ; // draw the road
    
    for (int i = 0; i < 4; ++ i) {
        glPushMatrix() ;
        generate(sea, mat_specular, high_shininess, no_mat) ;
        glRotatef(90 * i, 0.0, 1.0, 0.0) ;
        glTranslatef(mapsize * 3 / 4, 0 , mapsize * 1 / 4) ;
        glScalef(mapsize / 2, 1, mapsize * 3 / 2) ;
        drawCube(1.0) ;
        glPopMatrix() ;
    }
    drawWall() ;
    glPopMatrix() ;
    
}

//==============
// Initial Function
//==============

void init (void){ // Lighting settings are mainly from THE RED BOOK aka
    
    GLfloat ambient[] = {0.0 , 0.0 , 0.0 , 1.0 } ;
    GLfloat white_light[] = {0 , 0 , 0 , 1.0 } ;
    GLfloat diffuse[] = {0.4 , 0.4 , 0.4 ,1.0 } ;
    GLfloat lmodel_ambient[] = {0.4 ,0.4 , 0.4 , 1.0 } ;
    GLfloat local_view[] = {0.0} ;
    
    GLfloat light_position0 [] = { 0 , mapsize * 10000 , 0, 1.0 } ;
    
    glClearColor(0.77, 0.90, 0.94, 0.2) ;
    
    glEnable(GL_DEPTH_TEST) ;
    glShadeModel(GL_SMOOTH);
    
    //setShaders() ;
    
    glColorMaterial(GL_FRONT, GL_DIFFUSE) ;
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0) ;
    glLightfv(GL_LIGHT0, GL_COLOR, white_light) ;
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse) ;
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient) ;
    glEnable(GL_LIGHT0) ;
    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient) ;
    glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view) ;
    
    glEnable(GL_LIGHTING );
    glEnable(GL_AUTO_NORMAL) ;
    
    /*
     * Code for anti aliasing, I found the code from the Internet,
     * however can't figure out what the site called now.
     */
    
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); // Make round points, not square points
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);  // Antialias the lines
    
    SkyInit() ;
    roadInit() ;
    grassInit() ;
    
    makeMaze() ;
    
    treeID = createTree() ; // get the treeID from display lists
    roadID = createRoad() ; // mainly as the same above
    grassID = createGrass() ;
    grassAreaID = creatGrassArea() ;
    getRobotRoutes() ;

}


//===================
// Display Function
//===================

void display (void) {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;
    
    glPushMatrix() ;
    drawMap() ;
    glPopMatrix() ;
    
    glFlush() ;
    
}

//===================
// IDLE function
//===================

int timeout = 20 ;
int walkcnt = 0 ;
int clockcnt = 0 ;

void idle(){

    movingPartPositionX = routes[walkcnt][0] ;
    movingPartPositionY = routes[walkcnt][1] ;
    if (walkcnt < routesLength - 1) {
        if (movingPartPositionX - routes[walkcnt + 1][0] == 1) {
            headFor = 3;
            if (glutGet(GLUT_ELAPSED_TIME) % timeout == 0) {
                mpfx -= 0.1 ;
                ++ clockcnt ;
                if (clockcnt % 10 == 9) {
                    ++ walkcnt ;
                }

                glutPostRedisplay() ;
            }
        }
        if (movingPartPositionX - routes[walkcnt + 1][0] == -1) {
            headFor = 1 ;
            if (glutGet(GLUT_ELAPSED_TIME) % timeout == 0) {
                mpfx += 0.1 ;
                ++ clockcnt ;
                if (clockcnt % 10 == 9) {
                    ++ walkcnt ;
                }

                glutPostRedisplay() ;
            }
        }
        if (movingPartPositionY - routes[walkcnt + 1][1] == 1) {
            headFor = 2 ;
            if (glutGet(GLUT_ELAPSED_TIME) % timeout == 0) {
                mpfy -= 0.1 ;
                ++ clockcnt ;
                if (clockcnt % 10 == 9) {
                    ++ walkcnt ;
                }

                glutPostRedisplay() ;
            }
        }
        if (movingPartPositionY - routes[walkcnt + 1][1] == -1) {
            headFor = 0 ;
            if (glutGet(GLUT_ELAPSED_TIME) % timeout == 0) {
                mpfy += 0.1 ;
                ++ clockcnt ;
                if (clockcnt % 10 == 9) {
                    ++ walkcnt ;
                }

                glutPostRedisplay() ;
            }
        }
    }

    if (walkcnt == routesLength) {
        cout << "That's the end, thank you! " << endl ;
        exit(0) ;
    }
}

//===================
// Change The Size of Screen
//===================

void reshape(int w, int h){
    glViewport(0.0, 0.0, (GLsizei)w, (GLsizei)h) ;

    glMatrixMode(GL_PROJECTION) ;
    glLoadIdentity() ;
    
    gluPerspective(60, (GLfloat)w / (GLfloat)h , 1.0, sceneSize) ;
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity() ;
}

//===================
// Keyboard Function
//===================

void keyboard(unsigned char key , int x , int y ){

    switch (key) {
        case 27:
            exit(0) ;
            break;
            
        case ',': // lengthen the distance between the camera and robot
            if ( lookatatdistance + 3 <= mapsize / 2) {
                lookatatdistance += 3 ;
                glutPostRedisplay() ;
            }
            break ;
            
        case '.': // shorten the distance between the camera and robot
            if (lookatatdistance - 3 > 0 ){
                lookatatdistance -= 3 ;
                glutPostRedisplay() ;
            }
            break ;
            
        default:
            break;
    }
}

//======================
// Instruments function
//======================

void instruments(){ // print the spec in the console
    
    cout << " ROBOT IN WONDERLAND" << endl ;
    cout << endl << "================================" << endl << endl ;
    cout << " ,  =======> length the distance between the camera and robot" << endl ;
    cout << " .  =======> shorten the distance between the camera and robot" << endl ;
    cout << "esc =======> exit the program" << endl ;
}

//====================
// main function
//====================

int main (int argc , char * argv[] ){

    srand((unsigned)time(NULL)) ;
    
    instruments() ;
    
    glutInit(&argc, argv) ;
    
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH) ;
    glutInitWindowPosition(100, 100) ;
    glutInitWindowSize(1000, 1000) ;
    glutCreateWindow("Robot in Wonderland") ;
    
    init() ;
    
    glutDisplayFunc(display) ;
    glutReshapeFunc(reshape) ;
    glutKeyboardFunc(keyboard) ;
    glutIdleFunc(idle) ;
    
    glutMainLoop() ;
    return 0 ;
}






