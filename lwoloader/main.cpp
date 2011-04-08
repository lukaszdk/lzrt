#include <stdio.h>
#include <lwoloader.h>
#include "SDL.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <lzmath.h>
#include <aabb.h>
#include <iostream>
#include <kdtree.h>

GLuint displaylist;
static mesh_t *mesh;
static AABB maabb;

KDTree kdtree;

float angle = 150.0f;
float trans = -15;

using namespace lzmath;
using namespace std;

void renderaabb(AABB &aabb, bool renderchildren = true);

void renderkdnode(KDTree tree, int n, int depth)
{
	if(n == -1) return;

	kdnode_t *node = &tree.node[n];

	if(tree.IsLeaf(n)) 
	{
		// cout << "numChildren: " << tree.aabb[n].GetNumChildren() << endl;

		renderaabb(tree.aabb[n], false);
	}

	if(node->axis == KD_AXIS_NONE) return;

	AABB *aabb = &tree.aabb[n];

	float dif[4] = { 1,0,1,1 };

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);

	glEnable(GL_BLEND);		// Turn Blending On
	glDisable (GL_LIGHTING);

	glBlendFunc(GL_SRC_ALPHA,GL_ONE);	
	
	if(node->axis == KD_AXIS_X)
	{
		Point3f pmin = Point3f(node->split, aabb->pmin.y, aabb->pmin.z);
		Point3f pmax = Point3f(node->split, aabb->pmax.y, aabb->pmax.z);

		glBegin(GL_QUADS);
			glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
			glVertex3f(pmin.x, pmin.y, pmin.z);
			glVertex3f(pmin.x, pmax.y, pmin.z);
			glVertex3f(pmax.x, pmax.y, pmax.z);
			glVertex3f(pmax.x, pmin.y, pmax.z);
		glEnd();

	}
	
	if(node->axis == KD_AXIS_Y)
	{
		Point3f pmin = Point3f(aabb->pmin.x, node->split, aabb->pmin.z);
		Point3f pmax = Point3f(aabb->pmax.x, node->split, aabb->pmax.z);

		glBegin(GL_QUADS);
			glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
			glVertex3f(pmin.x, pmin.y, pmin.z);
			glVertex3f(pmax.x, pmax.y, pmin.z);
			glVertex3f(pmax.x, pmin.y, pmax.z);
			glVertex3f(pmin.x, pmax.y, pmax.z);
		glEnd();

	}

	if(node->axis == KD_AXIS_Z)
	{
		Point3f pmin = Point3f(aabb->pmin.x, aabb->pmin.y, node->split);
		Point3f pmax = Point3f(aabb->pmax.x, aabb->pmax.y, node->split);

		glBegin(GL_QUADS);
			glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
			glVertex3f(pmin.x, pmin.y, pmin.z);
			glVertex3f(pmax.x, pmin.y, pmin.z);
			glVertex3f(pmax.x, pmax.y, pmax.z);
			glVertex3f(pmin.x, pmax.y, pmax.z);
		glEnd();


	}

	glDisable(GL_BLEND);		// Turn Blending Off
	glEnable(GL_LIGHTING);	

	if(depth == 0) return;

	renderkdnode(tree, node->left, depth-1);
	renderkdnode(tree, node->right, depth-1);


}


void renderkdtree(KDTree tree, int depth)
{
	renderkdnode(tree, 0, depth);	
}

void renderkdtreeleaves(KDTree tree)
{
	int numleaves = tree.GetNumLeaves();
	int *leaves = tree.GetLeaves();

	for(int i=0; i < numleaves; i++)
	{
		renderaabb(tree.aabb[leaves[i]], false);
	}
}


void renderaabb(AABB &aabb, bool renderchildren)
{
	Point3f p0 = Point3f(aabb.pmin.x, aabb.pmax.y, aabb.pmin.z);
	Point3f p1 = Point3f(aabb.pmax.x, aabb.pmax.y, aabb.pmin.z);
	Point3f p2 = Point3f(aabb.pmax.x, aabb.pmin.y, aabb.pmin.z);
	
	Point3f p3 = Point3f(aabb.pmin.x, aabb.pmax.y, aabb.pmax.z);
	Point3f p4 = Point3f(aabb.pmin.x, aabb.pmin.y, aabb.pmax.z);
	Point3f p5 = Point3f(aabb.pmax.x, aabb.pmin.y, aabb.pmax.z);

	Point3f p6 = aabb.pmin;
	Point3f p7 = aabb.pmax;

	// cout << "pmin " << aabb.pmin << " pmax " << aabb.pmax << endl;	

	glLineWidth(1.0f); 

	float dif[4] = { 0,1,0,1 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, dif);

	glDisable (GL_LIGHTING);

	glBegin(GL_LINES);

		// Back
		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);

		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p6.x, p6.y, p6.z);

		glColor3f(1.0f, 1.0f, 0.0f);	
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p1.x, p1.y, p1.z);

		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f(p6.x, p6.y, p6.z);
		glVertex3f(p2.x, p2.y, p2.z);

		// Middle
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p3.x, p3.y, p3.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p7.x, p7.y, p7.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p5.x, p5.y, p5.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p4.x, p4.y, p4.z);
		glVertex3f(p6.x, p6.y, p6.z);

		// Front
		glColor3f(0.0f, 1.0f, 1.0f);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p7.x, p7.y, p7.z);

		glColor3f(0.0f, 1.0f, 1.0f);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p4.x, p4.y, p4.z);

		glColor3f(0.0f, 1.0f, 1.0f);	
		glVertex3f(p7.x, p7.y, p7.z);
		glVertex3f(p5.x, p5.y, p5.z);

		glColor3f(0.0f, 1.0f, 1.0f);
		glVertex3f(p4.x, p4.y, p4.z);
		glVertex3f(p5.x, p5.y, p5.z);
	glEnd();

	glEnable(GL_LIGHTING);

	if(renderchildren)
	{
		AABBInterface **children = aabb.GetChildren();

		for(int i=0; i < aabb.GetNumChildren(); i++)
		{
			renderaabb( *children[i]->GetAABB() );
		}
	}

	// printf("numchildren %i\n", aabb.GetNumChildren());

}

void renderoobb(AABB &aabb, Transform t)
{
	Point3f p0 = t(aabb.oobb[0]);
	Point3f p1 = t(aabb.oobb[1]);
	Point3f p2 = t(aabb.oobb[2]);
	
	Point3f p3 = t(aabb.oobb[3]);
	Point3f p4 = t(aabb.oobb[4]);
	Point3f p5 = t(aabb.oobb[5]);

	Point3f p6 = t(aabb.oobb[6]);
	Point3f p7 = t(aabb.oobb[7]);

	glLineWidth(1.0f); 

	float dif2[4] = { 0,0,1,1 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, dif2);

	glBegin(GL_LINES);

		// Back
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p6.x, p6.y, p6.z);

		glColor3f(0.0f, 1.0f, 0.0f);	
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p1.x, p1.y, p1.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p6.x, p6.y, p6.z);
		glVertex3f(p2.x, p2.y, p2.z);

		// Middle
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p3.x, p3.y, p3.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p7.x, p7.y, p7.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p5.x, p5.y, p5.z);

		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(p4.x, p4.y, p4.z);
		glVertex3f(p6.x, p6.y, p6.z);

		// Front
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p7.x, p7.y, p7.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p4.x, p4.y, p4.z);

		glColor3f(0.0f, 1.0f, 0.0f);	
		glVertex3f(p7.x, p7.y, p7.z);
		glVertex3f(p5.x, p5.y, p5.z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(p4.x, p4.y, p4.z);
		glVertex3f(p5.x, p5.y, p5.z);
	glEnd();

	AABBInterface **children = aabb.GetChildren();

	for(int i=0; i < aabb.GetNumChildren(); i++)
	{
		renderoobb( *children[i]->GetAABB(), t );
	}


}

void mainloopGL()
{
	int deltatime, curtime;
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslatef(0,0, trans);
	glRotatef(angle, 0, 1, 0);

	Transform t = Translate(0,0, trans);
	Transform r = RotateY(angle);

	Transform tt = t * r;

	// renderaabb(maabb);
	//renderoobb(maabb, tt);
	
	/*
	curtime = SDL_GetTicks(); // glutGet(GLUT_ELAPSED_TIME);
	deltatime = curtime - time;
	time = curtime;

	glCameraFirstPerson(deltatime, movex, movez, turnx, turny);
	
	*/	

	angle += 0.2;

	//float dif[4] = { 1,0,0,1 };
	// glMaterialfv(GL_FRONT, GL_DIFFUSE, dif);


    glCallList(displaylist);

	// renderkdtreeleaves(kdtree);

	// renderkdtree(kdtree, 10);
}

void initGL(int width, int height, char* title)
{
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
		fprintf(stderr,"Couldn't initialize SDL: %s\n",SDL_GetError());
		exit( 1 );
	}

	SDL_GL_SetAttribute( SDL_GL_RED_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 16 );
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
	// SDL_GL_SetAttribute( SDL_GL_SWAP_CONTROL, 1 );

	if ( SDL_SetVideoMode( width, height, 32, SDL_OPENGL ) == NULL ) 
	{
		fprintf(stderr, "Couldn't set GL mode: %s\n", SDL_GetError());
		SDL_Quit();
		exit(1);
	}

	SDL_WM_SetCaption( title, NULL );

	glClearColor (0.5f, 0.5f, 0.5f, 1.0f);
  	glShadeModel (GL_SMOOTH);
  	glEnable (GL_DEPTH_TEST);

	//glEnable (GL_LIGHTING);
	// glEnable (GL_LIGHT0);
  	
  	glViewport     ( 0, 0, width, height );
  	glMatrixMode   ( GL_PROJECTION );  // Select The Projection Matrix
  	glLoadIdentity ( );                // Reset The Projection Matrix
   	gluPerspective ( 60, (float)4/3, 1.0, 5000.0 );
 
    glMatrixMode   ( GL_MODELVIEW );  // Select The Model View Matrix
}



int main(int argc, char **argv)
{	
	initGL(640, 480, "OpenGL");

	if(argc == 2)
	{
		mesh = LoadLWO(argv[1]);

		displaylist = glGenLists(1);
		glNewList(displaylist, GL_COMPILE);

		maabb.Reset();

		maabb.AllocChildren(mesh->numtris);

		glBegin(GL_TRIANGLES);
			for(int i=0; i < mesh->numtris; i++)
			{
				
				//if( (i % 3) == 0) glColor3f(1.0f, 1.0f, 1.0f);
				//if( (i % 3) == 1) glColor3f(0.6f, 0.6f, 0.6f);
				// if( (i % 3) == 2) glColor3f(0.3f, 0.3f, 0.3f);
				
				/*
				printf("tris[%i].v1 = (%g, %g, %g)\n", i, mesh->tris[i].v1[0], mesh->tris[i].v1[1], mesh->tris[i].v1[2]);
				printf("tris[%i].v2 = (%g, %g, %g)\n", i, mesh->tris[i].v2[0], mesh->tris[i].v2[1], mesh->tris[i].v2[2]);
				printf("tris[%i].v3 = (%g, %g, %g)\n", i, mesh->tris[i].v3[0], mesh->tris[i].v3[1], mesh->tris[i].v3[2]);
				*/				
		
				float dif[4] = { mesh->tris[i].c[0], mesh->tris[i].c[1], mesh->tris[i].c[2] ,1 };

				glMaterialfv(GL_FRONT, GL_DIFFUSE, dif);

				glColor3f(mesh->tris[i].c[0], mesh->tris[i].c[1], mesh->tris[i].c[2]);

				glNormal3f(mesh->norms[i].n1[0], mesh->norms[i].n1[1], mesh->norms[i].n1[2]);
				glVertex3f(mesh->tris[i].v1[0], mesh->tris[i].v1[1],  mesh->tris[i].v1[2]);
				
				glColor3f(mesh->tris[i].c[0], mesh->tris[i].c[1], mesh->tris[i].c[2]);
				glNormal3f(mesh->norms[i].n2[0], mesh->norms[i].n2[1], mesh->norms[i].n2[2]);
				glVertex3f(mesh->tris[i].v2[0], mesh->tris[i].v2[1],  mesh->tris[i].v2[2]);

				glColor3f(mesh->tris[i].c[0], mesh->tris[i].c[1], mesh->tris[i].c[2]);				
				glNormal3f(mesh->norms[i].n3[0], mesh->norms[i].n3[1], mesh->norms[i].n3[2]);
				glVertex3f(mesh->tris[i].v3[0], mesh->tris[i].v3[1],  mesh->tris[i].v3[2]);
				

				Point3f p1 = Point3f(mesh->tris[i].v1[0], mesh->tris[i].v1[1], mesh->tris[i].v1[2]);
				Point3f p2 = Point3f(mesh->tris[i].v2[0], mesh->tris[i].v2[1], mesh->tris[i].v2[2]);
				Point3f p3 = Point3f(mesh->tris[i].v3[0], mesh->tris[i].v3[1], mesh->tris[i].v3[2]);

				
				AABB *child_aabb = new AABB();				
		
				child_aabb->Reset();
				child_aabb->Union(p1);
				child_aabb->Union(p2);
				child_aabb->Union(p3);

				child_aabb->SetOOBB();
				maabb.AddChild(child_aabb);

				//				cout << "child pmin " << child_aabb->pmin << endl;

				maabb.Union(p1);
				maabb.Union(p2);
				maabb.Union(p3);
	

			}
		glEnd();
		glEndList();

		maabb.SetOOBB();

		kdtree.BuildTree(maabb, 50);

		cout << "GetNumLeaves: " << kdtree.GetNumLeaves() << endl;

		// kdtree.Dump();


		// exit(0);

		
		bool done = false;

		while(! done )
		{
			SDL_Event event;

			while( SDL_PollEvent( &event ) )
			{
				SDLKey key = event.key.keysym.sym;

				switch( event.type ) 
				{
		  			case SDL_QUIT: done = true; break;
					case SDL_KEYDOWN: if ( key == SDLK_ESCAPE) done = true; break;
				}
			}
			
			mainloopGL();

			SDL_Delay( 0 );
			SDL_GL_SwapBuffers( );
		}


	}

	return 0;
}

