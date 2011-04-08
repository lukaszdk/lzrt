#include <stdio.h>
#include <lwoloader.h>
#include <lwo2.h>
#include <string.h>
#include <stdlib.h>

#ifdef _CELL
	#include <malloc_align.h>
	#include <free_align.h>
#endif



static void SetTri(mesh_t *mesh, int index, lwPoint *v1, lwPoint *v2, lwPoint *v3, float *n1, float *n2, float *n3, float *c)
{
	mesh->tris[index].v1[0] = v1->pos[0];
	mesh->tris[index].v1[1] = v1->pos[1];
	mesh->tris[index].v1[2] = v1->pos[2];

	mesh->tris[index].v2[0] = v2->pos[0];
	mesh->tris[index].v2[1] = v2->pos[1];
	mesh->tris[index].v2[2] = v2->pos[2];
	
	mesh->tris[index].v3[0] = v3->pos[0];
	mesh->tris[index].v3[1] = v3->pos[1];
	mesh->tris[index].v3[2] = v3->pos[2];

	mesh->c[index].c[0] = c[0];
	mesh->c[index].c[1] = c[1];
	mesh->c[index].c[2] = c[2];

	mesh->norms[index].n1[0] = n1[0];
	mesh->norms[index].n1[1] = n1[1];
	mesh->norms[index].n1[2] = n1[2];

	mesh->norms[index].n2[0] = n2[0];
	mesh->norms[index].n2[1] = n2[1];
	mesh->norms[index].n2[2] = n2[2];

	mesh->norms[index].n3[0] = n3[0];
	mesh->norms[index].n3[1] = n3[1];
	mesh->norms[index].n3[2] = n3[2];


}


mesh_t *LoadLWO(char *filename)
{
	unsigned int failID = 0;
	int failpos;
	char *sfailID[5];
	mesh_t *mesh;

	lwObject *lwobj = lwGetObject5(filename, &failID, &failpos);

	memset(sfailID, 0, 5);
	memcpy(sfailID, &failID, 4);
	
	if(failID > 0)
		printf("failID: %s\n", sfailID);
	
	if(lwobj != 0)
	{
		//printf("nlayers : %i\n", lwobj->nlayers);

		if(lwobj->nlayers > 1) return 0;

		// Read layers and polygons

		mesh = new mesh_t;	

		int ti = 0;

		for(int l = 0; l < lwobj->nlayers; l++)
		{
			lwPolygonList *polylist = &lwobj->layer[l].polygon;
			lwPointList *pointlist = &lwobj->layer[l].point;

			// printf("poly count: %i\n", polylist->count);
			// printf("point count: %i\n", pointlist->count);

			int tri_count = 0;

			for(int p = 0; p < polylist->count; p++)
			{
				lwPolygon *poly = &polylist->pol[p];

				if(poly->nverts >= 3)
				{
					tri_count += (poly->nverts-2);
				}

			}

			mesh->numtris = tri_count;

			// Allocate
			mesh->tris = new tri_t[mesh->numtris];
			mesh->norms = new norm_t[mesh->numtris];
			mesh->c = new col_t[mesh->numtris];
			
			//printf("numtries: %i\n", mesh->numtris);
	
			lwPoint *point = pointlist->pt;			

			for(int p = 0; p < polylist->count; p++)
			{
				// printf("poly nverts: %i\n", poly->nverts);

				lwPolygon *poly = &polylist->pol[p];

				// printf("surf name: %s\n", poly->surf->name);


				if(poly->nverts == 3)
				{
					lwPoint *v1 = &point[ poly->v[0].index ];
					lwPoint *v2 = &point[ poly->v[1].index ];
					lwPoint *v3 = &point[ poly->v[2].index ];

					float *n1 = poly->v[0].norm;
					float *n2 = poly->v[1].norm;
					float *n3 = poly->v[2].norm;

					SetTri(mesh, ti, v1, v2, v3, n1, n2, n3, poly->surf->color.rgb);
					
					ti++;
				}
				else
				{
					if(poly->nverts > 3)
					{
						lwPolygon *poly = &polylist->pol[p];

						lwPoint *v1 = &point[ poly->v[0].index ];
						lwPoint *v2 = &point[ poly->v[1].index ];
						lwPoint *v3 = &point[ poly->v[2].index ];
						
						float *n1 = poly->v[0].norm;
						float *n2 = poly->v[1].norm;
						float *n3 = poly->v[2].norm;

						SetTri(mesh, ti, v1, v2, v3, n1, n2, n3, poly->surf->color.rgb);

						// printf("normal %g, %g, %g\n", poly->v[0].norm[0], poly->v[0].norm[1], poly->v[0].norm[2]);

						ti++;

						for(int i=2; i < poly->nverts-1; i++)
						{				
							lwPoint *v2 = &point[ poly->v[i].index ];
							lwPoint *v3 = &point[ poly->v[i+1].index ];

							float *n2 = poly->v[i].norm;
							float *n3 = poly->v[i+1].norm;
	
							SetTri(mesh, ti, v1, v2, v3, n1, n2, n3, poly->surf->color.rgb);
						
							ti++;
						} 
					}
					else
					{
						printf("poly nverts: %i\n", poly->nverts);
					}
				}
			}


		}

	}
	else
	{
		printf("failID : %i , failpos: %i\n", failID, failpos);

	}
	
	lwFreeObject(lwobj);

	return mesh;

}

void FreeMesh(mesh_t *m)
{
	delete [] m->tris;
	delete [] m->norms;
	delete [] m->c;
}

