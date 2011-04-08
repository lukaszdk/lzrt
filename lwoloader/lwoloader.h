#ifndef _LWOLOADER_H_
#define _LWOLOADER_H_

typedef struct
{
	float v1[4];
	float v2[4];
	float v3[4];
} tri_t;

typedef struct
{
	float n1[4];
	float n2[4];
	float n3[4];
} norm_t;

typedef struct
{
	float c[3];
} col_t;

typedef struct
{
	int numtris;
	tri_t *tris;
	norm_t *norms;
	col_t *c;
} mesh_t;


mesh_t *LoadLWO(char *filename);
void FreeMesh(mesh_t *m);

#endif



