#include "table_2015.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct vect2 table_contour[] =
{
	vect2( -1500, -1000), vect2( -1500, 1000 ), vect2( 1500, 1000 ), vect2( 1500, -1000 ), vect2( -1500, -1000 )
};

const struct vect2 table_escalier[] =
{
	vect2( -533, 1000 ), vect2( 533, 1000 ), vect2( 533, 420 ), vect2( -533, 420 )
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (vect2*) table_contour, ARRAY_SIZE(table_contour) },
	{ (vect2*) table_escalier, ARRAY_SIZE(table_escalier) }
};
