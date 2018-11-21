#include "HKImgThinning.h"

using namespace HKCV;
using namespace cv;

//////////////////////////////////////////////////////////////////////////
/// Direction masks:		
///   N	   S	 W     E		
static	int	thin_masks[] = { 0200, 0002, 0040, 0010 };

/*	True if pixel neighbor map indicates the pixel is 8-simple and	*/
/*	not an end point and thus can be deleted.  The neighborhood	*/
/*	map is defined as an integer of bits abcdefghi with a non-zero	*/
/*	bit representing a non-zero pixel.  The bit assignment for the	*/
/*	neighborhood is:						*/
/*									*/
/*				a b c					*/
/*				d e f					*/
/*				g h i					*/
static	unsigned char thin_delete[512] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

/// Image thinning
bool HKCV::Thinning(Mat& _img)
{
	if(!_img.data || _img.type() != CV_8UC1)
	{
		CV_Error(CV_StsBadArg, "_img has incorrect data or type (not CV_8UC1)");	
		return false;
	}

	int		xsize, ysize;	/// Image resolution
	int		x, y;			/// Pixel location		
	int		i;				/// Pass index			
	int		pc = 0;			/// Pass count			
	int		count = 1;		/// Deleted pixel count	
	int     pre_cnt = -1;
	int     rep_cnt = 0;
	int		p, q;			/// Neighborhood maps of adjacent
	int		m;				/// scanline : Deletion direction mask
	uchar*  qb;				/// cells : Neighborhood maps of previous		

	xsize = _img.cols;
	ysize = _img.rows;

	qb = new uchar[xsize];
	qb[xsize-1] = 0;		/* Used for lower-right pixel	*/

	uchar* val_ptr;
	uchar* val_ptr2;

	/// Scan image while deletions
	while( count ) 
	{		
		pc++;
		count = 0;
		for( i = 0 ; i < 4 ; i++ ) 
		{
			m = thin_masks[i];

			/// Build initial previous scan buffer.		
			val_ptr = _img.ptr<uchar>(0);
			p = val_ptr[0] != 0;
			for ( x = 0 ; x < xsize-1 ; x++ )
				qb[x] = p = ((p<<1)&0006) | (val_ptr[x+1] != 0);

			/// Scan image for pixel deletion candidates.
			for( y = 0 ; y < ysize-1 ; y++ ) 
			{
				val_ptr2 = _img.ptr<uchar>(y);
				val_ptr  = _img.ptr<uchar>(y+1);

				q = qb[0];
				p = ((q<<3)&0110) | (val_ptr[0] != 0);

				for ( x = 0 ; x < xsize-1 ; x++ ) 
				{
					q = qb[x];
					p = ((p<<1)&0666) | ((q<<3)&0110) |
						(val_ptr[x+1] != 0);
					qb[x] = p;
					if ( ((p&m) == 0) && thin_delete[p] ) 
					{
						count++;
						val_ptr2[x] = 0;
					}
				}

				/// Process right edge pixel.
				p = (p<<1)&0666;
				if	( (p&m) == 0 && thin_delete[p] ) 
				{
					count++;
					val_ptr2[xsize-1] = 0;
				}
			}

			/// Process bottom scan line.
			val_ptr = _img.ptr<uchar>(ysize-1);
			for( x = 0 ; x < xsize ; x++ ) 
			{
				q = qb[x];
				p = ((p<<1)&0666) | ((q<<3)&0110);
				if( (p&m) == 0 && thin_delete[p] ) 
				{
					count++;
					val_ptr[x] = 0;
				}
			}
		}

		if(pre_cnt != count)
		{
			pre_cnt = count;
			rep_cnt = 0;
		}
		else
			rep_cnt++;

		if(rep_cnt >= 5)
		{
			delete [] qb;
			qb = NULL;
			return false;
		}
	}

	delete [] qb;
	qb = NULL;

	return true;
}