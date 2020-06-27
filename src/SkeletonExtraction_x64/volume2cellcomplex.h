/************************************************************************/
/* 
Author: Lu Liu
Date:	5/1

*/
/************************************************************************/

#pragma once

#include <vector>
#include <iostream>
using namespace std;


/************************************************************************/
/*
based on 6 connectivity
points 
every point has 6 parents edge, 
if some incident edge is not in cell complex, fill it with -1
edges 
every edge has 4 face parents, if incident face doesn't exist in 
cell complex, fill it with -1
every edge must have 2 children
faces
every face has 2 cell parents, if incident cell doesn't exist in 
cell complex, fill it with -1
every face has 4 children edges, and must have based on definition of 
cell complex

attention: make sure, there is no foreground points on the boundary of the 
volume, some edges and faces are not going to be visited to check if 
foreground, the algorithm assume they are background.
*/
/************************************************************************/
//template <class T1 >
//void volume2CC(	const T1* data,										//a 3d array, saved linearly, [x][y][z]
//				const int sizexyz[ 3 ], 				//size of the volume	
//				const double spacxyz[ 3 ],		//spacing between two points in the volume
//				const double cornerPos[ 3 ],			//corner position of the volume
//	
//				const T1 threshhold,									//above the threshold is considered as foreground
//
//				vector<float>& ccPts,								//cell complex points positions	
//				vector<int>& ptParents,							//point in the cell complex, every point's parents
//				vector<int>& edgeParents, vector<int>&edgeChildren,	//edges in teh cell complex, children and parenets,
//				vector<int>& faceParents, vector<int>& faceChildren,	//faces in teh cell complex, children and parents
//				vector<int>& cellChildren)	;						//cells in the cell complex, all its children(face index)
template <class T1 >
void volume2CC(	const T1* data,										//a 3d array, saved linearly, [x][y][z]
			   const int sizexyz[ 3 ], 				//size of the volume	
			   const double spacxyz[ 3 ],		//spacing between two points in the volume
			   const double cornerPos[ 3 ],			//corner position of the volume
			   const T1 threshhold,									//above the threshold is considered as foreground

			   vector<float>& ccPts,								//cell complex points positions	
			   vector<int>& ptParents,							//point in the cell complex, every point's parents
			   vector<int>& edgeParents, vector<int>&edgeChildren,	//edges in teh cell complex, children and parenets,
			   vector<int>& faceParents, vector<int>& faceChildren,	//faces in teh cell complex, children and parents
			   vector<int>& cellChildren)							//cells in the cell complex, all its children(face index)
			   
			  
{
	int sizex = sizexyz[ 0 ];
	int sizey = sizexyz[ 1 ];
	int sizez = sizexyz[ 2 ];

	int size = sizex * sizey * sizez;

	//////////////////////////////////////////////////////////////////////////
	/*int count = 0;
	for( int i = 0; i < size; i++  )
	{
		if( data[ i ] == 1 )
		{
			count ++;
		}

	}
	cout<<"data is 1, #"<<count<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//reserve estimated size
	ptParents.reserve( size );
	edgeParents.reserve( size * 2 / 3);
	edgeParents.reserve( size  / 2);
	faceParents.reserve( size / 2 );
	faceChildren.reserve( size * 0.75 );
	cellChildren.reserve( size );

	

	//vector<vector<vector<int> > > ptInds;
	//vector<vector<vector<vector<int> > > >edgeInds;	
	static int ***ptInds = nullptr;
	if (ptInds == nullptr){
		cout << "Start Allocate PtInds.....\n";
		ptInds = new int**[sizex];
		for (int i = 0; i < sizex; i++)
		{
			ptInds[i] = new int*[sizey];
			for (int j = 0; j < sizey; j++)
			{
				ptInds[i][j] = new int[sizez];
				for (int k = 0; k < sizez; k++){
					ptInds[i][j][k] = -1;
				}
			}
		}
	}
	else{
		cout << "PtInds Reuse.....\n";
		for (int i = 0; i < sizex; i++)
		{			
			for (int j = 0; j < sizey; j++)
			{				
				for (int k = 0; k < sizez; k++){
					ptInds[i][j][k] = -1;
				}
			}
		}
	}

	static int ****edgeInds = nullptr;
	if (edgeInds == nullptr){
		cout << "Start Allocate EdgeInds.....\n";
		edgeInds = new int***[sizex];
		for (int i = 0; i < sizex; i++)
		{
			edgeInds[i] = new int**[sizey];
			for (int j = 0; j < sizey; j++)
			{
				edgeInds[i][j] = new int*[sizez];
				for (int k = 0; k < sizez; k++)
				{
					edgeInds[i][j][k] = new int[3];
					edgeInds[i][j][k][0] = -1;
					edgeInds[i][j][k][1] = -1;
					edgeInds[i][j][k][2] = -1;
				}
			}
		}
	}
	else{
		cout << "EdgeInds Reuse.....\n";
		for (int i = 0; i < sizex; i++)
		{			
			for (int j = 0; j < sizey; j++)
			{				
				for (int k = 0; k < sizez; k++)
				{					
					edgeInds[i][j][k][0] = -1;
					edgeInds[i][j][k][1] = -1;
					edgeInds[i][j][k][2] = -1;
				}
			}
		}
	}
	
	

	cout<<"going through all the vertices.....";
	int pos = 0;
	int ind = 0;
	float xpos, ypos, zpos;
	xpos = cornerPos[ 0 ];
	for( int i = 0; i < sizex; i ++ )
	{
		ypos = cornerPos[ 1 ];
		for( int j = 0; j < sizey ; j ++ )
		{
			zpos = cornerPos[ 2 ];
			for( int k = 0; k < sizez; k ++ )
			{
				if( data[ pos ] > threshhold )
				{									
					ptInds[ i ][ j ][ k  ] = ind;
					ccPts.push_back( xpos  );
					ccPts.push_back( ypos  );
					ccPts.push_back( zpos  );

					//////////////////////////////////////////////////////////////////////////
					if( ind < 5  )
					{
						cout<<"{"
							<<xpos<<","
							<<ypos<<","
							<<zpos<<","
							<<"}";
					}
					ind ++;
				}

				pos ++;
				zpos += spacxyz[ 2 ];
			}

			ypos += spacxyz[ 1 ];
		}

		xpos += spacxyz[ 0 ];
	}
	cout<<"done\n";

	//////////////////////////////////////////////////////////////////////////
	cout<<"ccpts size:"<<ccPts.size()/3<<endl;
	//////////////////////////////////////////////////////////////////////////


	//go through all the edges, add edge.children, point.parents
	cout<<"going through all the edges, setting parents and children.....";
	int verNum = ind;
	ptParents.resize( 6 * verNum, -1 );
	ind = 0;
	for( int i = 0; i < sizex - 1 ; i ++ )
	{
		for( int j = 0; j < sizey - 1; j ++ )
		{
			for( int k = 0; k < sizez - 1; k ++ )
			{
				int curInd = ptInds[ i ][ j ][ k ];
				if( curInd != -1 )	//has already been set
				{
					//go through the potential 3 new edges in x, y and z directions
					int ptIndxyz[ 3 ]=
					{ 
						ptInds[ i + 1][ j ][ k ],
							ptInds[ i ][ j + 1 ][ k ],
							ptInds[ i ][ j ][ k + 1 ]
					};

					for( int ii = 0; ii < 3; ii ++ )  
					{
						if( ptIndxyz[ ii ] != -1 )//edge exist!
						{
							//edge's child
							edgeChildren.push_back( curInd );
							edgeChildren.push_back( ptIndxyz[ ii ] );

							//point's parent
							ptParents[ 6 * curInd + ii + 3 ] = ind;			//set the current point's parent in positive direction
							ptParents[ 6 * ptIndxyz[ ii ] + ii ] = ind;		//set the point's parent in the negative direction

							//set the edge's index
							edgeInds[ i ][ j ][ k ][ ii ] = ind;

							ind ++;
						}
					}					
				}
			}
		}
	}
	cout<<"done\n";
	//////////////////////////////////////////////////////////////////////////
	cout<<"edge numbeR"<<ind<<endl;
	//////////////////////////////////////////////////////////////////////////

	//clear index for points, allocate index for faces
	static int ****faceInds = nullptr;
	if (faceInds == nullptr){
		cout << "Start Allocate FaceInds.....\n";

		faceInds = new int***[sizex];
		for (int i = 0; i < sizex; i++)
		{
			faceInds[i] = new int**[sizey];
			for (int j = 0; j < sizey; j++)
			{
				faceInds[i][j] = new int*[sizez];
				for (int k = 0; k < sizez; k++){
					faceInds[i][j][k] = new int[3];
					faceInds[i][j][k][0] = -1;
					faceInds[i][j][k][1] = -1;
					faceInds[i][j][k][2] = -1;
				}
			}
		}
	}
	else{
		cout << "FaceInds Reuse.....\n";

		
		for (int i = 0; i < sizex; i++)
		{			
			for (int j = 0; j < sizey; j++)
			{				
				for (int k = 0; k < sizez; k++){					
					faceInds[i][j][k][0] = -1;
					faceInds[i][j][k][1] = -1;
					faceInds[i][j][k][2] = -1;
				}
			}
		}
	}
	

	//go through all the faces, add them if exists
	cout<<"going through all the faces, setting parents and children.....";
	edgeParents.resize( ind * 4, -1 );
	ind = 0;

	for( int i = 0; i < sizex - 1 ; i ++ )
	{
		for( int j = 0; j < sizey - 1; j ++ )
		{
			for( int k = 0; k < sizez - 1; k ++ )
			{
				//x direction
				int edgeInd4[ 4 ] = 
				{					
					edgeInds[ i ][ j ][ k ][ 1 ],
						edgeInds[ i ][ j + 1][ k ][ 2 ],
						edgeInds[ i ][ j ][ k + 1][ 1 ],
						edgeInds[ i ][ j ][ k ][ 2 ]
				};
				bool faceExist = 
					(edgeInd4[ 0 ] != -1 ) && 
					(edgeInd4[ 1 ] != -1 ) && 
					(edgeInd4[ 2 ] != -1 ) && 
					(edgeInd4[ 3 ] != -1 ) ;

				if( faceExist )
				{
					for( int ii  = 0; ii < 4; ii ++ )
					{
						faceChildren.push_back( edgeInd4[ii ] );
					}

					edgeParents[ 4 * edgeInd4[ 0 ] + 2 ] = ind;
					edgeParents[ 4 * edgeInd4[ 1 ] + 1 ] = ind;
					edgeParents[ 4 * edgeInd4[ 2 ] + 0 ] = ind;
					edgeParents[ 4 * edgeInd4[ 3 ] + 3 ] = ind;

					faceInds[ i ][ j ][ k  ][ 0 ] = ind;
					ind ++;
				}

				//y direction
				edgeInd4[ 0 ] = edgeInds[ i ][ j ][ k ][ 2 ];
				edgeInd4[ 1 ] = edgeInds[ i ][ j ][ k ][ 0 ];
				edgeInd4[ 2 ] = edgeInds[ i + 1 ][ j ][ k ][ 2 ];
				edgeInd4[ 3 ] = edgeInds[ i ][ j ][ k + 1][ 0 ];						

				faceExist = 
					(edgeInd4[ 0 ] != -1 ) && 
					(edgeInd4[ 1 ] != -1 ) && 
					(edgeInd4[ 2 ] != -1 ) && 
					(edgeInd4[ 3 ] != -1 ) ;
				if( faceExist )
				{
					for( int ii  = 0; ii < 4; ii ++ )
					{
						faceChildren.push_back( edgeInd4[ii ] );
					}

					edgeParents[ 4 * edgeInd4[ 0 ] + 2 ] = ind;
					edgeParents[ 4 * edgeInd4[ 1 ] + 3 ] = ind;
					edgeParents[ 4 * edgeInd4[ 2 ] + 0 ] = ind;
					edgeParents[ 4 * edgeInd4[ 3 ] + 1 ] = ind;					

					faceInds[ i ][ j ][ k  ][ 1 ] = ind;

					ind ++;
				}



				//z direction
				edgeInd4[ 0 ] = edgeInds[ i ][ j ][ k ][ 0 ];
				edgeInd4[ 1 ] = edgeInds[ i + 1 ][ j ][ k ][ 1 ];
				edgeInd4[ 2 ] = edgeInds[ i ][ j + 1 ][ k ][ 0 ];
				edgeInd4[ 3 ] = edgeInds[ i ][ j ][ k ][ 1 ];
				faceExist = 
					(edgeInd4[ 0 ] != -1 ) && 
					(edgeInd4[ 1 ] != -1 ) && 
					(edgeInd4[ 2 ] != -1 ) && 
					(edgeInd4[ 3 ] != -1 ) ;
				if( faceExist )
				{
					for( int ii  = 0; ii < 4; ii ++ )
					{
						faceChildren.push_back( edgeInd4[ii ] );
					}

					edgeParents[ 4 * edgeInd4[ 0 ] + 2 ] = ind;
					edgeParents[ 4 * edgeInd4[ 1 ] + 3 ] = ind;
					edgeParents[ 4 * edgeInd4[ 2 ] + 0 ] = ind;
					edgeParents[ 4 * edgeInd4[ 3 ] + 1 ] = ind;

					faceInds[ i ][ j ][ k  ][ 2 ] = ind;

					ind ++;
				}
			}
		}
	}
	cout<<"done\n";
	//////////////////////////////////////////////////////////////////////////
	cout<<"Face numbeR:"<<ind<<endl;
	//////////////////////////////////////////////////////////////////////////

	//go through the cells
	cout<<"going through all the cells, setting parents and children.....\t";
	faceParents.resize(  ind * 2, -1 );
	ind = 0;
	for( int i = 0; i < sizex - 1 ; i ++ )
	{
		for( int j = 0; j < sizey - 1; j ++ )
		{
			for( int k = 0; k < sizez - 1; k ++ )
			{
				//get the 6 faces for the cell with corner [i,j,k]
				int faceInd6[ 6 ] ={					
					faceInds[ i ][ j ][ k ][ 0 ],
						faceInds[ i + 1 ][ j ][ k ][ 0 ],
						faceInds[ i ][ j ][ k ][ 1 ],
						faceInds[ i ][ j + 1 ][ k ][ 1 ],
						faceInds[ i ][ j ][ k ][ 2 ],
						faceInds[ i ][ j ][ k + 1 ][ 2 ]
				};
				bool cellExist = (faceInd6[ 0 ] != -1 ) &&
					(faceInd6[ 1 ] != -1 ) &&
					(faceInd6[ 2 ] != -1 ) &&
					(faceInd6[ 3 ] != -1 ) &&
					(faceInd6[ 4 ] != -1 ) &&
					(faceInd6[ 5 ] != -1 ) ;
				if( cellExist )
				{
					//add the cell's children
					for( int ii = 0; ii < 6; ii ++ )
					{
						cellChildren.push_back( faceInd6[ ii ]);					
					}
					faceParents[ 2 * faceInd6[ 0 ] + 1 ] = ind;
					faceParents[ 2 * faceInd6[ 1 ] ] = ind;
					faceParents[ 2 * faceInd6[ 2 ] + 1 ] = ind;
					faceParents[ 2 * faceInd6[ 3 ] ] = ind;
					faceParents[ 2 * faceInd6[ 4 ] + 1 ] = ind;
					faceParents[ 2 * faceInd6[ 5 ] ] = ind;

					ind ++;
				}
			}
		}
	}	
	cout<<"done\n";

	//////////////////////////////////////////////////////////////////////////
	cout<<"cell number:"<<ind<<endl;
	//////////////////////////////////////////////////////////////////////////
	/*
	cout << "Start release  edgeInds memories\n";
	for (int i = 0; i < sizex; i++){
		for (int j = 0; j < sizey; j++){
			for (int k = 0; k < sizez; k++){
				delete[] edgeInds[i][j][k];				
			}
			delete[] edgeInds[i][j]; 			
		}
		delete[] edgeInds[i];		
	}
	delete[] edgeInds;
	
	cout << "Start release  faceInds memories\n";
	for (int i = 0; i < sizex; i++){
		for (int j = 0; j < sizey; j++){
			for (int k = 0; k < sizez; k++){
				delete[] faceInds[i][j][k];
			}
			delete[] faceInds[i][j];
		}
		delete[] faceInds[i];
	}
	delete[] faceInds;
	
	cout << "Start release  ptInds memories\n";
	for (int i = 0; i < sizex; i++)
	{
		for (int j = 0; j < sizey; j++)
			delete[] ptInds[i][j];

		delete[] ptInds[i];
	}
	delete[] ptInds;

	cout << "Release memories done\n";
	*/
}
