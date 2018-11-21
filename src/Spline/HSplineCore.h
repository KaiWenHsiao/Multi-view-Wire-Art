#pragma once

#include "SplineSample.h"
#include "../LinearSystem/include/LinearSystemLib.h"

#include <vector>

namespace HSSSpline
{
	template <int D>
	class HSplineCore
	{
	protected:
		//Ρ絬把计
		struct Spline_Ele
		{
			double val[D][4];
		};
		std::vector< Spline_Ele > m_LineSeg_List;

	//Spline	
	public:
		int BuildingSpline(PathPoints<D> &_points)
		{
			if (_points().size() < 2)
			{
				std::cerr << "Cubic spline point number less than 2!\n";
				m_LineSeg_List.clear();
				return 1;
			}

			using namespace LinearSystemLib;

			/*ミA痻皚*/
			GeneralSparseMatrix* GA = new GeneralSparseMatrix;
			GA->Create( _points().size(), _points().size());
			setup_matrixA(*GA, _points().size());

			//ミB痻皚
			// only support double type.
			//-------------------------------------------------
			const int dim = D;//x, y, w, h1, h2舱
			double** B = new double*[dim]; // create 2d.

			for (int i=0;i<dim;i++){B[i] = new double[_points().size()];}

			for (unsigned int i = 0; i < _points().size(); ++i)
			{
				if (i == 0)
				{
					for (int d=0;d<dim;d++){B[d][i] = 3*(_points[1][d] - _points[0][d]);}
				}else if (i == _points().size()-1)
				{
					for (int d=0;d<dim;d++){B[d][i] = 3*(_points[i][d] - _points[i-1][d]);}
				}else{
					for (int d=0;d<dim;d++){B[d][i] = 3*(_points[i+1][d] - _points[i-1][d]);}
				}
			}

			// create the linear system A x = B.
			//-------------------------------------------------
			// note: ミ A and B memory 常盢パ sls 璽砫恨瞶
			// ps. A and B must allocate from heap.
			// ps. GA 斗锣Θ, stable sparse matrix.

			SparseLinearSystem sls( new StableSparseMatrix(*GA), B, dim );

			// solving it !! A * x = B.
			//-------------------------------------------------
			try
			{
				// the solution will be stored here.
				double** S = 0;

				// solve it !!
				bool result = GeneralSparseLSSolver::GetInstance()->Solve( &sls, S );

				// output the solution..!!
				//-------------------------------------------------
				if( !result )
					cerr << "solve error!!! check your matrix." << endl;
				else
				{
					m_LineSeg_List.clear();
					for (unsigned int i = 0; i < _points().size()-1; ++i)
					{
						Spline_Ele ele;
						for (int d=0;d<dim;d++)
						{
							ele.val[d][0] = (double)( S[d][i+1] + S[d][i] - 2 * (_points[i+1][d] - _points[i][d]) );
							ele.val[d][1] = (double)( 3 * (_points[i+1][d] - _points[i][d]) - 2 * S[d][i] - S[d][i+1] );
							ele.val[d][2] = (double)( S[d][i] );
							ele.val[d][3] = (double)( _points[i][d] );
						}
						m_LineSeg_List.push_back(ele);
					}
				}

				// 斗璶︽ release solution memory, AB ぃノ!!
				//-------------------------------------------------
				if (S!=0)
				{
					for( int i = 0 ; i < dim ; ++i )delete[] S[i];
					delete[] S;
					S = NULL;
				}
			} 
			catch( exception e )
			{
				cerr << e.what() << endl;
			}
			return 0;
		}

		void setup_matrixA(LinearSystemLib::GeneralSparseMatrix& _matrix, int _size)
		{
			_matrix.SetElement( 0, 0, 2.0);
			for (int i = 1; i < _size-1; ++i)
			{
				_matrix.SetElement(i, i, 4.0);
			}
			for (int i = 0; i < _size-1; ++i)
			{
				_matrix.SetElement(i, i+1, 1.0);
				_matrix.SetElement(i+1, i, 1.0);
			}
			_matrix.SetElement( _size-1, _size-1, 2.0 );
		}

	//Get Info
	public:
		/*眔琿计*/
		int n_segs(){return m_LineSeg_List.size();}
		double get_value(int dim,int _seg, double _t)
		{
			double t2 = _t*_t, t3 = t2*_t;
			return (double)( m_LineSeg_List[_seg].val[dim][0] * t3 + m_LineSeg_List[_seg].val[dim][1] * t2 + m_LineSeg_List[_seg].val[dim][2] * _t + m_LineSeg_List[_seg].val[dim][3] );
		}

		//Ω稬だ
		double get_D1_value(int dim,int _seg, double _t)
		{
			double t2 = _t*_t;
			return (double)( 3 * m_LineSeg_List[_seg].val[dim][0] * t2 + 2 * m_LineSeg_List[_seg].val[dim][1] * _t + m_LineSeg_List[_seg].val[dim][2] );
		}

		double get_D2_value(int dim,int _seg, double _t)
		{
			return (double)( 6 * m_LineSeg_List[_seg].val[dim][0] * _t + 2 * m_LineSeg_List[_seg].val[dim][1]);
		}

		/*眔spline琘翴, seg琌琿, t琌把计*/
		PathPoint<D>     get_point(int _seg, double _t)
		{
			HSSSpline::PathPoint<D> p;
			for (int i=0;i<D;i++)
			{
				p[i] = get_value(i,_seg,_t);
			}
			return p;
		}
		PathPoint<D>     get_point(const Sample& sample){return get_point(sample.seg_idx,sample._t);}
	};
}