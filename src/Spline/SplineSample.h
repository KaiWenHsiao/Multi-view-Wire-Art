#pragma once
#include <vector>

namespace HSSSpline
{
	struct Sample
	{
		int seg_idx;//Ctrl Point 區段
		double _t;  //內差參數 [0,1]
		Sample():seg_idx(0),_t(0){}
		Sample(int idx,double t):seg_idx(idx),_t(t){}
		Sample(int idx,double t,double v):seg_idx(idx),_t(t){}
		Sample(const Sample& sample):seg_idx(sample.seg_idx),_t(sample._t){}

		void operator= (Sample& input)
		{
			seg_idx = input.seg_idx;
			_t	    = input._t;
		}

		bool operator> (Sample& input) const
		{
			if (seg_idx > input.seg_idx){return true;}
			else if(seg_idx == input.seg_idx){return _t > input._t;}
			else{return false;}
		}
		bool operator>= (Sample& input) const
		{
			if (seg_idx > input.seg_idx){return true;}
			else if(seg_idx == input.seg_idx){return _t >= input._t;}
			else{return false;}
		}
		bool operator< (Sample& input) const
		{
			if (seg_idx < input.seg_idx){return true;}
			else if(seg_idx == input.seg_idx){return _t < input._t;}
			else{return false;}
		}
		bool operator<= (Sample& input) const
		{
			if (seg_idx < input.seg_idx){return true;}
			else if(seg_idx == input.seg_idx){return _t <= input._t;}
			else{return false;}
		}
		bool operator== (Sample& input) const
		{
			return seg_idx == input.seg_idx && _t == input._t;
		}
		bool operator!= (Sample& input) const
		{
			return seg_idx != input.seg_idx || _t != input._t;
		}
	};
	typedef std::vector< Sample > Samples;

	template <int D>
	struct PathPoint
	{
		double val[D]; //x,y,w,top,bottom
		PathPoint(){for (int i=0;i<D;i++){val[i]=0;}}
		PathPoint(double _x,double _y)
		{
			for (int i=0;i<D;i++){val[i]=1;}
			if (D>=2){val[0]=_x;val[1]=_y;}
		}

		double& operator[](int idx)
		{
			if (idx<D)return val[idx];
			else return val[0];
		}

		void operator= (PathPoint<D>& input)
		{
			for (int i=0;i<D;i++){val[i]=input[i];}
		}

		std::string ToString()
		{
			std::string str;
			for (int i=0;i<D;i++)
			{
				char buff[100];
				sprintf(buff,"%lf ",val[i]);
				str+=buff;
			}
			return str;
		}
	};
	typedef PathPoint<2> Vector2;

	template <int D>
	struct PathPoints{
		std::vector< PathPoint<D> > val;
		std::vector< PathPoint<D> >& operator()(void){return val;}
		PathPoint<D>& operator[](unsigned int i){return val[i];}
	};
}
