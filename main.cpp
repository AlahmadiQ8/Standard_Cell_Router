#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "oaDesignDB.h"
#include "mycode.h"

#include <vector>
#include <list>
#include <stack>
#include <math.h>
#include <sstream>
#include <iomanip>

#define VDD_TOP    13300
#define VSS_BOTTOM 671

using namespace std;
using namespace oa;

static oaNativeNS ns;

//******************************************************
//		Grid class
//******************************************************
class Grid 
{
public:
	Grid (int x_max, int y_max)
	{
		s_x=x_max;
 		s_y=y_max;
		size = s_x*s_y;
		grid_layer0 = new bool[size];  // actual size counting x_max =x_size, y_max=y_size 
		grid_layer1 = new bool[size];
//		cout << "x_size :" << s_x << " y_size :" << s_y << " =" << size << endl;
	}
	~Grid()
	{
		delete [] grid_layer0;
		delete [] grid_layer1;
	}
	
	void set(int x, int y, int z, bool w)
	{
		coord = x + y*s_x;
	//	cout << "x : " << x << " y: " << y << " s_x: " << s_x <<" x + y* s_x: " << coord << endl; 
		if (z==0)
		grid_layer0[coord]= w;
		else
		grid_layer1[coord]= w;
	}
	
	int get(int x, int y, int z)
	{
		coord = x + y*s_x;
		if (z==0)
		return grid_layer0[coord];
		else
		return grid_layer1[coord];
	}
	
	int size_y()
	{
		return s_y;
	}
	
	void clear()
	{
		for (int i=0 ; i<size;i++)
		grid_layer0[i] = 0;
		for (int i=0 ; i<size;i++)
		grid_layer1[i] = 0;
	}

private:
	int s_x,s_y;
	bool *grid_layer0;
	bool *grid_layer1;
	long coord, size;
};

//******************************************************
// Node. z=0:metal1+contact z=1: metal2 
//******************************************************
struct Node {
	int x, y, z, h, x_parent,y_parent,z_parent;
	int f, g;
	bool operator==(const Node& node) const;
	bool operator<(const Node& node) const;

	};

bool Node::operator==(const Node& node) const
{

	return (node.f == f);
}
bool Node::operator<(const Node& node) const
{
	return (f > node.f);

}

//********************************************
// get coordinates of points in the new frame
//********************************************
void coord_tran(int x1,int x2, int& X1, int& X2,int& left_end,int& right_end, designRule& myDesignRule)
{
	int x1_,x2_;
	int LMARGIN = myDesignRule.LMARGIN;
	int RMARGIN = myDesignRule.RMARGIN;
		
	if (x1<x2)
	{
		x1_ = x1-LMARGIN;
		x2_ = x2+RMARGIN;
		
		X1 = x1-x1_;
		X2 = x2-x1_;
		left_end= x1_;
		right_end= x2_;		
	}
	else if (x1==x2)
	{
		x1_ = x1-LMARGIN;
		x2_ = x1+RMARGIN;
		X1 = x1-x1_;
		X2 = x2-x1_;
		left_end= x1_;
		right_end= x2_;		
	}
	else if (x1>x2)
	{
		x1_ = x1+RMARGIN;
		x2_ = x2-LMARGIN;
		left_end= x2_;
		right_end= x1_;
		X1 = x1-left_end;
		X2 = x2-left_end;
	}
}
	
//*********************************************************
// coordinate transform for smaller GRID
//*********************************************************
int tranf(int x, int factor)
{
	return (x/factor);
}

int tranb(int x, int factor)
{
	return (x*factor);
}

//*******************************************************
//  blockage checking function for Maze funtion 
//***********************************************************

bool WalkAble(Node& kid, Grid& grid) 
{	
	if (grid.get(kid.x,kid.y,kid.z) == 0 && kid.x >= 0 && kid.y >=0 && kid.x < grid.size_y() && kid.y < grid.size_y())
	return 1;
	else 
	return 0;
}

// *******************************************
// get boundary of cell
// *******************************************	
void cell_bound(oaDesign* design, int& TOP_END, int& RIGHT_END)
{
		oaBlock * topBlock = design->getTopBlock();
		oaLayerHeader * m1LayerHeader;
		m1LayerHeader = oaLayerHeader::find(topBlock, 8);
		
		oaShape * shape;
		oaRect * rect;
		
		oaIter<oaLPPHeader> LPPHeaderIter(m1LayerHeader->getLPPHeaders());
		oaLPPHeader * LPPHeader;
	  while (LPPHeader = LPPHeaderIter.getNext())
	  {
		oaIter<oaShape> shapeIter (LPPHeader->getShapes());

		while (shape = shapeIter.getNext())
		{
		  if (shape->getType()==oacRectType)
		  {
			rect = ((oaRect*)shape);
		  }
		  if (rect->isValid())
		  {
			oaBox box;
			rect->getBBox(box);
			  if(TOP_END<box.bottom())
			  {
				  TOP_END=box.bottom();
				  printf("vdd %d %d %d %d\n",box.left(),box.right(),box.top(),box.bottom());
				TOP_END = box.top();
				RIGHT_END = box.right();
			  }
		  }
		}
	  }
}
//*****************************************
// Special case connect function for misaligned  
// contacts. 
//****************************************
void specialcase(oaDesign* design, int x1, int y1, int x2, int y2, designRule& myDesignRule) // For general use, should implement grid and simple search.
{																   // Also, it should be check that if all points belongs to the same net.
		oaBlock *topBlock = design->getTopBlock();				   // use trial_connect, and pass the list of the nets' contacts too.
	
		oaLayerHeader * layerHeader;
		oaLayerHeader * contactLayerHeader;				
		contactLayerHeader = oaLayerHeader::find(topBlock, 7);
		oaLPPHeader *LPPHeader;
		oaShape * shape;
		oaRect * rect;
		layerHeader = contactLayerHeader;
		
		int VIA_EXT = myDesignRule.VIA_EXT;
		int F = myDesignRule.F;
		int WIDTH = myDesignRule.WIDTH;
		int top =0, bottom =0, left, right, low, high, new_top, new_bottom;
		if (x2>x1)
			left=x1, right=x2;
		else
			left=x2, right=x1;
		if (y2>y1)
			high=y2, low=y1;
		else 
			high=y1, low=y2;	
		
		oaIter<oaLPPHeader> LPPHeaderIter(layerHeader->getLPPHeaders());	
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						oaBox box;
						rect->getBBox(box);
						int temp = (box.top()+box.bottom())/2-(high+low)/2;
						if (box.left()>=left && box.left() <= right && abs(temp) < 3893 ) // reference is left edge (right is also edge)
						{
							if (box.bottom() > high)
							{
								top=1;
								new_top =box.top();
							}					
							else if(box.top() < low)
							{
								bottom=1;
								new_bottom = box.bottom();
							}
							
						} 
					}	
				}		
			}			
	    }
		if (top==0 && bottom==0)
		{
			oaRect::create(topBlock, 8, 1, oaBox(left,low-VIA_EXT,right+650,high+650+VIA_EXT)); 
		}
		else if (top==0 && bottom==1)
		{
			oaRect::create(topBlock, 8, 1, oaBox(left,new_bottom-VIA_EXT,right+650,high+650+VIA_EXT)); 
		}
		else if (top==1 && bottom==0)
		{
			oaRect::create(topBlock, 8, 1, oaBox(left,low-VIA_EXT,right+650,new_top+650+VIA_EXT)); 
		}
		else if (top==1 && bottom==1)
		{
			oaRect::create(topBlock, 8, 1, oaBox(left,new_bottom-VIA_EXT,right+650,new_top+650+VIA_EXT)); 
		}	
}


//************************************************************************************
// append Contact shapes to the grid layer(z=0) except shapes including 
// start point and end point
//************************************************************************************* 
void append_contact(oaDesign* design,int x1, int y1, int z1, int x2, int y2, int z2,Grid & grid, int TOP_END, designRule& myDesignRule)
{
		int F = myDesignRule.F;
		int ADD_SPACING_X = myDesignRule.ADD_SPACING_X;
		int ADD_SPACING_Y = myDesignRule.ADD_SPACING_Y;
		int LMARGIN = myDesignRule.LMARGIN;
		int RMARGIN = myDesignRule.RMARGIN;
		
		oaBlock *topBlock = design->getTopBlock();
	
		oaLayerHeader * layerHeader;
		oaLayerHeader * contactLayerHeader;
				
		contactLayerHeader = oaLayerHeader::find(topBlock, 7);
	
		oaLPPHeader *LPPHeader,*LPPHeader2;
		oaShape * shape;
		oaRect * rect;
		oaPathSeg* pathseg;
		
		layerHeader = contactLayerHeader;
		
		oaIter<oaLPPHeader> LPPHeaderIter(layerHeader->getLPPHeaders());	
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						int left_end, right_end, X1, X2;
						coord_tran(x1,x2,X1,X2, left_end, right_end,myDesignRule);
						oaBox box;
						rect->getBBox(box);
						
						if ((box.left()>left_end && box.right() < right_end) || (box.right()>left_end && box.left() < left_end) || box.left()<right_end && box.right() > right_end)
						{
								int box_left_new, box_right_new;
								if (box.left()>left_end && box.right() < right_end) // whole box is in the coord.
								{
									box_left_new = box.left() - left_end;
									box_right_new = box.right() - left_end;
								}
								else if (box.right()>left_end && box.left() < left_end)  // overlap not included.
								{
									box_left_new = 0;
									box_right_new = box.right() - left_end;
								}	
								else if (box.left()<right_end && box.right() > right_end)
								{
									box_left_new = box.left()-left_end;
									box_right_new = right_end-left_end;
								}
								
								if ((box_left_new > X2+325 || box.top() < y2+325 || box.bottom()> y2+325 || box_right_new < X2+325 || z2 == 1) && (box_left_new > X1+325 || box.top() < y1+325 || box.bottom()> y1+325 || box_right_new < X1+325 || z2 == 1))						
								{
									
									int left = tranf(box_left_new,F)-ADD_SPACING_X;
									if (left<0)
									left =0;
									int bottom = tranf(box.bottom(),F)-ADD_SPACING_Y;
									if (bottom<0)
									bottom =0;
									
									int right = tranf(box_right_new,F)+ADD_SPACING_X; 
									if (right> tranf(right_end-left_end,F))
									right = tranf(right_end-left_end,F);
									
									int top = tranf(box.top(),F)+ADD_SPACING_Y; //drop +1
									if (top > tranf(TOP_END,F))
									top = tranf(TOP_END,F);
									
									int W = right - left;
									int H = top - bottom;
								//	cout << "(block) left bottom right top : " << left*50+left_end << " "<< bottom*50  << " " << right*50+left_end << " " << top*50 << endl;
									for (int i=0; i<W+1 ; i++)
									{
											for (int j=0; j<H+1; j++)
											{
												grid.set(left+i,bottom+j,0,1);
											}
									}
									
								}	
						}
					}	
				}
			}
		}  
}

void append_metal1(oaDesign* design,int x1, int y1, int z1, int x2, int y2, int z2,Grid & grid, int TOP_END, designRule& myDesignRule, oaString name)
{
		int F = myDesignRule.F;
		int ADD_SPACING_X = myDesignRule.ADD_SPACING_X;
		int ADD_SPACING_Y = myDesignRule.ADD_SPACING_Y;
		int LMARGIN = myDesignRule.LMARGIN;
		int RMARGIN = myDesignRule.RMARGIN;
		
		oaBlock *topBlock = design->getTopBlock();
	
		oaLayerHeader * layerHeader;
		oaLayerHeader * m1LayerHeader;
	
		m1LayerHeader = oaLayerHeader::find(topBlock, 8);
	
		oaLPPHeader *LPPHeader;
		oaShape * shape;
		oaRect * rect;
		oaPathSeg* pathseg;
		
		layerHeader = m1LayerHeader;
		oaIter<oaLPPHeader> LPPHeaderIter(layerHeader->getLPPHeaders());
		
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						int left_end, right_end, X1, X2;
						coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);
						oaBox box;
						rect->getBBox(box);
						
						if(box.top()<TOP_END && box.bottom() >=0)
						{
							if ((box.left()>left_end && box.right() < right_end) || (box.right()>left_end && box.left() < left_end) || box.left()<right_end && box.right() > right_end)
							{
								int box_left_new, box_right_new;
								if (box.left()>left_end && box.right() < right_end) // whole box is in the coord.
								{
									box_left_new = box.left() - left_end;
									box_right_new = box.right() - left_end;
								}
								else if (box.right()>left_end && box.left() < left_end)  // overlap not included.
								{
									box_left_new = 0;
									box_right_new = box.right() - left_end;
								}	
								else if (box.left()<right_end && box.right() > right_end)
								{
									box_left_new = box.left()-left_end;
									box_right_new = right_end-left_end;
								}
								bool samenet=0;
								if ( rect->hasNet() ==0 )
									samenet=0;
								else
								{
									oaNet* net = rect->getNet();
									oaName oaname;
									oaString temp;
									net->getName(oaname);
									oaname.get(temp);
									if (temp == name)
									samenet=1;
									else
									samenet=0;
							//		cout << "rect belong to: " << temp << " routing netname: " << name << endl;
								}		
								
								if (samenet ==0)
								{
									
									int left = tranf(box_left_new,F)-ADD_SPACING_X;
									if (left<0)
									left =0;
									int bottom = tranf(box.bottom(),F)-ADD_SPACING_Y;
									if (bottom<0)
									bottom =0;
									
									int right = tranf(box_right_new,F)+ADD_SPACING_X; 
									if (right> tranf(right_end-left_end,F))
									right = tranf(right_end-left_end,F);
									
									int top = tranf(box.top(),F)+ADD_SPACING_Y; //drop +1
									if (top > tranf(TOP_END,F))
									top = tranf(TOP_END,F);
									
									int W = right - left;
									int H = top - bottom;
								//	cout << "(block) left bottom right top : " << left*50+left_end << " "<< bottom*50  << " " << right*50+left_end << " " << top*50 << endl;
									for (int i=0; i<W+1 ; i++)
									{
											for (int j=0; j<H+1; j++)
											{
												grid.set(left+i,bottom+j,0,1);
											}
									}
									
								}	
							}
						}
					}	
				}							
			}
		}  		
	 
}

void append_metal1_2(oaDesign* design,int x1, int y1, int x2, int y2, Grid & grid, int TOP_END, designRule& myDesignRule, oaString name)
{
		int F = myDesignRule.F;
		int ADD_SPACING_X = myDesignRule.ADD_SPACING_X;
		int ADD_SPACING_Y = myDesignRule.ADD_SPACING_Y;
		int LMARGIN = myDesignRule.LMARGIN;
		int RMARGIN = myDesignRule.RMARGIN;
		
		oaBlock *topBlock = design->getTopBlock();
	
		oaLayerHeader * layerHeader;
		oaLayerHeader * m1LayerHeader;
	
		m1LayerHeader = oaLayerHeader::find(topBlock, 8);
	
		oaLPPHeader *LPPHeader;
		oaShape * shape;
		oaRect * rect;
		int left_end, right_end, X1, X2;
		coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);				
		layerHeader = m1LayerHeader;
		oaIter<oaLPPHeader> LPPHeaderIter(layerHeader->getLPPHeaders());
		
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						oaBox box;
						rect->getBBox(box);
						
						if(box.top()<TOP_END && box.bottom() >=0)
						{
							if ((box.left()>left_end && box.right() < right_end) || (box.right()>left_end && box.left() < left_end) || box.left()<right_end && box.right() > right_end)
							{
								int box_left_new, box_right_new;
								if (box.left()>left_end && box.right() < right_end) // whole box is in the coord.
								{
									box_left_new = box.left() - left_end;
									box_right_new = box.right() - left_end;
								}
								else if (box.right()>left_end && box.left() < left_end)  // overlap not included.
								{
									box_left_new = 0;
									box_right_new = box.right() - left_end;
								}	
								else if (box.left()<right_end && box.right() > right_end)
								{
									box_left_new = box.left()-left_end;
									box_right_new = right_end-left_end;
								}
										
								if (box_left_new > X1+325 || box.top() < y1+325 || box.bottom() > y1+325 || box_right_new < X1+325 ) 
										
								{		
								
							//		cout << "(box) left bottom right top : " << box.left() << " "<< box.bottom()  << " " << box.right() << " " << box.top() << endl;
								 
									int left = tranf(box_left_new,F)-ADD_SPACING_X;
									if (left<0)
									left =0;
									int bottom = tranf(box.bottom(),F)-ADD_SPACING_Y;
									if (bottom<0)
									bottom =0;
									
									int right = tranf(box_right_new,F)+ADD_SPACING_X; 
									if (right> tranf(right_end-left_end,F))
									right = tranf(right_end-left_end,F);
									
									int top = tranf(box.top(),F)+ADD_SPACING_Y; //drop +1
									if (top > tranf(TOP_END,F))
									top = tranf(TOP_END,F);
									
									int W = right - left;
									int H = top - bottom;
								//	cout << "(block) left bottom right top : " << left*50+left_end << " "<< bottom*50  << " " << right*50+left_end << " " << top*50 << endl;
									for (int i=0; i<W+1 ; i++)
									{
											for (int j=0; j<H+1; j++)
											{
												grid.set(left+i,bottom+j,0,1);
											}
									}
									
								}	
							}
						}
					}	
				}							
			}
		}  		
	 
}

//************************************************************************************
// append Metal 2 shapes to the grid layer(z=1) except shapes including 
// start point and end point
//*************************************************************************************
void append_metal2(oaDesign* design,int x1, int y1, int z1, int x2, int y2, int z2,Grid & grid,int TOP_END, designRule& myDesignRule, oaString name)
{
		int F = myDesignRule.F;
		int ADD_SPACING_X = myDesignRule.ADD_SPACING_X;
		int ADD_SPACING_Y = myDesignRule.ADD_SPACING_Y;
		int LMARGIN = myDesignRule.LMARGIN;
		int RMARGIN = myDesignRule.RMARGIN;
				
		oaBlock *topBlock = design->getTopBlock();
	
		oaLayerHeader * layerHeader;
		oaLayerHeader * m2LayerHeader;
		
		m2LayerHeader = oaLayerHeader::find(topBlock, 12);
		oaLPPHeader *LPPHeader;
			
		oaShape * shape;
		oaRect * rect;
		oaPathSeg* pathseg;
		layerHeader = m2LayerHeader;
		oaIter<oaLPPHeader> LPPHeaderIter(layerHeader->getLPPHeaders());
		
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						int left_end, right_end, X1, X2;
						coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);
						oaBox box;
						rect->getBBox(box);
						
						if ((box.left()>left_end && box.right() < right_end) || (box.right()>left_end && box.left() < left_end) || box.left()<right_end && box.right() > right_end)
						{
								int box_left_new, box_right_new;
								if (box.left()>left_end && box.right() < right_end) // whole box is in the coord.
								{
									box_left_new = box.left() - left_end;
									box_right_new = box.right() - left_end;
								}
								else if (box.right()>left_end && box.left() < left_end)  // overlap not included.
								{
									box_left_new = 0;
									box_right_new = box.right() - left_end;
								}	
								else if (box.left()<right_end && box.right() > right_end)
								{
									box_left_new = box.left()-left_end;
									box_right_new = right_end-left_end;
								}
								
							
								int samenet=0;
								if ( rect->hasNet() ==0 )
									samenet=0;
								else
								{
									oaNet* net = rect->getNet();
									oaName oaname;
									oaString temp;
									net->getName(oaname);
									oaname.get(temp);
									if (temp == name)
									samenet=1;
									else
									samenet=0;
								//	cout << "rect belong to: " << temp << " routing netname: " << name << endl;
								}
								
								
								if (samenet ==0)
								{					
								
									int left = tranf(box_left_new,F)-ADD_SPACING_X;
									if (left<0)
									left =0;
									int bottom = tranf(box.bottom(),F)-ADD_SPACING_Y;
									if (bottom<0)
									bottom =0;
									
									int right = tranf(box_right_new,F)+ADD_SPACING_X; 
									if (right> tranf(right_end-left_end,F))
									right = tranf(right_end-left_end,F);
									
									int top = tranf(box.top(),F)+ADD_SPACING_Y; //drop +1
									if (top > tranf(TOP_END,F))
									top = tranf(TOP_END,F);
									
									int W = right - left;
									int H = top - bottom;
									for (int i=0; i<W+1 ; i++)
									{
											for (int j=0; j<H+1; j++)
											{
												grid.set(left+i,bottom+j,1,1);
											}
									}
									
								}	
						}
					}						
				}
			}	
		}  		
		
}

//**********************************************************
// A* function. returns a stack of all the points of the path
//**********************************************************
void Maze(int x1, int y1, int z1, int x2, int y2, int z2,Grid& grid, stack<Node>& pts)
{
	//*******************************************
	//  values of two points in coordinate
	// ****************************************
	int x_start= x1, y_start= y1, z_start=z1;     
	int x_goal = x2, y_goal = y2, z_goal=z2;
	// ****************************************
	//    A* code begins here.
	// ****************************************
	bool done =0;
	vector<Node> open, closed;
	std::make_heap (open.begin(),open.end());
	Node s, final; 
	s.f = 0;   
	s.g = 0;   
	s.x = x_start;
	s.y = y_start;
	s.z = z_start;
	open.push_back(s);
	std::push_heap (open.begin(),open.end());

	while(open.size()>0 && done==0 )
	{

			Node q = open.front();
			std::pop_heap(open.begin(),open.end());
			open.pop_back();
			Node kid[3]; 
			
			kid[0].x_parent = q.x;
			kid[0].y_parent = q.y;
			kid[0].z_parent = q.z;
			
			kid[1].x_parent = q.x;
			kid[1].y_parent = q.y;
			kid[1].z_parent = q.z;
			
			kid[2].x_parent = q.x;
			kid[2].y_parent = q.y;
			kid[2].z_parent = q.z;
						
			if (q.z==0)
			{
				kid[0].x = q.x;		//up
				kid[0].y = (q.y) +1;
				kid[0].z = 0;
				
				kid[1].x = q.x; 	//down
				kid[1].y = q.y -1;
				kid[1].z = 0;
			
				kid[2].x = q.x;		//high
				kid[2].y = q.y;
				kid[2].z = 1;
				
			} 		
			else
			{
				kid[0].x = q.x-1;		//left
				kid[0].y = q.y;
				kid[0].z = 1;
				
				kid[1].x = q.x +1; //right
				kid[1].y = q.y;
				kid[1].z = 1;
			
				kid[2].x = q.x;		//low
				kid[2].y = q.y;
				kid[2].z = 0;
			}
			
		for (int i=0; i<3; i++)
		{	
			if (WalkAble(kid[i], grid)) // passing node and m && kid[i].x>=0 && kid[i].y>=0
			{	      
				if (kid[i].x == x_goal && kid[i].y == y_goal && kid[i].z == z_goal)
				 {
					  final = kid[i];
					  done = 1;	  
				 }
				else
				{
					int via_constraint =2;
					kid[i].g = q.g + abs(q.x - kid[i].x) + abs(q.y - kid[i].y) +abs(q.z -kid[i].z)* via_constraint;
					if(z_goal == kid[i].z)
					{
						if (y_goal != kid[i].y || x_goal != kid[i].x)
							kid[i].h = abs(x_goal-kid[i].x) + abs(y_goal-kid[i].y);
						else	
							kid[i].h = abs(x_goal-kid[i].x) + abs(y_goal-kid[i].y) + 2*via_constraint ;
					}
					else
					kid[i].h = abs(x_goal-kid[i].x) + abs(y_goal-kid[i].y) + 1*via_constraint ;			
					kid[i].f = kid[i].g + kid[i].h;
		
					bool open_skip=0, closed_skip=0;
					
					for (int j=0; j< open.size();j++)
					{
						if (open[j].x == kid[i].x && open[j].y == kid[i].y && open[j].z == kid[i].z && open[j].f < kid[i].f)
							open_skip=1;
								
					}
					for (int k=0; k< closed.size();k++)
					{
						if (closed[k].x == kid[i].x && closed[k].y == kid[i].y && closed[k].z == kid[i].z && closed[k].f < kid[i].f)
							closed_skip=1;
					
					}				
					if (!open_skip && !closed_skip)   
					{ 
						open.push_back(kid[i]);
						std::push_heap (open.begin(),open.end());
					}
				}
			}		
		}
		closed.push_back(q);
		
	}	
	closed.push_back(final);
	// *******************************************
	//  Retrace
	// *******************************************
	Node saved = closed.back();
	pts.push(saved);
	int x_p = saved.x, y_p = saved.y, z_p = saved.z;

	for (int i=closed.size() -1 ; i > -1 ; i--)  // gonna try more efficient retrace later
	{
		bool parent_found=0;
		if (x_p == closed[i].x && y_p == closed[i].y && z_p == closed[i].z)
		{
			for(int j=i-1; j>-1 ; j--)
			{
				if(closed[i].x_parent == closed[j].x && closed[i].y_parent == closed[j].y && closed[i].z_parent == closed[j].z && parent_found==0)
				{
					x_p = closed[j].x;
					y_p = closed[j].y;
					z_p = closed[j].z;
					Node temp;
					temp.x = x_p;
					temp.y = y_p;
					temp.z = z_p;
					pts.push(temp);
					parent_found=1;			
				}
			}	
		}			
	}
}

//*************************************************
// Via creation function
//*************************************************
void createVia(int x, int y, oaDesign* design,designRule& myDesignRule)
{
oaBlock * topBlock = design->getTopBlock();
oaRect::create(topBlock, 11, 1, oaBox(x-myDesignRule.viaSize/2,y-myDesignRule.viaSize/2,x+myDesignRule.viaSize/2,y+myDesignRule.viaSize/2)); 
}

//*************************************************
// Draw Function: connect two points with wires
//*************************************************
void Draw(oaDesign *design, stack<Node>& pts, int x1, int y1, int x2, int y2, int& left_end, designRule& myDesignRule, oaString name )																												
{
	int F = myDesignRule.F;
	int WIDTH = myDesignRule.WIDTH;
	int VIA_EXT = myDesignRule.VIA_EXT;
	oaBlock * topBlock = design->getTopBlock();
	vector<Node> vec, points_return;
	vec.push_back(pts.top());
	pts.pop();		
	oaNet* net1;
	oaString netName;
	oaIter<oaNet>   netIterator(topBlock->getNets()); 	
	while (oaNet * net = netIterator.getNext()) 
	{
		net->getName(ns, netName);
		if (netName == name)
		net1 = net;
	}	
	
	//************************************************
	//	Extract only necessary points from the stack
	//*************************************************
	while ( pts.size() > 0)
	{
	
		Node a =pts.top();
		pts.pop();
				
			if (vec.back().z != a.z)
			{
				if ((vec.back().x==a.x && vec.back().y==a.y && vec.back().z==(1-a.z)) != 1)	
					{
					Node temp;
					temp.x=a.x;
					temp.y=a.y;
					temp.z=1-a.z;
					vec.push_back(temp);
					}
					vec.push_back(a);
					
				
			}
		if (pts.size()==0)
		{
			if((a.x == vec.back().x && a.y == vec.back().y && a.z == vec.back().z) !=1)
			{	
					vec.push_back(a);
			}
		}
	}
	//No Offset correction
	/*
	for (int i=0; i< vec.size();i++)
	{
		vec[i].x = tranb(vec[i].x,F) +left_end;
		vec[i].y = tranb(vec[i].y,F);
	}
	*/	
	//	OFFSET correction (beta)
	int x_offset1 = x1- (tranb(vec[0].x,F)+left_end); 
	int y_offset1 = y1- tranb(vec[0].y,F);
	int x_offset2 = x2- (tranb(vec.back().x,F)+left_end); 
	int y_offset2 = y2- tranb(vec.back().y,F);
//	cout << x_offset1 << " " << y_offset1 << " offset1" << endl;	
//	cout << x_offset2 << " " << y_offset2 << " offset2" << endl;
	//*************************************
	// Draw wires with the points obtained above
	// I will modify this so that wire width can vary for contact covering.
	//***************************************
	for (int i=0; i< vec.size();i++)
	{

		if (i<vec.size()-3)
		{
			if (x_offset1 > 0)
			vec[i].x = tranb(vec[i].x,F) + x_offset1 +left_end;
			else 
			vec[i].x = tranb(vec[i].x,F) - x_offset1 +left_end;
			
			if (y_offset1 > 0)
			vec[i].y = tranb(vec[i].y,F) + y_offset1;
			else
			vec[i].y = tranb(vec[i].y,F) - y_offset1;
		}
		
		else if (i==vec.size()-1)
		{
			if (vec[i-2].z != vec[i-1].z && vec[i-2].x == vec[i-1].x && vec[i-2].y == vec[i-1].y )
			{
				if (x_offset2 > 0)
				{
					vec[i-2].x = tranb(vec[i-2].x,F) + x_offset2 +left_end;
					vec[i-1].x = tranb(vec[i-1].x,F) + x_offset2 +left_end;
				}
				else 
				{
					vec[i-2].x = tranb(vec[i-2].x,F) - x_offset2 +left_end;
					vec[i-1].x = tranb(vec[i-1].x,F) - x_offset2 +left_end;
				}
				if (y_offset1 > 0)
				vec[i-1].y = tranb(vec[i-1].y,F) + y_offset1;
				else
				vec[i-1].y = tranb(vec[i-1].y,F) - y_offset1;	
				if (y_offset1 > 0)
				vec[i-2].y = tranb(vec[i-2].y,F) + y_offset1;
				else
				vec[i-2].y = tranb(vec[i-2].y,F) - y_offset1;	
				
				
				if (x_offset2 > 0)
				vec[i].x = tranb(vec[i].x,F) + x_offset2 +left_end;
				else 
				vec[i].x = tranb(vec[i].x,F) - x_offset2 +left_end;
			
				if (y_offset2 > 0)
				vec[i].y = tranb(vec[i].y,F) + y_offset2;
				else
				vec[i].y = tranb(vec[i].y,F) - y_offset2;	
				
			}
			else if (vec[i].z != vec[i-1].z && vec[i].x == vec[i-1].x && vec[i].y == vec[i-1].y )
			{
				if (x_offset2 > 0)
				{
					vec[i-1].x = tranb(vec[i-1].x,F) + x_offset2 +left_end;
					vec[i].x = tranb(vec[i].x,F) + x_offset2 +left_end;
				}
				else 
				{
					vec[i-1].x = tranb(vec[i-1].x,F) - x_offset2 +left_end;
					vec[i].x = tranb(vec[i].x,F) + x_offset2 +left_end;
				}
				if (y_offset1 > 0)
				{
					vec[i-1].y = tranb(vec[i-1].y,F) + y_offset2;
					vec[i-2].y = tranb(vec[i-2].y,F) + y_offset1;
				}
				else
				{
					vec[i-1].y = tranb(vec[i-1].y,F) - y_offset2;
					vec[i-2].y = tranb(vec[i-2].y,F) - y_offset1;					
				}
				if (y_offset2 > 0)
				vec[i].y = tranb(vec[i].y,F) + y_offset2;
				else
				vec[i].y = tranb(vec[i].y,F) - y_offset2;	
				if (x_offset1 > 0)
				vec[i-2].x = tranb(vec[i-2].x,F) + x_offset1 +left_end;
				else 
				vec[i-2].x = tranb(vec[i-2].x,F) - x_offset1 +left_end;
			}
			else
			{
				if (x_offset1 > 0)
				{
					vec[i-1].x = tranb(vec[i-1].x,F) + x_offset1 +left_end;
					vec[i-2].x = tranb(vec[i-2].x,F) + x_offset1 +left_end;
				
				}
				else 
				{
					vec[i-1].x = tranb(vec[i-1].x,F) - x_offset1 +left_end;
					vec[i-2].x = tranb(vec[i-2].x,F) - x_offset1 +left_end;
					
				}
				
				if (y_offset1 > 0)
				{
					vec[i-1].y = tranb(vec[i-1].y,F) + y_offset1;
					vec[i-2].y = tranb(vec[i-2].y,F) + y_offset1;
				
				}
				else
				{
					vec[i-1].y = tranb(vec[i-1].y,F) - y_offset1;
					vec[i-2].y = tranb(vec[i-2].y,F) - y_offset1;
					
				}
				if (x_offset2 > 0)
					vec[i].x = tranb(vec[i].x,F) + x_offset2 +left_end;
				else 
					vec[i].x = tranb(vec[i].x,F) - x_offset2 +left_end;
			
				if (y_offset2 > 0)
					vec[i].y = tranb(vec[i].y,F) + y_offset2;
				else
					vec[i].y = tranb(vec[i].y,F) - y_offset2;	
			}
		}
		
	}

	for (int i=0; i<vec.size()-1;i++)
	{	
		if (vec[i].z==vec[i+1].z)
		{
			int xlow, xhigh, ylow, yhigh;
			if (vec[i].x < vec[i+1].x)
	    	xlow = vec[i].x, xhigh = vec[i+1].x; 
			else
			xhigh = vec[i].x, xlow = vec[i+1].x;
			if (vec[i].y < vec[i+1].y)
	    	ylow = vec[i].y, yhigh = vec[i+1].y; 
			else
			yhigh = vec[i].y, ylow = vec[i+1].y;
			
			if (vec[i].z==0)
			{
				if(i==0)
				{
					oaRect* tmp=oaRect::create(topBlock, 8, 1, oaBox(xlow-WIDTH/2, ylow-325-VIA_EXT, xlow+WIDTH/2, yhigh+325+VIA_EXT));
					tmp->addToNet(net1);
					
				}
				else if(i==vec.size()-2)
				{
					oaRect* tmp1=oaRect::create(topBlock, 8, 1, oaBox(xlow-WIDTH/2, ylow-325-VIA_EXT, xlow+WIDTH/2, yhigh+325+VIA_EXT));
					tmp1->addToNet(net1);
					
				}
				else
				{
					oaRect* tmp2=oaRect::create(topBlock, 8, 1, oaBox(xlow-WIDTH/2, ylow-325-VIA_EXT, xlow+WIDTH/2, yhigh+325+VIA_EXT));
					tmp2->addToNet(net1);
					
				}
			}
			else
			{
				if (myDesignRule.WIDTH == 800 || myDesignRule.VIA_EXT == 350 || myDesignRule.SPACING == 800)
				{
				
					if(myDesignRule.WIDTH == 800 && myDesignRule.SPACING == 550 && myDesignRule.VIA_EXT == 350) 
					{
						oaRect* tmp4=oaRect::create(topBlock, 12, 1, oaBox(xlow-325, ylow-WIDTH/2-VIA_EXT, xhigh+325, yhigh+WIDTH/2+VIA_EXT));
						tmp4->addToNet(net1);
					}
					else if(myDesignRule.WIDTH == 650 && myDesignRule.VIA_EXT == 350)
					{
					oaRect* tmp5=oaRect::create(topBlock, 12, 1, oaBox(xlow-325, ylow-WIDTH/2-VIA_EXT, xhigh+325, yhigh+WIDTH/2+VIA_EXT));				
					tmp5->addToNet(net1);	
					}
					else if(myDesignRule.WIDTH == 800 && myDesignRule.VIA_EXT == 100)
					{
						oaRect* tmp5=oaRect::create(topBlock, 12, 1, oaBox(xlow-325, ylow-WIDTH/2-VIA_EXT, xhigh+325, yhigh+WIDTH/2+VIA_EXT));				
						tmp5->addToNet(net1);
					
					}
					else if(myDesignRule.WIDTH == 650 && myDesignRule.VIA_EXT == 100) 
					{
						oaRect* tmp5=oaRect::create(topBlock, 12, 1, oaBox(xlow-325, ylow-WIDTH/2-VIA_EXT, xhigh+325, yhigh+WIDTH/2+VIA_EXT));				
						tmp5->addToNet(net1);
					}
					else
					{
						oaRect* tmp4=oaRect::create(topBlock, 12, 1, oaBox(xlow-VIA_EXT-325, ylow-WIDTH/2, xhigh+VIA_EXT+325, yhigh+WIDTH/2));
						tmp4->addToNet(net1);
						
					}				
				}
				else
				{
					oaRect* tmp5=oaRect::create(topBlock, 12, 1, oaBox(xlow-325, ylow-WIDTH/2-VIA_EXT, xhigh+325, yhigh+WIDTH/2+VIA_EXT));				
					tmp5->addToNet(net1);				
				}
			}
		}
		else
		{   
			createVia(vec[i].x,vec[i].y, design,myDesignRule);
			if(i==0)		// ********************************************** MIGHT BE MISALIGNED
			{
				if (vec[i].z==0 && vec[i+1].z==1)
				{
					oaRect* tmp6=oaRect::create(topBlock, 8, 1, oaBox(vec[i].x-WIDTH/2, vec[i].y-325-VIA_EXT, vec[i].x+WIDTH/2, vec[i].y+325+VIA_EXT) );
					tmp6->addToNet(net1);
					
				}
			}
			
			if(i==vec.size()-2)
			{
				if (vec[i+1].z==0 && vec[i].z==1)
				{
					oaRect* tmp7=oaRect::create(topBlock, 8, 1, oaBox(vec[i].x-WIDTH/2, vec[i].y-325-VIA_EXT, vec[i].x+WIDTH/2, vec[i].y+325+VIA_EXT) );
					tmp7->addToNet(net1);
					
				}
			}
		}			
	}
}
//************************************************************
// trial_draw
// this function doesn't draw the path, but only returns points 
//************************************************************
void trial_Draw(oaDesign *design, stack<Node>& pts, int x1, int y1, int x2, int y2, int& left_end, vector<Node>& points_return, designRule& myDesignRule )																												
{
	int F = myDesignRule.F;
	int WIDTH = myDesignRule.WIDTH;
	int VIA_EXT = myDesignRule.VIA_EXT;
	oaBlock * topBlock = design->getTopBlock();
	vector<Node> vec;
	vec.push_back(pts.top());
	pts.pop();		
		
	//************************************************
	//	Extract only necessary points from the stack
	//*************************************************
	while ( pts.size() > 0)
	{
	
		Node a =pts.top();
		pts.pop();
				
			if (vec.back().z != a.z)
			{
				if ((vec.back().x==a.x && vec.back().y==a.y && vec.back().z==(1-a.z)) != 1)	
					{
					Node temp;
					temp.x=a.x;
					temp.y=a.y;
					temp.z=1-a.z;
					vec.push_back(temp);
					}
					vec.push_back(a);					
			}
		if (pts.size()==0)
		{
			if((a.x == vec.back().x && a.y == vec.back().y && a.z == vec.back().z) !=1)
			{	
					vec.push_back(a);
			}
		}
	}	
	points_return = vec;
}	
	
	
//*******************************************************
//	trial connect: detect if there's blockage in the path
//******************************************************
bool trial_connect( oaPoint p1, oaPoint p2, oaDesign* design,int TOP_END, designRule& myDesignRule, oaString name)
{
    stack<Node> p;
    vector<Node> getpoints;
	int x1 = p1.x(), y1=p1.y(), z1=0, x2=p2.x(), y2=p2.y(), z2=0;
	int left_end, right_end, X1, X2;
	coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);
	Grid grid(tranf(abs(X2-X1)+myDesignRule.LMARGIN+myDesignRule.RMARGIN,myDesignRule.F),tranf(TOP_END,myDesignRule.F));
	grid.clear();		
	
	
	append_contact(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule);
	append_metal1(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule,name);  
	append_metal2(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule,name );
	if ((grid.get(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),0)))
	{
		cout << "(trial)connect process bypassed. starting point is blocked" << endl;
		return 0;
	}
	else if	((grid.get(tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),0)))
	{
		cout << "(trial)connect process bypassed. ending point is blocked" << endl;
		return 0;
	}
	else
	{
		Maze(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),z1,tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),z2,grid,p);
		trial_Draw(design,p,x1+325,y1+325,x2+325,y2+325,left_end, getpoints, myDesignRule);	
	}
	if (getpoints.size() ==2)
	return 1;
	else
	return 0;	
}
/////////////////////////
//  for area_fix
//
bool trial_connect_2( oaPoint p1, oaPoint p2, oaPoint p3, oaDesign* design,int TOP_END, designRule& myDesignRule, oaString name)
{
		
    stack<Node> p;
    vector<Node> getpoints;
	int x1 = p1.x(), y1=p1.y(), x2=p2.x(), y2=p2.y(), z1=0, z2=0;
	int xcenter = p3.x(), ycenter= p3.y();
	int left_end, right_end, X1, X2;
	coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);
	Grid grid(tranf(abs(X2-X1)+myDesignRule.LMARGIN+myDesignRule.RMARGIN,myDesignRule.F),tranf(TOP_END,myDesignRule.F));
	grid.clear();		
	
//	cout << " x1 y1 x2 y2 " << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
//	cout << " x1 y1 reconverted " << X1+left_end << " " << X2+left_end << endl; 	
	//append_contact(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule);
	append_metal1_2(design, xcenter, ycenter, xcenter+100, ycenter+100, grid, TOP_END, myDesignRule,name);  
	if ((grid.get(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),0)))
	{
		cout << "(trial)connect process bypassed. starting point is blocked" << endl;
		return 0;
	}
	else if	((grid.get(tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),0)))
	{
		cout << "(trial)connect process bypassed. ending point is blocked" << endl;
		return 0;
	}
	else
	{
		Maze(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),z1,tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),z2,grid,p);
		trial_Draw(design,p,x1+325,y1+325,x2+325,y2+325,left_end, getpoints, myDesignRule);	
	}
	//	cout << "leftend" << left_end << endl;
	//	for (int i=0; i<getpoints.size();i++)
	//	cout << getpoints[i].x << " " << getpoints[i].y << " " << getpoints[i].z << endl;
	if (getpoints.size() ==2)
	return 1;
	else
	return 0;	
}
	

//*******************************************************
//		connect
//******************************************************
void connect( oaPoint p1, oaPoint p2, oaDesign* design,int TOP_END, designRule& myDesignRule, oaString name)
{
    stack<Node> p;
    int x1 = p1.x(), y1=p1.y(), z1=0, x2=p2.x(), y2=p2.y(), z2=0;
	int left_end, right_end, X1, X2;
	coord_tran(x1,x2,X1,X2, left_end, right_end, myDesignRule);
	Grid grid(tranf(abs(X2-X1)+myDesignRule.LMARGIN+myDesignRule.RMARGIN,myDesignRule.F),tranf(TOP_END,myDesignRule.F));
	grid.clear();		
	if (abs(x1-x2)< 650 && x1!=x2)
	{
		specialcase(design,x1,y1,x2,y2,myDesignRule);
	}
	else
	{	
		append_contact(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule);
		append_metal1(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule,name);  
		append_metal2(design, x1, y1, z1, x2, y2, z2, grid, TOP_END, myDesignRule,name );
		
		if ((grid.get(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),0)))
			cout << "connect process bypassed. starting point is blocked" << endl;
		else if	((grid.get(tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),0)))
			cout << "connect process bypassed. ending point is blocked" << endl;
		else
		{
		Maze(tranf(X1+325,myDesignRule.F),tranf(y1+325,myDesignRule.F),z1,tranf(X2+325,myDesignRule.F),tranf(y2+325,myDesignRule.F),z2,grid,p);
		Draw(design,p,x1+325,y1+325,x2+325,y2+325,left_end, myDesignRule,name);	
		}
	}
}
//*****************************************
// get Euclidian distance between two points
//*****************************************
int Distance(Node& a, Node& b)
{
	int a_x=a.x, b_x=b.x, a_y=a.y, b_y=b.y;	
	int temp;
	temp = sqrt((a_x/100-b_x/100)*(a_x/100-b_x/100) + (a_y/100-b_y/100)*(a_y/100-b_y/100));
	
  return(temp);
}

void multi_connect(oaPointArray& net_points,oaDesign* design, int TOP_END, designRule& myDesignRule,oaString name)
{
	vector<Node> pathpoints,singlepoints;
	vector<long> x_points, y_points;
	
//	cout << "******For each net ... " << endl;
	for(int i=0; i<net_points.getNumElements(); i++)
	{
		Node temp;
		temp.x=net_points[i].x(), temp.y=net_points[i].y(), temp.z=0;
		singlepoints.push_back(temp);
	}
	vector<Node> priority;
	Node first = singlepoints.front();
	pathpoints.push_back(first);
//	cout << " first :" << first.x << " " << first.y << " " << first.z << endl;
	singlepoints.erase(singlepoints.begin());
	// distance between first and every point in 
	for (int i=0; i<singlepoints.size();i++)
	{
		singlepoints[i].f = Distance(first, singlepoints[i]);
		if (first.x == singlepoints[i].x)
		priority.push_back(singlepoints[i]);
	}
	
	std::sort(singlepoints.begin(),singlepoints.end());
	Node second;
	if(priority.size()>0)
	{
		sort(priority.begin(),priority.end());
		second = priority.back();
		//****************************************************
		//  a blockage on straight path
		//****************************************************
		if (trial_connect(oaPoint(first.x,first.y), oaPoint(second.x, second.y), design,TOP_END, myDesignRule,name))	
		{
			for (int i=0; i<singlepoints.size() ; i++)
				{
					if ( singlepoints[i].x == second.x && singlepoints[i].y ==second.y && singlepoints[i].z == second.z)
						singlepoints.erase(singlepoints.begin()+ i);
				}
		}
		else
		{
			if	(second.x != singlepoints.back().x || second.y != singlepoints.back().y || second.z != singlepoints.back().z )
			{
				second = singlepoints.back();
				singlepoints.pop_back();
			}
			else 
			{
				second = singlepoints[singlepoints.size()-2];
				singlepoints.erase(singlepoints.end()-1);				
			}
		}
		//*************************************************			
	}
	else
	{	
		second = singlepoints.back();
		//*********************** finding deter for a premature turn
		if (first.x != second.x && abs(first.x-second.x) < myDesignRule.WIDTH/2+myDesignRule.SPACING)
		{
		second = singlepoints[singlepoints.size()-2];
		singlepoints.erase(singlepoints.end()-1);
		}		
		// ***********************
		else
		singlepoints.pop_back();
	}
	pathpoints.push_back(second);
	priority.clear();

	connect(oaPoint(first.x,first.y), oaPoint(second.x, second.y), design,TOP_END, myDesignRule,name);
//	cout << " second :" << second.x << " " << second.y << " " << second.z << endl;
	
	while ( singlepoints.size() >0)
	{
		for (int i=0; i<pathpoints.size() ; i++)
		{
			x_points.push_back(pathpoints[i].x);
			y_points.push_back(pathpoints[i].y);
		}
		sort(x_points.begin(),x_points.end());
		sort(y_points.begin(),y_points.end());
		
		Node center;
		Node next;
		Node fourth;
		
		center.x= (x_points.back() + x_points.front())/2;
		center.y= (y_points.back() + y_points.front())/2;
//		cout << center.x << " " << center.y << endl;
		for (int i=0; i<singlepoints.size();i++)
		{
			singlepoints[i].f = Distance(pathpoints.back(), singlepoints[i]);
			if (singlepoints[i].x == pathpoints.back().x)
			priority.push_back(singlepoints[i]);
		}
		if (priority.size() >0)
		{
			sort(priority.begin(),priority.end());
			next = priority.back();
			priority.clear();
				
			for (int i=0; i<pathpoints.size(); i++)
			{
				pathpoints[i].f = Distance(next, pathpoints[i]);
				if (next.x == pathpoints[i].x)
				priority.push_back(pathpoints[i]);				
			}
			sort(priority.begin(),priority.end());
			fourth = priority.back();
			if (trial_connect(oaPoint(next.x,next.y), oaPoint(fourth.x, fourth.y), design,TOP_END, myDesignRule,name)==1 
			|| singlepoints.size()<=2)
			{
				for (int i=0; i<singlepoints.size() ; i++)
				{
					if ( singlepoints[i].x == next.x && singlepoints[i].y ==next.y && singlepoints[i].z == next.z)
						singlepoints.erase(singlepoints.begin()+ i);
				}
				priority.clear();   // we keep elements in pathpoints
			}
			else	// if the straight path is blocked find another point, 
					// more strictly new coordinate has different x, tentatively implemented
			{ 
				for (int i=0; i<singlepoints.size();i++)
				{
					singlepoints[i].f = Distance(center, singlepoints[i]);
				
				}
				sort(singlepoints.begin(),singlepoints.end());
				if (singlepoints.back().x != priority.back().x)
				{
					next = singlepoints.back();
					singlepoints.pop_back();
				}
				else
				{
					next = singlepoints[singlepoints.size()-2];
					singlepoints.erase(singlepoints.end()-1);
				}
					
				for (int i=0; i<pathpoints.size(); i++)
				{
					pathpoints[i].f = Distance(next, pathpoints[i]);
				}
					sort(pathpoints.begin(),pathpoints.end());
					fourth=pathpoints.back();
			}
		}
		else
		{
		
			for (int i=0; i<singlepoints.size();i++)
			{
				singlepoints[i].f = Distance(center, singlepoints[i]);
			
			}
			sort(singlepoints.begin(),singlepoints.end());
			
			next = singlepoints.back();
			
			singlepoints.pop_back();
			for (int i=0; i<pathpoints.size(); i++)
			{
				pathpoints[i].f = Distance(next, pathpoints[i]);
			}
			sort(pathpoints.begin(),pathpoints.end());
			fourth=pathpoints.back();
			if (next.x != fourth.x && abs(next.x-fourth.x) < myDesignRule.WIDTH/2+myDesignRule.SPACING)
			{
					fourth = pathpoints[pathpoints.size()-2];
			}
		
		}
		connect( oaPoint( fourth.x,fourth.y ), oaPoint(next.x, next.y), design,TOP_END, myDesignRule,name);		
//		cout << " third (from) :" << fourth.x << " " << fourth.y << " " << endl; 
//		cout << " third (to) :" << next.x << " " << next.y << " " << endl; 
		pathpoints.push_back(next);	
	}	

}	
					
void area_fix(oaDesign* design, int TOP_END, designRule& myDesignRule)
{
		oaBlock *topBlock = design->getTopBlock();
	
		oaLayerHeader * m1LayerHeader;
		m1LayerHeader = oaLayerHeader::find(topBlock, 8);
	
		oaLPPHeader *LPPHeader;
		oaShape * shape;
		oaRect * rect;
	
		oaIter<oaLPPHeader> LPPHeaderIter(m1LayerHeader->getLPPHeaders());
		
		while (LPPHeader = LPPHeaderIter.getNext())
		{	
			oaIter<oaShape> shapeIter (LPPHeader->getShapes());
			while (shape = shapeIter.getNext())
			{
				if (shape->getType()==oacRectType)
				{
					rect = ((oaRect*)shape);
					if (rect->isValid())
					{
						
						oaBox box;
						rect->getBBox(box);
						if ( (box.right()- box.left())* (box.top() - box.bottom()) < myDesignRule.minArea )
						{
							oaString name;
							cout << "Detected a metal1< minArea at :" << box.left()+325 << " " << box.bottom()+325 << endl;
							int height_need = myDesignRule.minArea / (box.right() -box.left());							
							if (trial_connect_2( oaPoint(box.left(), box.bottom()+325), oaPoint( box.left(), height_need+box.bottom()-650),
								oaPoint(box.left(),box.bottom()),design, TOP_END, myDesignRule, name)==1 )
							{
								oaRect* tmp= oaRect::create(topBlock, 8, 1, oaBox(box.left(), box.bottom() , box.right() , box.bottom()+height_need));
								cout << "a metal1 rect has been successfully expended" << endl;								
							}	
							else if	(trial_connect_2( oaPoint( box.left(), box.bottom()-325), oaPoint( box.left(), box.top()-height_need),
							  oaPoint(box.left(),box.bottom()),design, TOP_END, myDesignRule, name) ==1 )
								
							{
								oaRect* tmp= oaRect::create(topBlock, 8, 1, oaBox(box.left(), box.top()-height_need , box.right() , box.top()));
								cout << "a metal1 rect has been successfully expended" << endl;								
							}
							else
								cout << "no space to expand metal1 " << endl;
						}
					}
				}
			}
		}
}
// ===  FUNCTION  ======================================================================
//         Name:  cover_contact
// =====================================================================================
    void
cover_contact ( oaBlock * topBlock,  oaPoint p1, int metal, designRule& myDesignRule ){
    int halfw = myDesignRule.WIDTH/2;
    oaPoint p_aligned = oaPoint(p1.x()+325, p1.y()+325);
    oaRect::create(topBlock, metal, 1, oaBox(p_aligned.x()-halfw,p_aligned.y()-325-myDesignRule.VIA_EXT,
                   p_aligned.x()+halfw,p_aligned.y()+325+myDesignRule.VIA_EXT)); 
}		// -----  end of function cover_contact  ----- 



 
std::string make_string(const std::string& a_prefix,
                        size_t a_suffix,
                        size_t a_max_length)
{
    std::ostringstream result;
    result << a_prefix <<
              std::setfill('0') <<
              std::setw(a_max_length - a_prefix.length()) <<
              a_suffix;
    return result.str();
}


//-----------------------------------------------------------------------------
//       MAIN         
//-----------------------------------------------------------------------------
int main(int argc,char* argv[]) {
    //-----------------------------------------------------------------------------
    //  import design rule information 
    //-----------------------------------------------------------------------------
    string token2;
    string line2;
    int count2=0;
    ifstream dRuleFile(argv[4]);
    designRule myDesignRule;
    bool oneTime = true;
    if (dRuleFile.is_open()){
        while ( getline (dRuleFile,line2)){
            if (oneTime){
                std::istringstream iss1(line2);
                while (iss1 >> token2){
                    switch (count2){
                        case 0:
                            myDesignRule.WIDTH = atoi((char*)token2.c_str());
                            count2++;
                            break;
                        case 1:
                            myDesignRule.SPACING = atoi((char*)token2.c_str());
                            count2++;
                            break;
                        case 2:
                            myDesignRule.VIA_EXT = atoi((char*)token2.c_str());
                            count2++;
                            break;
                        case 3:
                            myDesignRule.minArea = atoi((char*)token2.c_str());
                            count2++;
                            break;
                        case 4:
                            myDesignRule.viaSize = atoi((char*)token2.c_str());
                            count2++;
                            break;
                    }
                }
                oneTime = false;
            }
        }
    }
    dRuleFile.close();
    //Print out design rule spec
    cout << "minWidth: " << myDesignRule.WIDTH << endl;
    cout << "minSpacing: " << myDesignRule.SPACING << endl;
    cout << "minViaExt: " << myDesignRule.VIA_EXT << endl;
    cout << "minArea: " << myDesignRule.minArea << endl;
    cout << "viaSize: " << myDesignRule.viaSize << endl; // ViaSize is fixed for this project
    cout << "\n\n\n";
	myDesignRule.WIDTH = myDesignRule.WIDTH*10 ;
    myDesignRule.SPACING = 10*myDesignRule.SPACING;
    myDesignRule.VIA_EXT = 10*myDesignRule.VIA_EXT;
    myDesignRule.minArea = 100*myDesignRule.minArea;
    myDesignRule.viaSize = 10*myDesignRule.viaSize;

    //-----------------------------------------------------------------------------
    // Import txt file in netInfo array
    //-----------------------------------------------------------------------------
    std::vector<netInfo> nets; 
    netInfo net;
    string line; 
    string token;
    char* temp;
    //char *tokenTemp;// = (char*)token.c_str();
    oaUInt4 count=0;
    oaInt4 x, y; 
    oaUInt4 numOfLines=0;
    oaBox bBox;
    int a = 0;

    ifstream file0(argv[3]);
    if (file0.is_open()){
        ///read text file 
        while ( getline (file0,line) ){
            numOfLines++;
        }
        file0.close();
    } else {
        cerr << "FAILED to open file" << endl; 
        exit(1); 
    }

    ifstream myfile(argv[3]);
    netInfo tempNets[numOfLines];
    cout << "we have " << numOfLines << " nets in this design" << endl;
    cout << "***********************************" << endl;

    if (myfile.is_open()){
        ///read text file 
        cout << "reading " << argv[3] << endl;
        int j = 0;
        while ( getline (myfile,line) ){
            //cout << line << endl;
            std::istringstream iss(line);
            while(iss >> token) { 
                if (j >=numOfLines){
                    cerr << "error Bitch! segmentation foleta" << endl;
                }
                if (isNumber(token)){
                    switch (count){
                        case 0:
                            x = atoi((char*)token.c_str());
                            count++;
                            break;
                        case 1:
                            y = atoi((char*)token.c_str());
                            tempNets[j].points.append(oaPoint(x ,y));
                            count = 0; 
                            break; 
                    }
                } else {
                    temp = (char*)token.c_str();
                    if (token.compare(0,2,"IO")==0){
                        cout << "FOUND IO\n";
                        tempNets[j].netType = "IO";
                        tempNets[j].netName.clear();
                        tempNets[j].netName.insert(0,token,3,token.size());
                    } else {
                        tempNets[j].netType = (char*)token.c_str();
                        tempNets[j].netName = make_string("net",a++,6) ;
                    }
                    tempNets[j].points.getBBox(bBox);
                    tempNets[j].hpwl = bBox.getHeight() + bBox.getWidth();
                    nets.push_back(tempNets[j]);
                }
            }
            j++;
        }
        myfile.close();
    } else {
        cerr << "FAILED to open file" << endl; 
        exit(1); 
    }
    // initialize isRouted
    for (int i=0; i<nets.size(); i++){
        nets[i].isRouted = false; 
    }

    //-----------------------------------------------------------------------------
    //  List content of the txt file 
    //  AND SORT 
    //  to double check that the data is imported correctly,
    //-----------------------------------------------------------------------------
    if (true) { //argc == 4
        cout << "CONTENT BEFORE SORTING" << endl;
        listContents(nets);
        std::sort(nets.begin(), nets.end(), &netSorter);
        cout << "\n\nCONTENT AFTER SORTING" << endl;
        listContents(nets);
        for (int i=0; i<nets.size();i++){
            nets[i].points.sort(pointSorter);
        }
        cout << "\n\nCONTENT AFTER points SORTING" << endl;
        for (int i=0; i<nets.size();i++){
            nets[i].points.sort(pointSorter);
        }
        listContents(nets);
        //-----------------------------------------------------------------------------
        //  sort points correctly 
        //-----------------------------------------------------------------------------
        oaPointArray tempA;
        int min;
        oaPoint temp;
        for (int i=0; i<nets.size(); i++){   
            if (nets[i].points.getNumElements()>2){
               tempA= nets[i].points;
        //-----------------------------------------------------------------------------
               for (int j=0;j<tempA.getNumElements()-1;j++){ 
                    min = hpwl(tempA[j],tempA[j+1]);
                    for (int p=j+2; p<tempA.getNumElements(); p++){
                        if (hpwl(tempA[j],tempA[p]) <= min){
                            min = hpwl(tempA[j],tempA[p]);
                            std::swap(tempA[p],tempA[j+1]);
                        }
                    }
               }
               nets[i].points = tempA;
            }
        //-----------------------------------------------------------------------------
        }
        cout << "\n\nCONTENT AFTER very very points SORTING" << endl;
        listContents(nets);

    } else {
        cerr << "WRONG INPUT ARGUMENTS" << endl;
        exit(1);
    }
    cout << "\n\n\n\n"  << endl;
    try 
    {
    oaDesignInit(oacAPIMajorRevNumber, oacAPIMinorRevNumber, 3);

    oaNativeNS oaNs;
    oaString libraryPath("./DesignLib");
    oaString library("DesignLib");
    oaString top_cell(argv[1]);
    oaString new_top_cell(argv[2]);
    oaString layout_view("layout");
    oaScalarName libraryName(oaNs,library);
    oaScalarName cellName(oaNs,top_cell);
    oaScalarName newCellName(oaNs,new_top_cell);
    oaScalarName layoutView(oaNs,layout_view);


    // open the libs defined in "lib.def"
    oaLibDefList::openLibs();

    // locate the library
    oaLib *lib = oaLib::find(libraryName);
    
    if (!lib)
    {
        if (oaLib::exists(libraryPath))
        {
        lib = oaLib::open(libraryName, libraryPath);
        }
        else
        {
        lib = oaLib::create(libraryName, libraryPath);
        }
        if (lib)
        {
        //update the lib def list
        oaLibDefList *list = oaLibDefList::getTopList();
        if (list) 
        {
            oaString topListPath;
            list->getPath(topListPath);
            list->get(topListPath,'a');
            oaLibDef *newLibDef = oaLibDef::create(list, libraryName, libraryPath);
            list->save();
        }
        }
        else
        {
        cerr << "Error : Unable to create " << libraryPath << "/";
        cerr << library << endl;
        return 1;
        }
    }

    // open the design now
    oaDesign *design=oaDesign::open(libraryName, cellName, layoutView, 'r');
    // make a copy copy and save as a new design here
    design->saveAs(libraryName, newCellName, layoutView);
    oaScalarName name_buffer;
    oaString string_buffer;
    design->getLibName(name_buffer);
    name_buffer.get(oaNs,string_buffer);
    cout << "The library name for this design is : " << string_buffer << endl;

    design->getCellName(name_buffer);
    name_buffer.get(oaNs,string_buffer);
    cout << "The cell name for this design is : " << string_buffer << endl;

    design->getViewName(name_buffer);
    name_buffer.get(oaNs,string_buffer);
    cout << "The view name for this design is : " << string_buffer << endl;
    // operate the oaBlock
    oaBlock * topBlock = design->getTopBlock();
    // open the tech for new layer creating
    oaTech * tech = oaTech::open(lib, 'a');
    oaLayer * layer;

    // check if via1 layer is in the database
    layer =  oaLayer::find(tech, "via1");
    if (layer==NULL) {
        cout << "Creating via1 layer\n";
        oaPhysicalLayer::create(tech, "via1", 11, oacMetalMaterial, 11);
    }

    // check if metal2 layer is in the database
    layer =  oaLayer::find(tech, "metal2");
    if (layer==NULL) {
        cout << "Creating metal2 layer\n";
        oaPhysicalLayer::create(tech, "metal2", 12, oacMetalMaterial, 12);
    }

    cout << "** FINSIH IMPORT \n\n\n\n" << endl;
	
	myDesignRule.F = 50;
	if (myDesignRule.WIDTH == 800 || myDesignRule.VIA_EXT == 350 || myDesignRule.SPACING == 800)
	{
		if(myDesignRule.WIDTH == 800 && myDesignRule.SPACING == 550 && myDesignRule.VIA_EXT == 350) 
		{
		myDesignRule.ADD_SPACING_X = 15;
		myDesignRule.ADD_SPACING_Y = 15;
		}
		else if(myDesignRule.WIDTH == 650 && myDesignRule.VIA_EXT == 350)
		{
		myDesignRule.ADD_SPACING_X = 15;
		myDesignRule.ADD_SPACING_Y = 15;
		
		}
		else if(myDesignRule.WIDTH == 800 && myDesignRule.VIA_EXT == 100)
		{
		myDesignRule.ADD_SPACING_X = 15;
		myDesignRule.ADD_SPACING_Y = 15;
		}
		else if(myDesignRule.WIDTH == 650 && myDesignRule.VIA_EXT == 100) 
		{
		myDesignRule.ADD_SPACING_X = 16;
		myDesignRule.ADD_SPACING_Y = 18;
	    }
		else
		{
		myDesignRule.ADD_SPACING_X = 15;
		myDesignRule.ADD_SPACING_Y = 14;
		}
	}
	else
	{
		myDesignRule.ADD_SPACING_X = 16;
		myDesignRule.ADD_SPACING_Y = 18;
	}
	myDesignRule.LMARGIN = 2000;
	myDesignRule.RMARGIN = 3000;
	
    // get boundary of cell to define a grid
    int TOP_END, RIGHT_END;
    cell_bound(design, TOP_END, RIGHT_END);
	// dummy metal 2 object for LPPHeader
	oaRect *temp= oaRect::create(topBlock, 12, 1, oaBox(50,50,100,100)); 

    //-----------------------------------------------------------------------------
    //  route nets
    //-----------------------------------------------------------------------------

        
    //-----------------------------------------------------------------------------
    //  1st cover contact: 
    //-----------------------------------------------------------------------------
    for (int i=0; i<nets.size(); i++){   
        if (!nets[i].isRouted && nets[i].points.getNumElements()==1){
            // cover contacts 
            cover_contact(topBlock, nets[i].points[0], 8, myDesignRule);
        }
    }

    //-----------------------------------------------------------------------------
    //  Display IO names on contacts for IO type nets 
    //-----------------------------------------------------------------------------
    for (int i=0; i<nets.size(); i++){
        if (nets[i].netType.compare("IO") == 0){
            cout << "we have an IO net " << nets[i].netType << endl;
            cout << "\t IO's name: " << nets[i].netName << endl;
            oaText::create(topBlock,
                           8, 
                           1,
                           oaString(nets[i].netName.c_str()),
                           oaPoint(nets[i].points[0].x()+325, nets[i].points[0].y()+325),
                           oaTextAlign::oaTextAlign (oacCenterLeftTextAlign),
                           oaOrient::oaOrient(oacR0),
                           oaFont::oaFont(oacRomanFont),
                           325, //oaDist(1000)
                           false,
                           true,
                           true);
        }
    }

    //-----------------------------------------------------------------------------
    //  Route VDD and VSS
    //-----------------------------------------------------------------------------
    vector<netInfo> VDDNets; 
    vector<netInfo> VSSNets; 
    VDDNets.clear();
    VSSNets.clear();

    stack<Node> powerPath;
    for (int i=0; i<nets.size(); i++){
        if (nets[i].netType.compare("VDD") == 0){
            nets[i].isRouted = true;
            VDDNets.push_back(nets[i]);
            cout << "VDD added\n";
        }
        else if (nets[i].netType.compare("VSS") == 0){
            nets[i].isRouted = true;
            VSSNets.push_back(nets[i]);
            cout << "VSS added\n";
        }
    }
    // Route VDDNets and VSSNets 
    for (int i=0; i<VDDNets.size(); i++){
        for (int j=0; j<VDDNets[i].points.getNumElements(); j++){
            oaRect::create(topBlock, 8, 1, oaBox(VDDNets[i].points[j].x()+325-myDesignRule.WIDTH/2, VDDNets[i].points[j].y()-myDesignRule.VIA_EXT,
                        VDDNets[i].points[j].x()+325+myDesignRule.WIDTH/2,VDD_TOP)); 
            while(!powerPath.empty()){
                cout<<"not empty\n";
                powerPath.pop();
            }
        }
    }
    for (int i=0; i<VSSNets.size(); i++){
        for (int j=0; j<VSSNets[i].points.getNumElements(); j++){
            oaRect::create(topBlock, 8, 1, oaBox(VSSNets[i].points[j].x()+325-myDesignRule.WIDTH/2, VSS_BOTTOM,
                        VSSNets[i].points[j].x()+325+myDesignRule.WIDTH/2, VSSNets[i].points[j].y()+650+myDesignRule.VIA_EXT)); 
            while(!powerPath.empty()){
                cout<<"not empty\n";
                powerPath.pop();
            }
        }
    }


    //-----------------------------------------------------------------------------
    //  route the rest of the nets that have at least two points 
    //-----------------------------------------------------------------------------
    for (int i=0; i<nets.size(); i++){   
        //route untoured nets that have at least two points
        if (!nets[i].isRouted && nets[i].points.getNumElements()>1){
            if (nets[i].points.getNumElements() <3)
			{	
				oaString temp = oaString(nets[i].netName.c_str());	
				oaNet::create(topBlock, oaName(ns, temp) );	
			//	cout << " Name sent(twopinnet): " << temp << endl;
				
				for (int j=0; j<nets[i].points.getNumElements()-1; j++)
				{
				
					
					connect(nets[i].points[j], nets[i].points[j+1],
                        design, TOP_END, myDesignRule, temp);
				
				}
			}
			else
			{
				oaString temp = oaString(nets[i].netName.c_str());	
			//	cout << " Name sent(multinet): " << temp << endl;
				oaNet::create(topBlock, oaName(ns, temp) );	
				multi_connect(nets[i].points,design,TOP_END, myDesignRule, temp);
			}
            nets[i].isRouted = true;
        }
    }

	cout << "******************" << endl;
	cout << " Routing is done! " << endl;	
	cout << "******************" << endl;
// **************************
//  minArea, only run when min area = 780000 & WIDTH = 650
// ****************************	
	if (myDesignRule.minArea == 780000 && myDesignRule.WIDTH == 650)
	{
		cout << "Checking minimum area" << endl;
		area_fix(design,TOP_END, myDesignRule);
	}
	
	
	

	// dummy object deleted.
	temp->destroy();
/* *********************************** */

 //***********************************************************************************
  // save the design
  design->saveAs(libraryName, newCellName, layoutView);
  // save the tech with new created layers
  tech->save();
  design->close();

  }
 catch (oaException &excp)
 {
  cout << "ERROR :" << excp.getMsg() << endl;
  exit(1);
 }
 return 0;
}