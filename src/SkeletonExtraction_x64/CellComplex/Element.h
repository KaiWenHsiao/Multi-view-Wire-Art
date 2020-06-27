#define ISEXIST(signs) ((signs)&0x1)
#define SETEXIST(signs) ((signs) = ((signs) | 0x1))
#define CLEAREXIST(signs) ((signs) = ((signs) & 0xfe))

#define ISLOCKED(signs) (((signs)& 0x2)>>1)
#define SETLOCKED(signs) ((signs) = ((signs) | 0x2))
#define CLEARLOCKED(signs ) ((signs) = ((signs) & 0xfd))

#define ISISOSET(signs) (((signs)& 0x4)>>2)
#define SETISOSET(signs) ((signs) = ((signs) | 0x4))
#define CLEARISOSET(signs ) ((signs) = ((signs) & 0xfb))

#define GETTAGGED2(signs) (((signs)& 0x18) >> 3)
#define SETTAGGED2(signs,tag2) ((signs) = ((signs) | ((tag2) << 3)))
#define CLEARTAGGED2(signs ) ((signs) = ((signs) & 0xe7))

#define GETPARENT(signs) (((signs)& 0xe0) >> 5)
#define SETPARENT(signs,parInd) ((signs) = ((signs) | (( parInd ) << 5)))
#define CLEARPARENT(signs) ((signs) = ((signs) & 0x1f))

#define GETCHILD2(signs) (((signs)& 0xe0) >> 5)
#define SETCHILD2(signs,parInd) ((signs) = (((signs) & 0x1f) | (( parInd ) << 5)))
#define CLEARCHILD2(signs) ((signs) = ((signs) & 0x1f))

#define ISTAGGED(signs) (((signs)& 0x8)>>3)
#define SETTAGGED(signs) ((signs) = ((signs) | 0x8))
#define CLEARTAGGED(signs ) ((signs) = ((signs) & 0xf7))

#define GETCHILD(signs) (((signs)& 0x70)>>4)
#define SETCHILD(signs, childID) ((signs) = (((signs) & 0x0f) | ((childID)<<4) ))

#define ITERNUM(signs) (((signs)&0xc)>>2)
#define SETITERNUM(signs,iter) (signs = (((signs) & 0xf3) | ((iter) << 2)))
#define CLEARITERNUM(signs) (signs = ((signs) & 0xf3))

#define SETUNREMOVABLE(signs) ((signs) = ((signs) | 0x10))
#define CLEARUNREMOVABLE(signs) ((signs) = ((signs) & 0xef))

#define GETUNREMOVABLE(signs) (((signs)>>4)&0x1)
#define GETRELOCK(signs) (((signs)>>5)&0x1)
#define SETRELOCK(signs) ((signs) = ((signs) | 0x20))

#define BYTE unsigned char 

class Element
{
public:

	bool isExist;
	float speed;
	float distance;

	Element() {}
	~Element() {}

	BYTE isoNum;
	BYTE signs;

	virtual int* getChildren() = 0;
	virtual int* getParents() = 0;
	virtual int getChildrenNum() = 0;
	virtual int getParentsNum() = 0;
	virtual void setType(int type, int typeVal) = 0;
	virtual void getType(int& type, int& typeVal) = 0;
};

class Point : public Element
{
public:

	int parents[6];
	int type;

	Point()
	{
		speed = 0;
		isoNum = 255;
		signs = 0;
		distance = 0;
		type = 0;

		isExist = true;
		for (int i = 0; i < 6; i++)
		{
			parents[i] = -1;
		}
	}

	~Point() {}

	int* getChildren() { return NULL; }
	int* getParents() { return parents; }
	int getChildrenNum() { return 0; }
	int getParentsNum() { return 6; }
	void setType(int type, int typeVal) {
		this->type = (typeVal << 5) + type;
	}
	void getType(int& type, int& typeVal)
	{
		type = (this->type & 0x1f);
		typeVal = this->type >> 5;
	}
};

class Edge : public Element
{
public:

	int children[2];
	int parents[4];
	BYTE bestRadius;
	int type;

	Edge()
	{
		isExist = true;
		speed = 0;
		isoNum = 255;
		signs = 0;
		bestRadius = 0;
		distance = 0;
		type = 0;

		for (int i = 0; i < 2; i++)
		{
			children[i] = -1;
		}
		for (int i = 0; i < 4; i++)
		{
			parents[i] = -1;
		}
	}

	~Edge() {}

	int* getChildren() { return children; }
	int* getParents() { return parents; }
	int getChildrenNum() { return 2; }
	int getParentsNum() { return 4; }
	void setType(int type, int typeVal) {
		this->type = (typeVal << 5) + type;
	}
	void getType(int& type, int& typeVal)
	{
		type = (this->type & 0x1f);
		typeVal = this->type >> 5;
	}
};

class Face : public Element
{
public:

	int children[4];
	int parents[2];
	BYTE bestRadius;
	int type;

	Face()
	{
		isExist = true;
		speed = 0;
		isoNum = 255;
		signs = 0;
		bestRadius = 0;
		distance = 0;

		for (int i = 0; i < 4; i++)
		{
			children[i] = -1;
		}
		for (int i = 0; i < 2; i++)
		{
			parents[i] = -1;
		}
		type = 0;
	}

	~Face(){}

	int* getChildren() { return children; }
	int* getParents() { return parents; }
	int getChildrenNum() { return 4; }
	int getParentsNum() { return 2; }
	void setType(int type, int typeVal) {
		this->type = (typeVal << 5) + type;
	}
	void getType(int& type, int& typeVal)
	{
		type = (this->type & 0x1f);
		typeVal = this->type >> 5;
	}
};

class Cell : public Element
{
public:

	int children[6];

	Cell()
	{
		isExist = true;
		speed = 0;
		isoNum = 255;
		signs = 0;
		distance = 0;

		for (int i = 0; i < 6; i++)
		{
			children[i] = -1;
		}
	}

	~Cell() {}

	int* getChildren() { return children; }
	int* getParents() { return NULL; }
	int getChildrenNum() { return 6; }
	int getParentsNum() { return 0; }
	void setType(int type, int typeVal) { }
	void getType(int& type, int& typeVal) {}
};
