/* \author Aaron Brown */
// Quiz on implementing kd tree

// Structure to represent node attributs
struct Point
{
	std::vector<float> coords;
	int id;

	Point(std::vector<float> arr, int setId)
	: coords(arr), id(setId)
	{}

	~Point() {}
};

// Structure to represent node of kd tree
struct Node
{
	Point point;
	Node* left;
	Node* right;

	Node(Point point, Node* left, Node* right)
	:	point(point), left(left), right(right)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree3D
{
	Node* root;

	KdTree3D(std::vector<Point>* point_arr)
	{
	    std::vector<Point>::iterator begin = point_arr->begin();
		std::vector<Point>::iterator end = point_arr->end();

	    int size = point_arr->size();
	    int depth = 0;
	    root = generate_tree(begin, end, size, depth);
	}

	~KdTree3D()
	{
		delete root;
	}

	struct compareLess {
	    compareLess(int dimension) { this->dimension = dimension; }
	    bool operator () (Point pt1, Point pt2) 
	    { return (pt1.coords[dimension] < pt2.coords[dimension]); }

	    int dimension;
	};

    void sortLess(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int dimension){
        std::sort(begin, end, compareLess(dimension));
    }

	Node* generate_tree(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int size, int depth)
	{
		if (begin == end) {
			return NULL;
		}

		if (size > 1) {
			sortLess(begin, end, depth);
		}

		std::vector<Point>::iterator middle, l_begin, l_end, r_begin, r_end;

		middle = begin + (size / 2);
		l_begin = begin;
		l_end = middle;
		r_begin = middle + 1;
		r_end = end;

		int l_len = size / 2;
		int r_len = size - l_len - 1;

		int dim = begin->coords.size();

		Node* left;
		if (l_len > 0 && dim > 0) {
			left = generate_tree(l_begin, l_end, l_len, (depth + 1) % dim);
		} else {
			left = NULL;
		}
		Node* right;
		if (r_len > 0 && dim > 0) {
			right = generate_tree(r_begin, r_end, r_len, (depth + 1) % dim);
		} else {
			right = NULL;
		}
		
		return new Node( *middle, left, right);

	}

	void searchHelper(std::vector<int>* ids, Node*& node, std::vector<float> target, float distanceTol, uint depth) {

		if (node != NULL){
			float mxd = target[0] - distanceTol;
			float pxd = target[0] + distanceTol;
			float myd = target[1] - distanceTol;
			float pyd = target[1] + distanceTol;
			float mzd = target[2] - distanceTol;
			float pzd = target[2] + distanceTol;
			
			if (( mxd <= node->point.coords[0] && node->point.coords[0] <= pxd ) 
				&& ( myd <= node->point.coords[1] && node->point.coords[1] <= pyd )
				&& ( mzd <= node->point.coords[2] && node->point.coords[2] <= pzd )) {
					float distance = std::sqrt(pow(node->point.coords[0] - target[0], 2) 
										+ pow(node->point.coords[1] - target[1], 2)
										+ pow(node->point.coords[2] - target[2], 2));
					if (distance <= distanceTol) {
						ids->push_back(node->point.id);	
					}
			}

			uint index = depth % 3;

			if ( ( target[index] - distanceTol ) < node->point.coords[ index ] ) {
				searchHelper( ids, node->left, target, distanceTol, depth + 1 );
			}

			if ( ( target[index] + distanceTol ) > node->point.coords[ index ]) {
				searchHelper( ids, node->right, target, distanceTol, depth + 1 );
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	template<typename PointT>
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<float> pt = {target.x, target.y, target.z};
		searchHelper(&ids, root, pt, distanceTol, 0);
		return ids;
	}
};