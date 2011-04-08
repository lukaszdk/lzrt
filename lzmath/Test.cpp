#include <iostream>

#include <lzmath.h>

using namespace std;
using namespace lzmath;


int main(int argc, char* argv[])
{
	cout << "lzMath Test" << endl;

	Vector3f v1 = Vector3f(0,1,2);
	Vector3f v2 = Vector3f(1.1, 2.2, 3.3);

	cout << v1 << endl;
	cout << v1 + v2 << endl;
	cout << v1 * 2 << endl;
	cout << v2 / 2 << endl;
	cout << -v1 << endl;
	cout << "Dot: " << Dot(v1, v2) << endl;
	cout << "Cross: " << Cross(v1, v2) << endl;
	

	return 0;
}
