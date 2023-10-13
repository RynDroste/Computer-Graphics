#include "tgaimage.h"
#include <iostream>
using namespace std;
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);


void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
	bool steep = false; // 标记斜率绝对值是否大于1
	if (abs(x1 - x0) < abs(y1 - y0)) {
		steep = true;
		swap(x0, y0); // 斜率绝对值大于1时，将（x0, y0)和(x1, y1)互换位置
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1); // x0 > x1时，将x0与x1互换位置
		swap(y0, y1);
	}
	double dx = x1 - x0;
	double dy = y1 - y0;
	double m = dy / dx;
	int y = y0;
	float P[100];
	P[0] = 2 * dy - dx;
	int k = 0;
	for (int x = x0; x < x1; ++x) {
		if (steep == true) {
			image.set(y, x, color);
		}
		if (steep == false) {
			image.set(x, y, color);
		}
		if (P[k] > 0) {
			P[k + 1] = P[k] + 2 * dy - 2 * dx;
			x += 1;
			y += 1;
		}
		if (P[k] < 0) {
			P[k + 1] = P[k] + 2 * dy;
			x += 1;
		}
		k += 1;
	}
	cout << dx;
	cout << steep;
}

int main(int argc, char** argv) {
	TGAImage image(100, 100, TGAImage::RGB);
	line(13, 20, 23, 40, image, white);
	image.flip_vertically();
	image.write_tga_file("output.tga");
    return 0;
}

