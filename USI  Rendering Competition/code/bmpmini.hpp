#pragma once
/* BSD 2-Clause License

Copyright (c) 2021, Changjiang Yang
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
*/
#include <vector>
#include <fstream>
#include <cstdint>
#include <string>
#include <stdexcept>

namespace image {
#pragma pack(push, 1)
	struct BMPHeader {
		// BMP file header
		uint16_t magic_number; // magic number: BM
		uint32_t file_size;	// in bytes
		uint16_t reserved1;	// Reserved
		uint16_t reserved2;    // Reserved
		uint32_t offset_data;  // Offset of byte where bitmap image data starts

		// BMP information header
		uint32_t header_size;	// size of this header, in bytes (40)
		int32_t width;         // width in pixels (signed integer)
		int32_t height;        // height in pixels (signed integer, positive- bottom-up, negative- top-down)
		uint16_t planes;       // number of color planes (must be 1)
		uint16_t bit_per_pixel;    // number of bits per pixel. Typical values are 1, 4, 8, 16, 24 and 32
		uint32_t compression_method;  // compression method: 0 - uncompressed.
		uint32_t image_size;          // size of the raw bitmap data in bytes, 0 - uncompressed images
		int32_t h_pixels_per_meter; // horizontal resolution of the image
		int32_t v_pixels_per_meter; // vertical resolution of the image
		uint32_t colors_in_palette;    // number of colors in the color palette. Use 0 for the max number of colors allowed by bit_count
		uint32_t important_colors;     // number of important colors used, or 0 when every color is important; generally ignored
		
		BMPHeader() : 
			magic_number(0x4D42), file_size(0), reserved1(0), reserved2(0), offset_data(54),
			header_size(40), width(0), height(0), planes(1), bit_per_pixel(24),
			compression_method(0), image_size(0), h_pixels_per_meter(2835), v_pixels_per_meter(2835),
			colors_in_palette(0), important_colors(0) {}
	};
#pragma pack(pop)

	struct ImageView {
		ImageView(int w, int h, int c, uint8_t* d) :
			width(w), height(h), channels(c), data(d) {}
		ImageView() : width(0), height(0), channels(0), data(NULL) {}
		int width;
		int height;
		int channels;
		uint8_t* data;
	};

	struct ColorPalette {
		ColorPalette() {
			table[0] = 0;
			for (int i = 1; i < 256; i++)
				table[i] = table[i - 1] + 0x00010101u;
		}
		uint32_t table[256];
	};

	class BMPMini
	{
	public:
		void read(const std::string& filename) {
			std::ifstream istrm(filename, std::ios::binary);
			if (!istrm) {
				throw std::ios::failure("Cannot open the input file: " + filename);
			}
			istrm.read(reinterpret_cast<char*>(&header), sizeof(BMPHeader));

			if (header.compression_method != 0 || header.colors_in_palette != 0) {
				throw std::invalid_argument("Only no compression is supported currently");
			}
			
			istrm.seekg(header.offset_data, std::ios::beg);
			int padded_row_size = paddedRowSize();
			int image_size = header.height * padded_row_size;
			std::vector< uint8_t> data(image_size);
			istrm.read(reinterpret_cast<char*>(&data[0]), data.size());

			pixel_data.reserve((header.width * header.height * header.bit_per_pixel) / 8);
			for (int i = 0; i < header.height; i++) {
				int k = header.height - 1 - i;
				uint8_t* ptr = reinterpret_cast<uint8_t*>(&data[0]) + k * padded_row_size;
				pixel_data.insert(pixel_data.end(), ptr, ptr+ (header.width* header.bit_per_pixel)/8);
			}
		}

		void write(const std::string& filename) {
			// upside down and padding
			int padded_row_size = paddedRowSize();
			std::vector<uint8_t> image_data(header.image_size);
			int channels = header.bit_per_pixel / 8;
			for (int i = 0; i < header.height; i++) {
				int k = header.height - 1 - i;
				uint8_t* ptr = &pixel_data[k * header.width * channels];
				std::copy(ptr, ptr + header.width * channels, &image_data[i*padded_row_size]);
			}

			std::ofstream ostrm(filename, std::ios_base::binary);
			if (!ostrm) {
				throw std::runtime_error("Cannot open the output file: " + filename);
			}
			ostrm.write((const char*)&header, sizeof header);
			if (is8bit()) {
				static ColorPalette color_palette;  // Use static instead of constexpr for C++11 compatibility
				ostrm.write((const char*)color_palette.table, sizeof(ColorPalette));
			}
			ostrm.write((const char*)&image_data[0], image_data.size());
		}

		void write(const ImageView rawImage, const std::string& filename) {
			header.width = rawImage.width;
			header.height = rawImage.height;
			header.bit_per_pixel = rawImage.channels * 8;
			int padded_row_size = paddedRowSize();
			header.image_size = header.height * padded_row_size;
			header.offset_data = sizeof(header) + sizeof(ColorPalette);
			header.file_size = header.offset_data + header.image_size;
			pixel_data.resize(rawImage.width * rawImage.height * rawImage.channels);
			std::copy(rawImage.data, rawImage.data + pixel_data.size(), pixel_data.begin());
			write(filename);
		}

		ImageView get() {
			return ImageView(header.width, header.height, header.bit_per_pixel /8, &pixel_data[0]);
		}

	private:
		bool is8bit() const { return header.bit_per_pixel == 8; }
		int paddedRowSize() const { return ((header.width * header.bit_per_pixel + 31) / 32) * 4; }
		BMPHeader header;
		std::vector<uint8_t> pixel_data;
	};
}