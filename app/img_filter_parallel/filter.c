#include "filter.h"

uint8_t gaussian(uint8_t buffer[5][5]){
	int32_t sum = 0, mpixel;
	uint8_t i, j;

	int16_t kernel[5][5] =	{	{2, 4, 5, 4, 2},
					{4, 9, 12, 9, 4},
					{5, 12, 15, 12, 5},
					{4, 9, 12, 9, 4},
					{2, 4, 5, 4, 2}
				};
	for (i = 0; i < 5; i++)
		for (j = 0; j < 5; j++)
			sum += ((int32_t)buffer[i][j] * (int32_t)kernel[i][j]);
	mpixel = (int32_t)(sum / 159);

	return (uint8_t)mpixel;
}

uint32_t isqrt(uint32_t a){
	uint32_t i, rem = 0, root = 0, divisor = 0;

	for (i = 0; i < 16; i++){
		root <<= 1;
		rem = ((rem << 2) + (a >> 30));
		a <<= 2;
		divisor = (root << 1) + 1;
		if (divisor <= rem){
			rem -= divisor;
			root++;
		}
	}
	return root;
}

uint8_t sobel(uint8_t buffer[3][3]){
	int32_t sum = 0, gx = 0, gy = 0;
	uint8_t i, j;

	int16_t kernelx[3][3] =	{	{-1, 0, 1},
					{-2, 0, 2},
					{-1, 0, 1},
				};
	int16_t kernely[3][3] =	{	{-1, -2, -1},
					{0, 0, 0},
					{1, 2, 1},
				};
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			gx += ((int32_t)buffer[i][j] * (int32_t)kernelx[i][j]);
			gy += ((int32_t)buffer[i][j] * (int32_t)kernely[i][j]);
		}
	}
	
	sum = isqrt(gy * gy + gx * gx);

	if (sum > 255) sum = 255;
	if (sum < 0) sum = 0;

	return (uint8_t)sum;
}

void do_gaussian(uint8_t *input, uint8_t *output, int32_t width, int32_t height){
	int32_t i = 0, j = 0, k, l;
	uint8_t image_buf[5][5];
	
	for(i = 0; i < height; i++){
		if (i > 1 && i < height-2){
			for(j = 0; j < width; j++){
				if (j > 1 && j < width-2){
					for (k = 0; k < 5; k++)
						for(l = 0; l < 5; l++)
							image_buf[k][l] = input[(((i + l-2) * width) + (j + k-2))];

					output[((i * width) + j)] = gaussian(image_buf);
				}else{
					output[((i * width) + j)] = input[((i * width) + j)];
				}
			}
		}else{
			output[((i * width) + j)] = input[((i * width) + j)];
		}
	}
}

void do_sobel(uint8_t *input, uint8_t *output, int32_t width, int32_t height){
	int32_t i = 0, j = 0, k, l;
	uint8_t image_buf[3][3];
	
	for(i = 0; i < height; i++){
		if (i > 2 && i < height-3){
			for(j = 0; j < width-1; j++){
				if (j > 2 && j < width-3){
					for (k = 0; k < 3; k++)
						for(l = 0; l < 3; l++)
							image_buf[k][l] = input[(((i + l-1) * width) + (j + k-1))];

					output[((i * width) + j)] = sobel(image_buf);
				}else{
					output[((i * width) + j)] = 0;
				}
			}
		}else{
			output[((i * width) + j)] = 0;
		}
	}
}

void splitSobel(uint8_t *input, uint8_t *output, int32_t l, int32_t k ) {
    uint8_t buffAux[266*266];
    int32_t i, j;
	int32_t colunm;

    //memset(buffAux, 0, sizeof(buffAux));

    colunm = (l * HEIGHT) > 0 ? (l * HEIGHT) : 0;
    
    for (i = 0; i < 256; i++) {
        for (j = 0; j < 256; j++) {
            buffAux[(((i+5) * 266) + j) + 5] =  input[((i * 256) + j)];
        }    
    }

    for(i = 0; i < 42; i++) {
        for (j = 0; j < 42; j++) {
            output[((i * 42) + j)] = buffAux[(((i+k) * 266) + (j+colunm))];
        }
    }
}

void splitGauss(uint8_t *input, uint8_t *output, int32_t l, int32_t k) {
    uint8_t buffAux[260*260];
    int32_t i = 0, j = 0;
	int32_t colunm = 0;

    //memset(buffAux, 0, sizeof(buffAux));

    colunm = (l * HEIGHT) > 0 ? (l * HEIGHT) : 0;

    //printf("Spliting gaussian buffer: K %d l %d\n", k, l);
    
    for (i = 0; i < 256; i++) {
        for (j = 0; j < 256; j++) {
            buffAux[(((i+2) * 260) + j) + 2] =  input[((i * 256) + j)];
           //printf(" %d", ((i+2) * 260) + j + 2);
        }    
        //printf("\n");
    }

    for(i = 0; i < 36; i++) {
        for (j = 0; j < 36; j++) {
            output[((i * 36) + j)] = buffAux[(((i+k) * 260) + (j+colunm))];
        }
    }
}

void cutImage(uint8_t *output, uint8_t *newMatriz, filter_type filter) {
    int32_t i = 0, j = 0;
 
    switch (filter) {
    case GAUSSIAN:
        for (i = 0; i < 256; i++) {
            for (j = 0; j < 256; j++) {  
                output[((i * 256) + j)] = newMatriz[(((i+2) * 260) + j) + 2];
            }    
        }

        break;
    case SOBEL:
        for (i = 0; i < 256; i++) {
            for (j = 0; j < 256; j++) {  
                output[((i * 256) + j)] = newMatriz[(((i+5) * 266) + j) + 5];
            }    
        }

        break;
    }
}

void appendBuffer(uint8_t *newMatriz, uint8_t *buff, int32_t l, int32_t k, filter_type filter) {
    int32_t i = 0, j = 0, colunm = 0;

    colunm = (l * HEIGHT) > 0 ? (l * HEIGHT) : 0;

    switch (filter) {
    case GAUSSIAN:
        for(i = 0; i < 32; i++) {
            for (j = 0; j < 32; j++) {
                newMatriz[(((i+k) * 260) + (j+colunm))] = buff[(((i+2) * 36) + j+2)]; 
            }
        }

        break;
    case SOBEL:
        for(i = 0; i < 32; i++) {
            for (j = 0; j < 32; j++) {
                newMatriz[(((i+k+4) * 266) + (j+colunm+4))] = buff[(((i+4) * 42) + j+4)];
            }
        }

        break;
    }
}

void showImg(uint8_t *img) {
    int32_t i, j, k = 0;
    int32_t this_width = 256, this_height = 256;

    printf("int32_t width = %d, height = %d;\n", this_width, this_height);
		printf("uint8_t image[] = {\n");
		for (i = 0; i < this_height; i++){
			for (j = 0; j < this_width; j++){
				printf("0x%x", img[i * this_width + j]);
				if ((i < this_height-1) || (j < this_width-1)) printf(", ");
				if ((++k % 16) == 0) printf("\n");
			}
		}
	printf("};\n");
}