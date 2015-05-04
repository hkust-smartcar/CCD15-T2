#include <medianFilter.h>
#include <stdlib.h>
#include <cstring>

void medianFilter(uint16_t* corrupted, uint16_t* smooth, int length)
{  
	int l = 0;
	smooth[0] = corrupted[0];
	smooth[length - 1] = corrupted[length - 1];
	for(int i = 1; i < length - 1; i++){
		uint16_t window[5];
		for(int j = 0; j < 5; j++){
			if(i + j < length - 1){
				window[j] = corrupted[i + j];
			}
		}

		uint16_t temp[5];
		for(int j = 0; j < 5; j++){
			temp[j] = window[j];
			for(int k = j; k < 5; k++){
				if(window[k] < window[j]){
					if(l != k){
						l = k;
						temp[j] = window[k];
					}
				}
			}
		}
		smooth[i] = temp[2];
	}
//    memcpy ( smooth, corrupted, length*sizeof(uint16_t) );
//	for (int i=1;i<length-1;i++)
//	{
//		int k = 0;
//		uint16_t window[9];
//		for(int ii = i - 1; ii < i + 2; ++ii)
//		window[k++] = corrupted[ii];
//		for (int m = 0; m < 9; ++m)
//		{
//			int min = m;
//			for (int n = m + 1; n < 9; ++n)
//				if (window[n] < window[min])
//					min = n;
//			uint16_t temp = window[m];
//			window[m] = window[min];
//			window[min] = temp;
//		}
//		smooth[i] = window[4];
//	}
}
