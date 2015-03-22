/*
 * Vector.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef VECTOR_H_
#define VECTOR_H_

namespace Math{

	class Vector{

		public:
			static void Add(double*, double*, double*);
			static void Scale(double*, double*, double);
			static void Multiplication(double*, double[3][3], double*);
			static double DotProduct(double*, double*);
			static void CrossProduct(double*, double*, double*);
		private:
	};
};

using namespace Math;

#endif /* VECTOR_H_ */
