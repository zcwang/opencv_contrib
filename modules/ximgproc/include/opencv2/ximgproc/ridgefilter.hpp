/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*
	Ridge Detection Filter.

	References and Useful links:
	1. [Ridge filter Mathematica ](http://reference.wolfram.com/language/ref/RidgeFilter.html)

	2. [An Unbiased Detector of Curvilinear Structures](dl.acm.org/citation.cfm?id=279239)

	3. [Properties of Ridges and Cores for Two-Dimensional Images](http://www.springerlink.com/content/l6564431145n1880/)

	4. Mathematical insight at : [@nikie](https://dsp.stackexchange.com/users/291/nikie) [article on segmentation of veins in leaves](https://dsp.stackexchange.com/questions/1714/best-way-of-segmenting-veins-in-leaves)

	OpenCV port by : Kushal Vyas (@kushalvyas), Venkatesh Vijaykumar(@venkateshvijaykumar)

 */


#ifndef __OPENCV_XIMGPROC_RIDGEFILTER_HPP__
#define __OPENCV_XIMGPROC_RIDGEFILTER_HPP__

#include <opencv2/core.hpp>

namespace cv{
	namespace ximgproc{
		//! @addtogroup ximgproc_filters
		//! @{
		
		class CV_EXPORTS_W RidgeDetectionFilter : public Algorithm{
			/**
			 *
			 *	@brief	Applies Ridge Detection Filter to an input image. Implements Ridge detection similar to the one in Mathematica using the eigen values from the Hessian Matrix of the input image. For more information refer references.
			 *	
			 */
		public:
			/**
			 * @brief Create pointer to the Ridge detection filter.
			 * @param ddepth (int). Specifies output image depth. defualt is CV_32FC1
			 * @param dx. Order of derivative x
			 * @param dy. Order of derivative y.
			 * @param ksize. sobel kernel size
			 * @param scale (optional). scale value for derivative values
			 * @param delta (optional) . optional bias added to output
			 * @param borderType. pixel extrapolation method. (refer https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=sobel#sobel)
			 * 
			 */

			CV_WRAP static Ptr<RidgeDetectionFilter> create(int ddepth = CV_32FC1, int dx=1, int dy=1, int ksize = 3, double scale = 1, double delta = 0, int borderType = BORDER_DEFAULT);

			/**
			 * @brief Apply Ridge detection filter on input image.
			 * @param img. 	 Source cv::Mat types as supported by cv::Sobel. img can be 1-Channel or 3-Channels.
			 * @param out.	Output image with ridges. 
			 * 
			 */
			CV_WRAP virtual void getRidgeFilteredImage(InputArray _img, OutputArray out) = 0;
		};
	}
}

#endif
