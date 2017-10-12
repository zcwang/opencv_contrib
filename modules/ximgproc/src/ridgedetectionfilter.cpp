/*
 *  By downloading, copying, installing or using the software you agree to this license.
 *  If you do not agree to this license, do not download, install,
 *  copy or use the software.
 *
 *
 *  License Agreement
 *  For Open Source Computer Vision Library
 *  (3 - clause BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met :
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and / or other materials provided with the distribution.
 *
 *  * Neither the names of the copyright holders nor the names of the contributors
 *  may be used to endorse or promote products derived from this software
 *  without specific prior written permission.
 *
 *  This software is provided by the copyright holders and contributors "as is" and
 *  any express or implied warranties, including, but not limited to, the implied
 *  warranties of merchantability and fitness for a particular purpose are disclaimed.
 *  In no event shall copyright holders or contributors be liable for any direct,
 *  indirect, incidental, special, exemplary, or consequential damages
 *  (including, but not limited to, procurement of substitute goods or services;
 *  loss of use, data, or profits; or business interruption) however caused
 *  and on any theory of liability, whether in contract, strict liability,
 *  or tort(including negligence or otherwise) arising in any way out of
 *  the use of this software, even if advised of the possibility of such damage.
 */
/*********************************************************
*				Ridge Detection filter
*This code implements a ridge detection filter based on hessian matrix
* Authors : Venkatesh Vijaykumar, Kushal Vyas
* Ref:  @stackoverflow link
*
*/

#include "precomp.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc/ridgefilter.hpp"


namespace cv{
	namespace ximgproc{

		class RidgeDetectionFilterImpl : public RidgeDetectionFilter{
			public:
				int ddepth, dx,  dy,  ksize=3;
				double scale=1,  delta=0;
				int borderType=BORDER_DEFAULT;

				RidgeDetectionFilterImpl(int ddepth, int dx, int dy, int ksize=3, double scale=1, double delta=0, int borderType=BORDER_DEFAULT) {
					this.ddepth = ddepth;
					this.dx = dx;
					this.dy = dy;
					this.ksize = ksize;
					this.scale = scale;
					this.delta = delta;
					this.borderType = borderType;
				};
				virtual void getRidges(InputArray &img, OutputArray &out);
			private:
				virtual void getSobelX(InputArray &img, OutputArray &out);
				virtual void getSobelY(InputArray &img, OutputArray &out);
		};

		void RidgeDetectionFilterImpl::getSobelX(InputArray & _img, OutputArray & _out){
			_out.create(_img.size(), CV_32F);
			Sobel(_img, _out, CV_32F, 1, 0, 3);
		}

		void RidgeDetectionFilterImpl::getSobelY(InputArray & _img, OutputArray & _out){
			_out.create(_img.size(), CV_32F);
			Sobel(_img, _out, CV_32F, 0, 1, 3);
		}

		void RidgeDetectionFilterImpl::getRidges(InputArray & _img, OutputArray & _out){
			Mat img = _img.getMat();
			CV_Assert(!img.empty());
			CV_Assert(img.channels() == 1 || img.channels() == 3);

			if(img.channels() == 3){
				cvtColor(img, img, COLOR_BGR2GRAY);
			}

			Mat sbx, sby;
			getSobelX(img, sbx);
			getSobelY(img, sby);

			Mat sbxx, sbyy, sbxy;
			getSobelX(sbx, sbxx);
			getSobelY(sby, sbyy);
			getSobelY(sbx, sbxy);

			Mat sb2xx, sb2yy, sb2xy;
			multiply(sbxx, sbxx, sb2xx);
			multiply(sbyy, sbyy, sb2yy);
			multiply(sbxy, sbxy, sb2xy);

			Mat sbxxyy;
			multiply(sbxx, sbyy, sbxxyy);

			Mat rootex;
			rootex = (sb2xx +  multiply(sb2xy, Scalar(4))  - multiply(sbxxyy , Scalar(2)) + sb2xy );
			Mat root;
			sqrt(rootex, root);
			Mat ridgexp;
			ridgexp = ( (sbxx + sbyy) + root );
			ridgexp /= 2;
			ridgexp.copyTo(_out);
		}
	}
}








