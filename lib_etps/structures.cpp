#include "stdafx.h"
#include "structures.h"
#include "functions.h"

// Pixel defininions
///////////////////////////

// For debug purposes
string Pixel::GetPixelsAsString()
{
    stringstream ss;

    ss << '{';
    for (int r = ulr; r < lrr; r++) {
        for (int c = ulc; c < lrc; c++) {
            if (r != ulr || c != ulc) ss << ',';
            ss << '{' << r << ',' << c << '}';
        }
    }
    ss << '}';
    return ss.str();
}

void Pixel::Split(int row1, int row2, int col1, int col2, Pixel& p11, Pixel& p12, Pixel& p21, Pixel& p22)
{
    int cols = GetCSize();
    int cols1 = cols / 2;
    int rows = GetRSize();
    int rows1 = rows / 2;
    
    p11.Initialize(row1, col1, ulr, ulc, ulr + rows1, ulc + cols1);
    p12.Initialize(row1, col2, ulr, ulc + cols1, ulr + rows1, ulc + cols);
    p21.Initialize(row2, col1, ulr + rows1, ulc, ulr + rows, ulc + cols1);
    p22.Initialize(row2, col2, ulr + rows1, ulc + cols1, ulr + rows, ulc + cols);
    p11.superPixel = superPixel;
    p12.superPixel = superPixel;
    p21.superPixel = superPixel;
    p22.superPixel = superPixel;
    p11.border = border & (BTopFlag | BLeftFlag);
    p12.border = border & (BTopFlag | BRightFlag);
    p21.border = border & (BLeftFlag | BBottomFlag);
    p22.border = border & (BRightFlag | BBottomFlag);
    superPixel->numP += 3;
}

void Pixel::SplitRow( int row1, int row2, int col, Pixel& p11, Pixel& p21)
{
    int cols = GetCSize();
    int rows = GetRSize();
    int rows1 = rows / 2;

    p11.Initialize(row1, col, ulr, ulc, ulr + rows1, ulc + cols);
    p21.Initialize(row2, col, ulr + rows1, ulc, ulr + rows, ulc + cols);
    p11.superPixel = superPixel;
    p21.superPixel = superPixel;
    p11.border = border & (BTopFlag | BLeftFlag | BRightFlag);
    p21.border = border & (BLeftFlag | BBottomFlag | BRightFlag);
    superPixel->numP++;
}

void Pixel::CopyTo(int row, int col, Pixel& p11)
{
    int cols = GetCSize();
    int rows = GetRSize();

    p11.Initialize(row, col, ulr, ulc, ulr + rows, ulc + cols);
    p11.superPixel = superPixel;
    p11.border = border;
}

void Pixel::SplitColumn(int row, int col1, int col2, Pixel& p11, Pixel& p12)
{
    int cols = GetCSize();
    int cols1 = cols / 2;
    int rows = GetRSize();

    p11.Initialize(row, col1, ulr, ulc, ulr + rows, ulc + cols1);
    p12.Initialize(row, col2, ulr, ulc + cols1, ulr + rows, ulc + cols);
    p11.superPixel = superPixel;
    p12.superPixel = superPixel;
    p11.border = border & (BTopFlag | BLeftFlag | BBottomFlag);
    p12.border = border & (BTopFlag | BRightFlag | BBottomFlag);
    superPixel->numP++;
}

void Pixel::UpdateInliers(const cv::Mat1d& dispImg, double threshold, cv::Mat1b& inliers) const
{
    const Plane_d& plane = ((SuperpixelStereo*)superPixel)->plane;

    for (int i = ulr; i < (int)lrr; i++) {
        for (int j = ulc; j < (int)lrc; j++) {
            const double& disp = dispImg(i, j);
            inliers(i, j) = fabs(DotProduct(plane, i, j, 1.0) - disp) < threshold;
        }
    }
}

// Superpixel defininions
///////////////////////////

// We remove Pixel p from this superpixel, recalculates size and energies
void Superpixel::GetRemovePixelData(const PixelData& pd, PixelChangeData& pcd) const
{
    std::vector<double> _newSumR, _newSumG, _newSumB;
    std::vector<double> _newSumR2, _newSumG2, _newSumB2;

    std::transform(sumR.begin(),sumR.end(), pd.sumR.begin(), std::back_inserter(_newSumR), std::minus<double>());
    std::transform(sumG.begin(),sumG.end(), pd.sumG.begin(), std::back_inserter(_newSumG), std::minus<double>());
    std::transform(sumB.begin(),sumB.end(), pd.sumB.begin(), std::back_inserter(_newSumB), std::minus<double>());
    std::transform(sumR2.begin(),sumR2.end(), pd.sumR2.begin(), std::back_inserter(_newSumR2), std::minus<double>());
    std::transform(sumG2.begin(),sumG2.end(), pd.sumG2.begin(), std::back_inserter(_newSumG2), std::minus<double>());
    std::transform(sumB2.begin(),sumB2.end(), pd.sumB2.begin(), std::back_inserter(_newSumB2), std::minus<double>());

    pcd.newEApp = CalcAppEnergy(_newSumR, _newSumG, _newSumB,
        _newSumR2, _newSumG2, _newSumB2, size - pd.size, numP - 1, imageDimensions, pNorm);
    pcd.newEReg = CalcRegEnergy(sumRow - pd.sumRow, sumCol - pd.sumCol,
        sumRow2 - pd.sumRow2, sumCol2 - pd.sumCol2, size - pd.size, numP - 1);
    pcd.newSize = size - pd.size;
}


// We add Pixel p to this superpixel, recalculates size and energies
void Superpixel::GetAddPixelData(const PixelData& pd, PixelChangeData& pcd) const
{
    std::vector<double> _newSumR, _newSumG, _newSumB;
     std::vector<double> _newSumR2, _newSumG2, _newSumB2;

     std::transform(sumR.begin(),sumR.end(), pd.sumR.begin(), std::back_inserter(_newSumR), std::plus<double>());
     std::transform(sumG.begin(),sumG.end(), pd.sumG.begin(), std::back_inserter(_newSumG), std::plus<double>());
     std::transform(sumB.begin(),sumB.end(), pd.sumB.begin(), std::back_inserter(_newSumB), std::plus<double>());
     std::transform(sumR2.begin(),sumR2.end(), pd.sumR2.begin(), std::back_inserter(_newSumR2), std::plus<double>());
     std::transform(sumG2.begin(),sumG2.end(), pd.sumG2.begin(), std::back_inserter(_newSumG2), std::plus<double>());
     std::transform(sumB2.begin(),sumB2.end(), pd.sumB2.begin(), std::back_inserter(_newSumB2), std::plus<double>());

    pcd.newEApp = CalcAppEnergy(_newSumR, _newSumG, _newSumB, _newSumR2,
        _newSumG2, _newSumB2, size + pd.size, numP + 1, imageDimensions, pNorm);
    pcd.newEReg = CalcRegEnergy(sumRow + pd.sumRow, sumCol + pd.sumCol,
        sumRow2 + pd.sumRow2, sumCol2 + pd.sumCol2, size + pd.size, numP + 1);
    pcd.newSize = size + pd.size;
}

// SuperpixelStereo definitions
/////////////////////////////////

void SuperpixelStereo::SetPlane(Plane_d& plane_)
{
    plane = plane_;
}

void SuperpixelStereo::UpdateDispSum(const cv::Mat1d& depthImg, double inlierThresh, double noDisp)
{
    sumDisp = 0;
    for (Pixel* p : pixels) {
        sumDisp += p->CalcDispSum(depthImg, plane, inlierThresh, noDisp);
    }
}

void SuperpixelStereo::CalcPlaneLeastSquares(const cv::Mat1d& depthImg)
{
    LeastSquaresPlane(sumIRow, sumIRow2, sumICol, sumICol2, sumIRowCol, sumIRowD, sumIColD, sumID, nI, plane);
}

void SuperpixelStereo::GetRemovePixelDataStereo(const PixelData& pd, PixelChangeDataStereo& pcd) const 
{
    GetRemovePixelData(pd, pcd);
    pcd.newEDisp = sumDisp - pd.sumDispP;
}

void SuperpixelStereo::GetAddPixelDataStereo(const PixelData& pd, PixelChangeDataStereo& qcd) const
{
    GetAddPixelData(pd, qcd);
    qcd.newEDisp = sumDisp + pd.sumDispQ;
}

double SuperpixelStereo::CalcDispEnergy(const cv::Mat1d& dispImg, double inlierThresh, double noDisp)
{
    double sum = 0.0;

    for (Pixel* p : pixels) {
        sum += p->CalcDispSum(dispImg, plane, inlierThresh, noDisp);
    }
    return sum;
}

void SuperpixelStereo::CheckAppEnergy(const cv::Mat& img)
{
    std::vector<double> rr(imageDimensions,0.0), rr2(imageDimensions,0.0), gg(imageDimensions,0.0), gg2(imageDimensions,0.0), bb(imageDimensions,0.0), bb2(imageDimensions,0.0);
    //auto addition = [&](int l, int r){ return (l + r);};

    for (Pixel* p : pixels) {
        std::vector<double> r, r2 , g, g2 , b , b2;
        p->CalcRGBDSum(img, r, r2, g, g2, b, b2);
        std::transform(rr.begin(),rr.end(), r.begin(), rr.begin(), std::plus<double>());
        std::transform(rr2.begin(),rr2.end(), r2.begin(), rr2.begin(), std::plus<double>());
        std::transform(gg.begin(),gg.end(), g.begin(), gg.begin(), std::plus<double>());
        std::transform(gg2.begin(),gg2.end(), g2.begin(), gg2.begin(), std::plus<double>());
        std::transform(bb.begin(),bb.end(), b.begin(), bb.begin(), std::plus<double>());
        std::transform(bb2.begin(),bb2.end(), b2.begin(), bb2.begin(), std::plus<double>());
        //rr += r; rr2 += r2; gg += g; gg2 += g2; bb += b; bb2 += b2;
    }
    for(int d = 0; d < imageDimensions; ++d){
        if (fabs(rr.at(d) - sumR.at(d)) > 0.01 || fabs(rr2.at(d) - sumR2.at(d)) > 0.01 || fabs(gg.at(d)- sumG.at(d)) > 0.01
                || fabs(gg2.at(d) - sumG2.at(d)) > 0.01 || fabs(bb.at(d) - sumB.at(d)) > 0.01 || fabs(bb2.at(d) - sumB2.at(d)) > 0.01) {
            cout << "app sum mismatch";
        }
    }
}

void SuperpixelStereo::CheckRegEnergy()
{
    double rr = 0, rr2 = 0, cc = 0, cc2 = 0, rrcc = 0;

    for (Pixel* p : pixels) {
        double r = 0, r2 = 0, c = 0, c2 = 0, rc = 0;
        p->CalcRowColSum(r, r2, c, c2, rc);
        rr += r; rr2 += r2; cc += c; cc2 += c2; rrcc += rc;
    }
    if (fabs(rr - sumRow) > 0.01 || fabs(rr2 - sumRow2) > 0.01 || fabs(cc - sumCol) > 0.01 || fabs(cc2 - sumCol2) > 0.01 || fabs(rrcc - sumRowCol) > 0.01) {
        cout << "colrow sum mismatch";
    }
    if (fabs(eReg - CalcRegEnergy(sumRow, sumCol, sumRow2, sumCol2, size, numP)) > 0.01) {
        cout << "RE mismatch";
    }
}

// Functions
///////////////////////////////////////////////////////////////////////////////

//void LeastSquaresPlane(double sumIRow, double sumIRow2, double sumICol, double sumICol2, double sumIRowCol, double sumIRowD, double sumIColD,
//    double sumID, double nI, Plane_d& plane)
//{
//    CV_Assert(sumIRow >= 0 && sumIRow2 >= 0 && sumICol >= 0 && sumICol2 >= 0 && sumIRowCol >= 0);
//
//    cv::Mat1d left = (cv::Mat1d(3, 3) << sumIRow2, sumIRowCol, sumIRow, sumIRowCol, sumICol2, sumICol, sumIRow, sumICol, nI);
//    cv::Mat1d right = (cv::Mat1d(3, 1) << sumIRowD, sumIColD, sumID);
//    cv::Mat1d result;
//
//    if (cv::solve(left, right, result, cv::DECOMP_CHOLESKY)) {
//        plane.x = result(0, 0);
//        plane.y = result(1, 0);
//        plane.z = result(2, 0);
//
//        Plane_d dbgPlane;
//
//        //LeastSquaresPlaneDebug(sumIRow2, sumIRowCol, sumIRow, sumIRowD, sumIRowCol, sumICol2,
//        //    sumICol, sumIColD, sumIRow, sumICol, nI, sumID, dbgPlane);
//        //double d = norm(dbgPlane - plane);
//        //if (d > 0.01) {
//        //    cout << " plane diff: " << d << " ";
//        //}
//       // CV_Assert(d < 0.001);
//
//    } else {
//
//        // leave as it is... plane.x = plane.y = plane.z = 0.0;
//    }
//
//}

void LeastSquaresPlane(double sumIRow, double sumIRow2, double sumICol, double sumICol2, double sumIRowCol, double sumIRowD, double sumIColD,
    double sumID, double nI, Plane_d& plane)
{
    CV_Assert(sumIRow >= 0 && sumIRow2 >= 0 && sumICol >= 0 && sumICol2 >= 0 && sumIRowCol >= 0);

    LeastSquaresPlaneDebug(sumIRow2, sumIRowCol, sumIRow, sumIRowD, sumIRowCol, sumICol2,
        sumICol, sumIColD, sumIRow, sumICol, nI, sumID, plane);

}

