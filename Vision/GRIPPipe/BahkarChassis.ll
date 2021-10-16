//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat hsvThresholdOutput;
	Mat blurOutput;
	BlobsReport findBlobsOutput;
}

//
// Steps
//

Step HSV_Threshold0
{
    Mat hsvThresholdInput = source0;
    List hsvThresholdHue = [11.33093525179856, 39.09090909090908];
    List hsvThresholdSaturation = [199.50539568345323, 255.0];
    List hsvThresholdValue = [176.57374100719423, 255.0];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}

Step Blur0
{
    Mat blurInput = hsvThresholdOutput;
    BlurType blurType = GAUSSIAN;
    Double blurRadius = 5.405405405405405;

    blur(blurInput, blurType, blurRadius, blurOutput);
}

Step Find_Blobs0
{
    Mat findBlobsInput = blurOutput;
    Double findBlobsMinArea = 1.0;
    List findBlobsCircularity = [0.0, 1.0];
    Boolean findBlobsDarkBlobs = false;

    findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);
}

Step NTPublish_BlobsReport0
{
    BlobsReport ntpublishBlobsreportData = findBlobsOutput;
    String ntpublishBlobsreportName = "Balls";
    Boolean ntpublishBlobsreportPublishX = true;
    Boolean ntpublishBlobsreportPublishY = true;
    Boolean ntpublishBlobsreportPublishSize = true;

    ntpublishBlobsreport(ntpublishBlobsreportData, ntpublishBlobsreportName, ntpublishBlobsreportPublishX, ntpublishBlobsreportPublishY, ntpublishBlobsreportPublishSize, );
}




