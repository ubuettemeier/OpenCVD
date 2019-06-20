**OpenCVD**

<p>OpenCVD ist eine interaktive Visualisierungssoftware.
Sie ermöglicht eine schnelle und effektive Unterstützung bei der Entwicklung von Bildverarbeitungs Applikationen.
Sie unterstützt eine Großzahl von OpenCV Funktionen. 
Darüber hinaus sind auch spezialisierte Bausteine verfügbar.</p>

Die Arbeitsweise stellt sich wie folgt dar:

- In der Bildverarbeitungsapplikation wird anstelle von cv:: der namespace CVD:: verwendet.
- Die aufgerufenen CVD:: Funktionen leiten ihre Parameter zu einem lokalen Server.
- Dieser bietet nun die Möglichkeit, Funktionsparameter während der Laufzeit zu verändern.

<p>Der Server bietet vollen Zugriff auf alle Funktionsparameter. Das einbinden von Eingabefeldern und Trackbars entfällt.</p>
<hr></hr>
<p>OpenCVD is an interactive visualization software.
It enables fast and effective support in the development of image processing applications. 
It supports a large number of OpenCV functions. 
In addition, specialized components are also available.</p>

The working method is as follows:

- In the image processing application, the namespace CVD:: is used instead of cv::.
- The called CVD:: functions redirect their parameters to a local server.
- This server now offers the possibility to change function parameters during runtime.

<p>The server offers full access to all function parameters. There is no need to integrate input fields and trackbars.</p>
<hr></hr>

![alt](Readme.png)

<hr></hr>

**Example**

see: samples/short_sample/short_sample.cpp

<hr></hr>

**Entwicklungsumgebung / development-environment**

- Ubuntu 18.04.2 LTS
- OpenCV 3.3.0

**Server:**

- qtcreator
- Qt 5.9.5

**Samples:**
**include:**

- gcc version 7.3.0
- g++ --std=c++11
- OpenCV 3.3.0
- Makefile's use pkg-config --cflag --clib opencv

<hr></hr>

**aktuell sind folgende OpenCV Funktionen implementiert / currently the following OpenCV functions are implemented**

- adaptiveThreshold, approxPolyDP
- blur
- Canny, cvtColor, calcHist, convertScaleAbs, cornerHarris
- dilate
- erode
- findContours, fitLine
- GaussianBlur, getStructuringElement
- HoughCircles, HoughLinesP, HoughLines
- imread
- Laplacian
- medianBlur, morphologyEx
- normalize
- pyrDown, pyrUp
- resized
- Scharr, Sobel, scaleAdd
- threshold

**cv::Mat Funktionen / cv::Mat functions**

- Mat ( int rows, int cols, int type );
- Mat ( Size size, int type );
- Mat ( Size size, int type, const Scalar& s );
- Mat ( int rows, int cols, int type, const Scalar& s );
- Mat ( Mat &m, Rect &ROI );
- Mat.convertTo ( OutputArray m, int rtype, double alpha, double beta );
- Mat.assigTo ( Mat &m, int type );
- Mat.ones ( int rows, int cols, int type );
- Mat.ones ( Size size, int type );
- Mat.ones ( int ndims, const int* sz, int type );


