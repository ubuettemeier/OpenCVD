**OpenCVD**

OpenCVD ist eine interaktive Visualisierungssoftware.
Sie ermöglicht eine schnelle und effektive Unterstützung bei der Entwicklung von Bildverarbeitungs Applikationen.
Sie unterstützt eine Großzahl von OpenCV Funktionen. 
Darüber hinaus sind auch spezialisierte Bausteine verfügbar.

Die Arbeitsweise stellt sich wie folgt dar:
- In der Bildverarbeitungsapplikation wird anstelle von cv:: der namespace CVD:: verwendet.
- Die aufgerufenen CVD:: Funktionen leiten ihre Parameter zu einem lokalen Server.
- Dieser bietet nun die Möglichkeit, Funktionsparameter während der Laufzeit zu verändern.
-----------------------------------------------------------------------------------------------------------------------------------------

OpenCVD is an interactive visualization software.
It enables fast and effective support in the development of image processing applications. 
It supports a large number of OpenCV functions. 
In addition, specializedcomponents are also available.

The working method is as follows:
- In the image processing application, the namespace CVD:: is used instead of cv::.
- The called CVD::: functions redirect their parameters to a local server.
- This server now offers the possibility to change function parameters during runtime.

-----------------------------------------------------------------------------------------------------------------------------------------

![alt](Readme.png)

-----------------------------------------------------------------------------------------------------------------------------------------
**Entwicklungsumgebung / development-environment:**

- Ubuntu 18.04.2 LTS
- OpneCV 3.3.0

Server:

- qtcreator

Client:

- gcc version 7.3.0
- g++ --std=c++11

**folgende OpenCV Funktionen sind implementiert / the following OpenCV functions are implemented**

- medianBlur
- blur
- GaussianBlur
- Canny
- threshold
- cvtColor
- dilate
- erode
- morphologyEx
- getStructuringElement
- convertScaleAbs
- findContours
- Laplacian
- imread
- normalize
- calcHist
