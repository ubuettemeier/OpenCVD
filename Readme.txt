OpenCVD

OpenCVD ist eine interaktive Visualisierungssoftware.
Sie ermöglicht eine schnelle und effektive Unterstützung bei der Entwicklung von Bildverarbeitungs Applikationen.
Sie unterstützt eine Großzahl von OpenCV Funktionen. Darüber hinaus sind auch spezialisierte Bausteine verfügbar.

Die Arbeitsweise stellt sich wie folgt dar:
  In der Bildverarbeitungsapplikation wird anstelle von cv:: der namespace CVD:: verwendet.
  Die aufgerufenen CVD:: Funktionen leiten ihre Parameter zu einem lokalen Server.
  Dieser bietet nun die Möglichkeit, Funktionsparameter während der Laufzeit zu verändern.

Entwicklungsumgebung:
Ubuntu 18.04.2 LTS
OpenCV 3.3.0

Server:
qtcreator

Client:
gcc version 7.3.0
g++ --std=c++11

folgende OpenCV Funktionen sind implementiert

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

