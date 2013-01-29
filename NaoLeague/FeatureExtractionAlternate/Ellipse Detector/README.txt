ABOUT THIS SOURCE CODE
The files in this folder contain the source code of ELSD, published in 
'A Parameterless Line Segment and Elliptical Arc Detector with Enhanced Ellipse
Fitting', V. Patraucean, P. Gurdjos, R. Grompone von Gioi, ECCV2012.

Corresponding author: viorica patraucean vpatrauc@enseeiht.fr.  
 
The code generating and validating line segment hypotheses is taken up from 
LSD source code, available at http://www.ipol.im/pub/art/2012/gjmr-lsd/.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU Affero General Public License as published by the Free 
Software Foundation, either version 3 of the License, or (at your option) any
later version. 


REQUIREMENTS
The ELSD source code needs the CLAPACK/CBLAS library for some linear algebra 
computations. Version 3.2.1 was used.


COMPILATION AND EXECUTION
If the paths to the libraries are ok, a simple 'make' would compile and 
produce the executable.

To run it, use

./elsd image_name.pgm

where:
image_name.pgm is the name of the image to be analysed by ELSD. 
This ELSD version works only with PGM images.


FILE DESCRIPTION
'makefile' is an example of makefile to compile the source code;
'elsd.c' contains the main() function, and some other functions, 
e.g. for read/write pgm images;
'process_line.c' contains the functions used to produce and validate
the line segment hypothesis; the code is taken up mainly from the 
LSD source code.
'process_curve.c' contains the functions used to produce the ellipse/circle
hypotheses;
'valid_curve.c' contains the functions to validate a circle/ellipse hypothesis;
'write_svg.c' : functions to write the result in svg format; the parameters of 
the circular/elliptical arcs are also written in 'ellipses.txt' in the form:
x_c y_c a b theta ang_start ang_end
'test_image.pgm' is an example image on which you can test the detector. If the 
compilation went ok, execute the detector with ./elsd test_image.pgm
You should get a result (in SVG format) similar to 'test_result.png'. In the 
console, you should get the number of each type of feature.
In this case '17 310 165', meaning 17 elliptical arcs, 310 circular arcs, and 165 
line segments. Also, the program displays the execution time, which is 
machine-dependent (1.2s on our regular machine).  
