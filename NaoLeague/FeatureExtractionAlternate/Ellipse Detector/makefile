#ELSD - Ellipse and Line Segment Detector
#
#  Copyright (c) 2012 viorica patraucean (vpatrauc@gmail.com)
#  
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Affero General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Affero General Public License for more details.
#
#  You should have received a copy of the GNU Affero General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.


INC_PATH=/home/amogh/Downloads/CLAPACK-3.2.1/INCLUDE
LIB_PATHS= -L/usr/lib
LIBS= -llapack -lblas -lf2c -lm 
CFLAGS= -O0 -Wall -Wextra
CC=g++ -I$(INC_PATH)
EXEC = elsd

all: $(EXEC)

elsd: elsd.o valid_curve.o process_line.o process_curve.o write_svg.o 
	$(CC) -o elsd elsd.o valid_curve.o process_line.o process_curve.o write_svg.o $(LIB_PATHS) $(LIBS)

elsd.o: elsd.c
	$(CC) -o elsd.o -c elsd.c $(CFLAGS)	

valid_curve.o: valid_curve.c
	$(CC) -o valid_curve.o -c valid_curve.c $(CFLAGS)

process_line.o: process_line.c
	$(CC) -o process_line.o -c process_line.c $(CFLAGS)

process_curve.o: process_curve.c
	$(CC) -o process_curve.o -c process_curve.c $(CFLAGS)

write_svg.o: write_svg.c
	$(CC) -o write_svg.o -c write_svg.c $(CFLAGS)

clean :
	rm -fr *.o


