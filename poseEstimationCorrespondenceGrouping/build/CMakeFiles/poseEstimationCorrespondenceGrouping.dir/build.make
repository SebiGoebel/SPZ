# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sebi/SPZ/poseEstimationCorrespondenceGrouping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build

# Include any dependencies generated for this target.
include CMakeFiles/poseEstimationCorrespondenceGrouping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/poseEstimationCorrespondenceGrouping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/poseEstimationCorrespondenceGrouping.dir/flags.make

CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o: CMakeFiles/poseEstimationCorrespondenceGrouping.dir/flags.make
CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o: ../poseEstimationCorrespondenceGrouping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o -c /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/poseEstimationCorrespondenceGrouping.cpp

CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/poseEstimationCorrespondenceGrouping.cpp > CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.i

CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/poseEstimationCorrespondenceGrouping.cpp -o CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.s

# Object files for target poseEstimationCorrespondenceGrouping
poseEstimationCorrespondenceGrouping_OBJECTS = \
"CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o"

# External object files for target poseEstimationCorrespondenceGrouping
poseEstimationCorrespondenceGrouping_EXTERNAL_OBJECTS =

poseEstimationCorrespondenceGrouping: CMakeFiles/poseEstimationCorrespondenceGrouping.dir/poseEstimationCorrespondenceGrouping.cpp.o
poseEstimationCorrespondenceGrouping: CMakeFiles/poseEstimationCorrespondenceGrouping.dir/build.make
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_surface.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_keypoints.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_tracking.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_recognition.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_stereo.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_outofcore.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_people.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/libOpenNI.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
poseEstimationCorrespondenceGrouping: /usr/lib/libOpenNI2.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libfreetype.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libz.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libjpeg.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libpng.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libtiff.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libexpat.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libqhull_r.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_registration.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_segmentation.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_features.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_filters.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_sample_consensus.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_ml.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_visualization.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_search.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_kdtree.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_io.so
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_octree.so
poseEstimationCorrespondenceGrouping: /usr/lib/libOpenNI.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
poseEstimationCorrespondenceGrouping: /usr/lib/libOpenNI2.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libjpeg.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libpng.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libtiff.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libexpat.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libfreetype.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libz.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libGLEW.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libSM.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libICE.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libX11.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libXext.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libXt.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
poseEstimationCorrespondenceGrouping: /usr/local/lib/libpcl_common.so
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
poseEstimationCorrespondenceGrouping: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
poseEstimationCorrespondenceGrouping: CMakeFiles/poseEstimationCorrespondenceGrouping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable poseEstimationCorrespondenceGrouping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/poseEstimationCorrespondenceGrouping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/poseEstimationCorrespondenceGrouping.dir/build: poseEstimationCorrespondenceGrouping

.PHONY : CMakeFiles/poseEstimationCorrespondenceGrouping.dir/build

CMakeFiles/poseEstimationCorrespondenceGrouping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/poseEstimationCorrespondenceGrouping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/poseEstimationCorrespondenceGrouping.dir/clean

CMakeFiles/poseEstimationCorrespondenceGrouping.dir/depend:
	cd /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sebi/SPZ/poseEstimationCorrespondenceGrouping /home/sebi/SPZ/poseEstimationCorrespondenceGrouping /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build /home/sebi/SPZ/poseEstimationCorrespondenceGrouping/build/CMakeFiles/poseEstimationCorrespondenceGrouping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/poseEstimationCorrespondenceGrouping.dir/depend
