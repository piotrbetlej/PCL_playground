# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.1

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/Stuff/bot/work/pcl_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/Stuff/bot/work/pcl_test-build

# Include any dependencies generated for this target.
include CMakeFiles/pcl_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcl_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcl_test.dir/flags.make

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o: CMakeFiles/pcl_test.dir/flags.make
CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o: /media/Stuff/bot/work/pcl_test/src/xtionviewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /media/Stuff/bot/work/pcl_test-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o -c /media/Stuff/bot/work/pcl_test/src/xtionviewer.cpp

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /media/Stuff/bot/work/pcl_test/src/xtionviewer.cpp > CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.i

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /media/Stuff/bot/work/pcl_test/src/xtionviewer.cpp -o CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.s

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.requires:
.PHONY : CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.requires

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.provides: CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcl_test.dir/build.make CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.provides.build
.PHONY : CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.provides

CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.provides.build: CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o

# Object files for target pcl_test
pcl_test_OBJECTS = \
"CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o"

# External object files for target pcl_test
pcl_test_EXTERNAL_OBJECTS =

pcl_test: CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o
pcl_test: CMakeFiles/pcl_test.dir/build.make
pcl_test: /usr/local/lib/libboost_system.so
pcl_test: /usr/local/lib/libboost_filesystem.so
pcl_test: /usr/local/lib/libboost_thread.so
pcl_test: /usr/local/lib/libboost_date_time.so
pcl_test: /usr/local/lib/libboost_iostreams.so
pcl_test: /usr/local/lib/libboost_serialization.so
pcl_test: /usr/local/lib/libboost_chrono.so
pcl_test: /usr/local/lib/libpcl_common.so
pcl_test: /usr/local/lib/libpcl_octree.so
pcl_test: /usr/lib/libOpenNI.so
pcl_test: /usr/local/share/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so
pcl_test: /usr/local/lib/libpcl_io.so
pcl_test: /usr/local/lib/libflann_cpp_s.a
pcl_test: /usr/local/lib/libpcl_kdtree.so
pcl_test: /usr/local/lib/libpcl_search.so
pcl_test: /usr/local/lib/libpcl_sample_consensus.so
pcl_test: /usr/local/lib/libpcl_filters.so
pcl_test: /usr/local/lib/libpcl_features.so
pcl_test: /usr/local/lib/libpcl_ml.so
pcl_test: /usr/local/lib/libpcl_segmentation.so
pcl_test: /usr/local/lib/libpcl_visualization.so
pcl_test: /usr/lib/libqhull.so
pcl_test: /usr/local/lib/libpcl_surface.so
pcl_test: /usr/local/lib/libpcl_registration.so
pcl_test: /usr/local/lib/libpcl_keypoints.so
pcl_test: /usr/local/lib/libpcl_tracking.so
pcl_test: /usr/local/lib/libpcl_recognition.so
pcl_test: /usr/local/lib/libpcl_stereo.so
pcl_test: /usr/local/lib/libpcl_people.so
pcl_test: /usr/local/lib/libpcl_outofcore.so
pcl_test: /usr/local/lib/libboost_system.so
pcl_test: /usr/local/lib/libboost_filesystem.so
pcl_test: /usr/local/lib/libboost_thread.so
pcl_test: /usr/local/lib/libboost_date_time.so
pcl_test: /usr/local/lib/libboost_iostreams.so
pcl_test: /usr/local/lib/libboost_serialization.so
pcl_test: /usr/local/lib/libboost_chrono.so
pcl_test: /usr/lib/libqhull.so
pcl_test: /usr/lib/libOpenNI.so
pcl_test: /usr/local/share/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so
pcl_test: /usr/local/lib/libflann_cpp_s.a
pcl_test: /usr/local/lib/libvtkIOVideo-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingMath-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingStencil-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersSelection-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingMorphological-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingLOD-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOImport-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOMINC-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersFlowPaths-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOParallel-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersSMP-6.1.so.1
pcl_test: /usr/local/lib/libvtkViewsContext2D-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersHyperTree-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingFreeTypeOpenGL-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingStatistics-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOInfovis-6.1.so.1
pcl_test: /usr/local/lib/libvtklibxml2-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingLIC-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersGeneric-6.1.so.1
pcl_test: /usr/local/lib/libvtkViewsGeovis-6.1.so.1
pcl_test: /usr/local/lib/libvtkViewsInfovis-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOEnSight-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersVerdict-6.1.so.1
pcl_test: /usr/local/lib/libvtkverdict-6.1.so.1
pcl_test: /usr/local/lib/libvtkDomainsChemistry-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersProgrammable-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOExodus-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersTexture-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOLSDyna-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingVolumeAMR-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOSQL-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingImage-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOAMR-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOExport-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOMovie-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOPLY-6.1.so.1
pcl_test: /usr/local/lib/libvtkInteractionImage-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersParallelImaging-6.1.so.1
pcl_test: /usr/local/lib/libpcl_common.so
pcl_test: /usr/local/lib/libpcl_octree.so
pcl_test: /usr/local/lib/libpcl_io.so
pcl_test: /usr/local/lib/libpcl_kdtree.so
pcl_test: /usr/local/lib/libpcl_search.so
pcl_test: /usr/local/lib/libpcl_sample_consensus.so
pcl_test: /usr/local/lib/libpcl_filters.so
pcl_test: /usr/local/lib/libpcl_features.so
pcl_test: /usr/local/lib/libpcl_ml.so
pcl_test: /usr/local/lib/libpcl_segmentation.so
pcl_test: /usr/local/lib/libpcl_visualization.so
pcl_test: /usr/local/lib/libpcl_surface.so
pcl_test: /usr/local/lib/libpcl_registration.so
pcl_test: /usr/local/lib/libpcl_keypoints.so
pcl_test: /usr/local/lib/libpcl_tracking.so
pcl_test: /usr/local/lib/libpcl_recognition.so
pcl_test: /usr/local/lib/libpcl_stereo.so
pcl_test: /usr/local/lib/libpcl_people.so
pcl_test: /usr/local/lib/libpcl_outofcore.so
pcl_test: /usr/local/lib/libvtkIONetCDF-6.1.so.1
pcl_test: /usr/local/lib/libvtkGeovisCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkproj4-6.1.so.1
pcl_test: /usr/local/lib/libvtkViewsCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkInfovisLayout-6.1.so.1
pcl_test: /usr/local/lib/libvtkChartsCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkInfovisCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonColor-6.1.so.1
pcl_test: /usr/local/lib/libvtkexoIIc-6.1.so.1
pcl_test: /usr/local/lib/libvtkNetCDF_cxx-6.1.so.1
pcl_test: /usr/local/lib/libvtkNetCDF-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOXML-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOXMLParser-6.1.so.1
pcl_test: /usr/local/lib/libvtkexpat-6.1.so.1
pcl_test: /usr/local/lib/libvtksqlite-6.1.so.1
pcl_test: /usr/local/lib/libvtkhdf5_hl-6.1.so.1
pcl_test: /usr/local/lib/libvtkhdf5-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersAMR-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingGL2PS-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingContext2D-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingOpenGL-6.1.so.1
pcl_test: /usr/lib/x86_64-linux-gnu/libXt.so
pcl_test: /usr/local/lib/libvtkgl2ps-6.1.so.1
pcl_test: /usr/lib/x86_64-linux-gnu/libGLU.so
pcl_test: /usr/lib/x86_64-linux-gnu/libSM.so
pcl_test: /usr/lib/x86_64-linux-gnu/libICE.so
pcl_test: /usr/lib/x86_64-linux-gnu/libX11.so
pcl_test: /usr/lib/x86_64-linux-gnu/libXext.so
pcl_test: /usr/local/lib/libvtkRenderingLabel-6.1.so.1
pcl_test: /usr/local/lib/libvtkoggtheora-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOGeometry-6.1.so.1
pcl_test: /usr/local/lib/libvtkjsoncpp-6.1.so.1
pcl_test: /usr/local/lib/libvtkInteractionWidgets-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingHybrid-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOImage-6.1.so.1
pcl_test: /usr/local/lib/libvtkDICOMParser-6.1.so.1
pcl_test: /usr/local/lib/libvtkmetaio-6.1.so.1
pcl_test: /usr/local/lib/libvtkpng-6.1.so.1
pcl_test: /usr/local/lib/libvtktiff-6.1.so.1
pcl_test: /usr/local/lib/libvtkjpeg-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersHybrid-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingAnnotation-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingFreeType-6.1.so.1
pcl_test: /usr/local/lib/libvtkftgl-6.1.so.1
pcl_test: /usr/local/lib/libvtkfreetype-6.1.so.1
pcl_test: /usr/lib/x86_64-linux-gnu/libGL.so
pcl_test: /usr/local/lib/libvtkRenderingVolume-6.1.so.1
pcl_test: /usr/local/lib/libvtkInteractionStyle-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingColor-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersParallel-6.1.so.1
pcl_test: /usr/local/lib/libvtkRenderingCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersExtraction-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersGeometry-6.1.so.1
pcl_test: /usr/local/lib/libvtkParallelCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOLegacy-6.1.so.1
pcl_test: /usr/local/lib/libvtkIOCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkzlib-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersModeling-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersSources-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersGeneral-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonComputationalGeometry-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersImaging-6.1.so.1
pcl_test: /usr/local/lib/libvtkFiltersStatistics-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingFourier-6.1.so.1
pcl_test: /usr/local/lib/libvtkalglib-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingGeneral-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingSources-6.1.so.1
pcl_test: /usr/local/lib/libvtkImagingCore-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonExecutionModel-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonDataModel-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonMisc-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonSystem-6.1.so.1
pcl_test: /usr/local/lib/libvtksys-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonTransforms-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonMath-6.1.so.1
pcl_test: /usr/local/lib/libvtkCommonCore-6.1.so.1
pcl_test: CMakeFiles/pcl_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcl_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcl_test.dir/build: pcl_test
.PHONY : CMakeFiles/pcl_test.dir/build

CMakeFiles/pcl_test.dir/requires: CMakeFiles/pcl_test.dir/src/xtionviewer.cpp.o.requires
.PHONY : CMakeFiles/pcl_test.dir/requires

CMakeFiles/pcl_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl_test.dir/clean

CMakeFiles/pcl_test.dir/depend:
	cd /media/Stuff/bot/work/pcl_test-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/Stuff/bot/work/pcl_test /media/Stuff/bot/work/pcl_test /media/Stuff/bot/work/pcl_test-build /media/Stuff/bot/work/pcl_test-build /media/Stuff/bot/work/pcl_test-build/CMakeFiles/pcl_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl_test.dir/depend

