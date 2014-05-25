#-------------------------------------------------
#
# Project created by QtCreator 2014-04-15T08:52:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = calibration_omni
TEMPLATE = app


SOURCES += src/main.cpp\
           src/calibration_dialog.cpp \
           src/image_subscribe.cpp    \
           src/whites.cpp             \
           src/scanpoints.cpp          \
           src/omniimage.cpp


LIBS +=/opt/ros/groovy/lib/libroscpp.so          \
/usr/lib/x86_64-linux-gnu/libpthread.so          \
/usr/lib/libboost_signals-mt.so                  \
/usr/lib/libboost_filesystem-mt.so              \
/usr/lib/libboost_system-mt.so                   \
/opt/ros/groovy/lib/libcpp_common.so             \
/opt/ros/groovy/lib/libroscpp_serialization.so   \
/opt/ros/groovy/lib/librostime.so                \
/usr/lib/libboost_date_time-mt.so                \
/usr/lib/libboost_thread-mt.so                   \
/opt/ros/groovy/lib/librosconsole.so             \
/usr/lib/libboost_regex-mt.so                    \
/usr/lib/liblog4cxx.so                           \
/opt/ros/groovy/lib/libxmlrpcpp.so               \
/opt/ros/groovy/lib/libmessage_filters.so        \
/opt/ros/groovy/lib/libimage_transport.so        \
/usr/lib/libtinyxml.so                           \
/opt/ros/groovy/lib/libclass_loader.so           \
/usr/lib/libPocoFoundation.so                    \
/usr/lib/x86_64-linux-gnu/libdl.so               \
/opt/ros/groovy/lib/libconsole_bridge.so         \
/opt/ros/groovy/lib/libroslib.so                 \
/opt/ros/groovy/lib/libcv_bridge.so              \
/opt/ros/groovy/lib/libopencv_calib3d.so         \
/opt/ros/groovy/lib/libopencv_contrib.so         \
/opt/ros/groovy/lib/libopencv_core.so            \
/opt/ros/groovy/lib/libopencv_features2d.so      \
/opt/ros/groovy/lib/libopencv_flann.so           \
/opt/ros/groovy/lib/libopencv_gpu.so             \
/opt/ros/groovy/lib/libopencv_highgui.so         \
/opt/ros/groovy/lib/libopencv_imgproc.so         \
/opt/ros/groovy/lib/libopencv_legacy.so          \
/opt/ros/groovy/lib/libopencv_ml.so              \
/opt/ros/groovy/lib/libopencv_nonfree.so         \
/opt/ros/groovy/lib/libopencv_objdetect.so       \
/opt/ros/groovy/lib/libopencv_photo.so           \
/opt/ros/groovy/lib/libopencv_stitching.so       \
/opt/ros/groovy/lib/libopencv_superres.so        \
/opt/ros/groovy/lib/libopencv_video.so           \
/opt/ros/groovy/lib/libopencv_videostab.so       \
/opt/ros/groovy/lib/libimage_geometry.so

INCLUDEPATH+= /opt/ros/groovy/include  \
              /opt/ros/groovy/include/opencv \
              /usr/include    \
              include/core    \
              include

HEADERS  += include/calibration_dialog.h \
            include/image_subscribe.h    \
            include/whites.h             \
            include/scanpoints.h         \
            include/core/core.hpp        \
            include/omniimage.h

FORMS    += calibration_dialog.ui
