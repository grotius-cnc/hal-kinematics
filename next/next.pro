QT -= gui

TEMPLATE = lib
DEFINES += NEXT_LIBRARY

CONFIG += c++20

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    next.cpp

HEADERS += \
    next_global.h \
    next.h

INCLUDEPATH +=  /usr/include/ \
                /usr/local/include/opencascade/ \
                /usr/local/include/kdl/ \
                /usr/include/eigen3/ \
                /usr/local/lib/ \

LIBS += -L/usr/local/lib/ \

# project
INCLUDEPATH+=   libspline/ \
                libocct/ \
                libdxfrw/ \
                libscurve/ \
                libhal/ \
                libkinematic/ \
                libmotion/ \
                libcavalier/ \

# lcnc
INCLUDEPATH +=  /usr/lib/ \
                /opt/lib/linuxcnc/modules/ \
                /opt/include/ \
                /usr/include/linuxcnc/

#Hal
INCLUDEPATH+=   /opt/linuxcnc/include/ \
                /opt/linuxcnc/src/hal/ \


LIBS+= -L/opt/linuxcnc/lib/
LIBS+= -llinuxcnchal

#Opencascade
LIBS+= -L/usr/local/lib/ \

INCLUDEPATH +=  /usr/local/include/kdl/ \
                /usr/local/include/opencascade/ \
                /usr/include/eigen3/ \
                /usr/include/ \
                /usr/local/lib/ \

LIBS += -lorocos-kdl -llinuxcnchal -Iinclude -Isrc/emc/rs274ngc -Llib -lnml -llinuxcnc -llinuxcnchal -llinuxcncini -lposemath

# Opencascade
LIBS += -lTKPrim
LIBS += -lTKernel -lTKMath -lTKTopAlgo -lTKService
LIBS += -lTKG2d -lTKG3d -lTKV3d -lTKOpenGl
LIBS += -lTKBRep -lTKXSBase -lTKGeomBase
LIBS += -lTKMeshVS -lTKXSDRAW
LIBS += -lTKLCAF -lTKXCAF -lTKCAF
LIBS += -lTKCDF -lTKBin -lTKBinL -lTKBinXCAF -lTKXml -lTKXmlL -lTKXmlXCAF
# -- IGES support
LIBS += -lTKIGES
# -- STEP support
LIBS += -lTKSTEP -lTKXDESTEP -lTKXDEIGES
# -- STL support
LIBS += -lTKSTL
# -- OBJ/glTF support

LIBS += -lTKRWMesh

#src/base/io_occ_base_mesh.cpp \
#src/base/io_occ_gltf.cpp \
#src/base/io_occ_obj.cpp

# -- VRML support
LIBS += -lTKVRML


# this copies the configuration files etc to the build direcory. So user has only to edit the source directory.
copydata.commands = $(COPY_DIR) $$PWD/* $$OUT_PWD
first.depends = $(first) copydata
export(first.depends)
export(copydata.commands)
QMAKE_EXTRA_TARGETS += first copydata

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
