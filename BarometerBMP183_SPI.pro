#-------------------------------------------------
#
# Project created by QtCreator 2014-11-15T08:42:25
#
#-------------------------------------------------

QT       += core
QT       += network
QT       -= gui
TARGET = BarometerBMP183_SPI
TEMPLATE = app

CONFIG   += console
CONFIG   += qt warn_on static
CONFIG   -= app_bundle
CONFIG   += static

OBJECTS_DIR     = tmp
MOC_DIR         = tmp
RCC_DIR         = tmp
UI_DIR          = tmp

message(Compiling Project $$TARGET)

SOURCES += src/main.cpp \
           src/BMP183.cpp

HEADERS += src/BMP183.h
