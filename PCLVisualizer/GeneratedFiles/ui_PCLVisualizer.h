/********************************************************************************
** Form generated from reading UI file 'PCLVisualizer.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVISUALIZER_H
#define UI_PCLVISUALIZER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLVisualizerClass
{
public:
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QPushButton *button_play;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PCLVisualizerClass)
    {
        if (PCLVisualizerClass->objectName().isEmpty())
            PCLVisualizerClass->setObjectName(QString::fromUtf8("PCLVisualizerClass"));
        PCLVisualizerClass->resize(829, 537);
        centralWidget = new QWidget(PCLVisualizerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, 0, 831, 501));
        button_play = new QPushButton(centralWidget);
        button_play->setObjectName(QString::fromUtf8("button_play"));
        button_play->setGeometry(QRect(0, 420, 41, 31));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(50, 430, 54, 12));
        PCLVisualizerClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PCLVisualizerClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 829, 23));
        PCLVisualizerClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PCLVisualizerClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PCLVisualizerClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PCLVisualizerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PCLVisualizerClass->setStatusBar(statusBar);

        retranslateUi(PCLVisualizerClass);

        QMetaObject::connectSlotsByName(PCLVisualizerClass);
    } // setupUi

    void retranslateUi(QMainWindow *PCLVisualizerClass)
    {
        PCLVisualizerClass->setWindowTitle(QApplication::translate("PCLVisualizerClass", "PCLVisualizer", nullptr));
        button_play->setText(QApplication::translate("PCLVisualizerClass", "play", nullptr));
        label->setText(QApplication::translate("PCLVisualizerClass", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PCLVisualizerClass: public Ui_PCLVisualizerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVISUALIZER_H
